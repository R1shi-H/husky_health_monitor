"""Read-only HTTP server for the dashboard page and its JSON API.

Runs on a daemon thread next to the ROS executor. It only reads data the
monitor has already computed, so a slow browser can never stall the checks.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


class DashboardServer:

    def __init__(self, host, port, index_html, api):
        """api must provide snapshot(), history_query(seconds) and graph_listing()."""
        handler = _make_handler(Path(index_html), api)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        self._thread = threading.Thread(target=self.httpd.serve_forever, kwargs={'poll_interval': 0.2},
                                        name='health-web', daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def _make_handler(index_html, api):

    class Handler(BaseHTTPRequestHandler):
        server_version = 'HuskyHealth/1.0'

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ('/', '/index.html'):
                self._send(200, 'text/html; charset=utf-8', index_html.read_bytes())
            elif url.path == '/api/health':
                self._json(api.snapshot())
            elif url.path == '/api/history':
                try:
                    seconds = float(parse_qs(url.query).get('seconds', ['900'])[0])
                except ValueError:
                    seconds = 900.0
                self._json(api.history_query(max(10.0, min(seconds, 86400.0))))
            elif url.path == '/api/graph':
                self._json(api.graph_listing())
            else:
                self.send_error(404)

        def _json(self, data):
            self._send(200, 'application/json', json.dumps(data, separators=(',', ':')).encode())

        def _send(self, status, content_type, body):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass  # keep the ROS log for health events, not page loads

    return Handler
