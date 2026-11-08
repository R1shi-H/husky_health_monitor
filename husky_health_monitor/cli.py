"""Terminal view of the health monitor, in the spirit of voxl-inspect-services.

Reads the monitor's HTTP API, so it needs no ROS networking setup. On the robot:
    ros2 run husky_health_monitor health_status --watch
From a laptop on Tailscale (plain Python, no ROS needed):
    python3 cli.py --url http://<husky-tailscale-ip>:8088 --watch

Exit code (without --watch): 0 healthy, 1 warning, 2 error, 3 monitor unreachable.
"""

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

ICONS = {'ok': '✓', 'idle': '○', 'waiting': '…', 'warn': '⚠', 'error': '✗'}
COLORS = {'ok': '\033[32m', 'idle': '\033[2m', 'waiting': '\033[2m', 'warn': '\033[33m',
          'error': '\033[31m'}
BOLD, DIM, RESET = '\033[1m', '\033[2m', '\033[0m'
EXIT_CODES = {'ok': 0, 'idle': 0, 'waiting': 0, 'warn': 1, 'error': 2}
NAME_WIDTH = 32
WIDTH = 60


def fetch(url, timeout=3.0):
    with urllib.request.urlopen(url.rstrip('/') + '/api/health', timeout=timeout) as response:
        return json.load(response)


def render(snap, color=True, details=True):
    def paint(code, text):
        return f'{code}{text}{RESET}' if color else text

    def row(name, status, level, bold=False):
        name = name if len(name) <= NAME_WIDTH - 1 else name[:NAME_WIDTH - 2] + '…'
        code = COLORS[level] + (BOLD if bold else '')
        return f'{name:<{NAME_WIDTH}}' + paint(code, f'{ICONS[level]} {status}')

    stamp = time.strftime('%H:%M:%S', time.localtime(snap['time']))
    lines = [paint(BOLD, 'HUSKY A300 SYSTEM HEALTH'),
             paint(DIM, f'{snap["robot"]} on {snap.get("hostname") or "?"} at {stamp}'),
             '=' * WIDTH]
    for group in snap['groups']:
        lines.append(paint(BOLD, f'{group["name"].upper():<{NAME_WIDTH}}STATUS'))
        lines.append('-' * WIDTH)
        for check in group['checks']:
            lines.append(row(check['name'], check['value'], check['level']))
            shown = check['level'] in ('warn', 'error') or check.get('show_detail')
            if details and shown and check.get('detail'):
                lines.append(paint(DIM, f'  └ {check["detail"]}'))
        lines.append('')
    overall = snap['overall']
    counts = overall['counts']
    lines += ['=' * WIDTH,
              row('OVERALL SYSTEM STATUS:', overall['label'], overall['level'], bold=True),
              paint(DIM, f'{counts["ok"] + counts["idle"]} healthy · {counts["warn"]} warning · '
                         f'{counts["error"]} error · {snap["graph"]["nodes"]} nodes / '
                         f'{snap["graph"]["topics"]} topics visible')]
    return '\n'.join(lines)


def _suggest(hz):
    """Round a measured rate down to a tidy expected_hz value."""
    for step in (100, 50, 10, 5, 1, 0.5, 0.1):
        if hz >= step * 2:
            return round(hz / step) * step
    return round(hz, 2)


def calibrate(snap):
    """Measured vs configured rates, to help set expected_hz on the real robot."""
    lines = [f'{"CHECK":<28}{"MEASURED":>10}{"CONFIGURED":>12}{"SUGGESTED":>11}',
             '-' * 61]
    for group in snap['groups']:
        for check in group['checks']:
            if 'hz' not in check or check.get('latched'):
                continue
            measured = check['hz']
            configured = check.get('expected_hz')
            suggested = _suggest(measured) if measured > 0 else None
            flag = ''
            if configured and suggested and measured < 0.8 * configured:
                flag = '  < below config'
            lines.append(f'{check["name"][:27]:<28}{measured:>9.1f} '
                         f'{configured if configured is not None else "-":>11} '
                         f'{suggested if suggested is not None else "-":>10}{flag}')
    lines += ['', 'Rates are averaged over each check\'s window (5 s or more). Put the suggested',
              'values in the config as expected_hz: warn fires below 80%, error below 50%.']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Show Husky health from the health monitor.')
    parser.add_argument('--url', default=os.environ.get('HUSKY_HEALTH_URL', 'http://localhost:8088'),
                        help='monitor address (default: $HUSKY_HEALTH_URL or http://localhost:8088)')
    parser.add_argument('-w', '--watch', nargs='?', const=1.0, type=float, metavar='SECONDS',
                        help='refresh every SECONDS (default 1)')
    parser.add_argument('--brief', action='store_true', help='hide the reason under warnings/errors')
    parser.add_argument('--no-color', action='store_true')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--json', action='store_true', help='print the raw JSON snapshot')
    mode.add_argument('--calibrate', action='store_true',
                      help='compare measured topic rates with the configured expected_hz')
    args, _ = parser.parse_known_args(argv)  # ignore --ros-args from `ros2 run`
    color = sys.stdout.isatty() and not args.no_color

    try:
        while True:
            try:
                snap = fetch(args.url)
            except (urllib.error.URLError, OSError, ValueError) as exc:
                reason = getattr(exc, 'reason', exc)
                output = (f'Cannot reach the health monitor at {args.url} ({reason}).\n'
                          'Is it running? Start it on this computer with run_local_test.sh, or '
                          'point --url at the robot: --url http://<husky-tailscale-ip>:8088')
                code = 3
            else:
                code = EXIT_CODES[snap['overall']['level']]
                if args.json:
                    output = json.dumps(snap, indent=2)
                elif args.calibrate:
                    output = calibrate(snap)
                else:
                    output = render(snap, color, details=not args.brief)
            if not args.watch:
                print(output, file=sys.stderr if code == 3 else sys.stdout)
                return code
            sys.stdout.write('\033[H\033[2J' + output + '\n')
            sys.stdout.flush()
            time.sleep(args.watch)
    except KeyboardInterrupt:
        return 0


if __name__ == '__main__':
    sys.exit(main())
