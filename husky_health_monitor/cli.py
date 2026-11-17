"""Terminal view of the health monitor, in the spirit of voxl-inspect-services.

Reads the monitor's HTTP API, so it needs no ROS networking setup. On the robot:
    ros2 run husky_health_monitor health_status --watch
From a laptop on Tailscale (plain Python, no ROS needed):
    python3 cli.py --url http://<husky-tailscale-ip>:8088 --watch

Exit code (without --watch): 0 healthy, 1 warning, 2 error, 3 monitor unreachable.
"""

import argparse
import itertools
import json
import os
import shutil
import sys
import textwrap
import time
import urllib.error
import urllib.request

ICONS = {'ok': '✓', 'idle': '○', 'waiting': '…', 'warn': '⚠', 'error': '✗'}
COLORS = {'ok': '\033[32m', 'idle': '\033[2m', 'waiting': '\033[2m', 'warn': '\033[33m',
          'error': '\033[31m'}
BOLD, DIM, RESET = '\033[1m', '\033[2m', '\033[0m'
EXIT_CODES = {'ok': 0, 'idle': 0, 'waiting': 0, 'warn': 1, 'error': 2}
GAP = 4          # spaces between columns
MIN_COLUMN = 48  # narrowest column; a terminal fits as many as it can, up to MAX_COLUMNS
MAX_COLUMN = 56
MAX_COLUMNS = 3
WIDTH = 60       # the --calibrate table


def fetch(url, timeout=3.0):
    with urllib.request.urlopen(url.rstrip('/') + '/api/health', timeout=timeout) as response:
        return json.load(response)


def _fit(text, width):
    return text if len(text) <= width else text[:max(0, width - 1)] + '…'


def render(snap, color=True, details=False, width=None):
    """Overall status first, then every check, in as many columns as the terminal fits
    (one in a narrow terminal, up to three). With details, the reason goes under each
    warning or error."""
    def paint(code, text):
        return f'{code}{text}{RESET}' if color else text

    if width is None:
        width = shutil.get_terminal_size((80, 24)).columns
    columns = max(1, min(MAX_COLUMNS, (width + GAP) // (MIN_COLUMN + GAP)))
    col = width if columns == 1 else min(MAX_COLUMN, (width + GAP) // columns - GAP)
    name_width = min(24, 1 + max((len(c['name']) for g in snap['groups'] for c in g['checks']),
                                 default=10))

    # Lines are (plain, painted) pairs, so columns can be padded by their visible width
    blocks = []
    for group in snap['groups']:
        title = group['name'].upper()
        block = [(title, paint(BOLD, title))]
        for check in group['checks']:
            level = check['level']
            name = f' {_fit(check["name"], name_width - 1):<{name_width}}'
            status = _fit(f'{ICONS[level]} {check["value"]}', col - len(name))
            block.append((name + status, name + paint(COLORS[level], status)))
            shown = level in ('warn', 'error') or check.get('show_detail')
            if details and shown and check.get('detail'):
                for line in textwrap.wrap(check['detail'], col - 5, initial_indent='   └ ',
                                          subsequent_indent='     '):
                    block.append((line, paint(DIM, line)))
        blocks.append(block)

    col = min(col, max(len(plain) for block in blocks for plain, _ in block))  # no wider than needed
    # Keep the groups in order, split where the columns come out closest in height
    columns = min(columns, len(blocks)) or 1
    heights = [len(b) for b in blocks]
    cuts = min(itertools.combinations(range(1, len(blocks)), columns - 1),
               key=lambda c: max(sum(heights[a:b]) for a, b in zip((0, *c), (*c, len(blocks)))))
    stacks = [[line for block in blocks[a:b] for line in block]
              for a, b in zip((0, *cuts), (*cuts, len(blocks)))]
    body = []
    for row in itertools.zip_longest(*stacks, fillvalue=('', '')):
        line = ''
        for i, (plain, painted) in enumerate(row):
            pad = ' ' * (col + GAP - len(plain)) if i < len(row) - 1 else ''
            line += painted + pad
        body.append(line.rstrip())

    overall = snap['overall']
    counts = overall['counts']
    level = overall['level']
    stamp = time.strftime('%H:%M:%S', time.localtime(snap['time']))
    lines = [paint(BOLD, 'HUSKY A300 HEALTH') +
             paint(DIM, f'  {snap["robot"]} on {snap.get("hostname") or "?"} · {stamp}'),
             paint(COLORS[level] + BOLD, f'{ICONS[level]} {overall["label"]}') +
             paint(DIM, f'  {counts["ok"] + counts["idle"]} healthy · {counts["warn"]} warning · '
                        f'{counts["error"]} error · {snap["graph"]["nodes"]} nodes / '
                        f'{snap["graph"]["topics"]} topics'),
             '']
    return '\n'.join(lines + body)


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
    parser.add_argument('-d', '--details', action='store_true',
                        help='also show the reason under each warning or error')
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
                    output = render(snap, color, details=args.details)
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
