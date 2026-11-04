"""Probes that run outside ROS: ping and systemd. The monitor calls these on a
background thread so a slow answer never delays the checks."""

import os
import re
import subprocess

# Local test mode (set by local_test_env.sh): units that don't exist on this computer are
# reported as active, and units named in FAILED_ENV as failed, so the dashboard can be
# tried without the robot. Units that do exist here (e.g. tailscaled) are always real.
SIMULATE_ENV = 'HUSKY_HEALTH_SIMULATE_SERVICES'
FAILED_ENV = 'HUSKY_HEALTH_FAILED_SERVICES'

_RTT = re.compile(r'time[=<]([\d.]+)\s*ms')
_PROPERTIES = 'LoadState,ActiveState,SubState,Result,StatusText,ActiveEnterTimestamp'


def ping(host, timeout_s):
    """Round-trip time in ms, or None if the host didn't answer."""
    try:
        out = subprocess.run(['ping', '-n', '-c', '1', '-W', str(max(1, round(timeout_s))), host],
                             capture_output=True, text=True, timeout=timeout_s + 2)
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = _RTT.search(out.stdout)
    return float(match[1]) if out.returncode == 0 and match else None


def systemd_state(unit, user=False, timeout_s=3.0):
    """(properties, error): systemctl's properties for unit, e.g. {'ActiveState': 'active'}."""
    if unit in _units(os.environ.get(FAILED_ENV, '')):
        return _simulated('failed', 'failed', 'exit-code'), None
    cmd = ['systemctl', *(['--user'] if user else []), 'show', unit, f'--property={_PROPERTIES}']
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as exc:
        props, error = None, f'could not run systemctl: {exc}'
    else:
        if out.returncode == 0:
            props = dict(line.split('=', 1) for line in out.stdout.splitlines() if '=' in line)
            error = None
        else:
            props, error = None, out.stderr.strip() or f'systemctl exited with {out.returncode}'
    if os.environ.get(SIMULATE_ENV) and (props is None or props.get('LoadState') == 'not-found'):
        return _simulated('active', 'running', 'success'), None
    return props, error


def _units(text):
    return {u.strip() if '.' in u else u.strip() + '.service' for u in text.split(',') if u.strip()}


def _simulated(active, sub, result):
    return {'LoadState': 'loaded', 'ActiveState': active, 'SubState': sub, 'Result': result,
            'StatusText': 'simulated (local test mode)'}
