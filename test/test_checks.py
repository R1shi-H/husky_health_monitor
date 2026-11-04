import math
import subprocess
from pathlib import Path
from types import SimpleNamespace as Msg

import pytest
import yaml

from husky_health_monitor import probes
from husky_health_monitor.checks import (ERROR, IDLE, OK, WAITING, WARN, ConfigError, Context,
                                         GraphInfo, build_groups, effective_level, get_field,
                                         resolve_name)

NS = 'a300_00095'
CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'a300_00095.yaml'


def make(spec, group='Test'):
    return build_groups({'groups': [{'name': group, 'checks': [spec]}]}, NS)[0][1][0]


def feed(check, hz, seconds, start=0.0, msg=None):
    """Deliver messages at hz for seconds; returns the time of the last one."""
    t = start
    for i in range(int(hz * seconds)):
        t = start + i / hz
        check.on_message(t, msg)
    return t


def graph(**publishers):
    return GraphInfo(topics={t: ['x/msg/Y'] for t in publishers}, publishers=publishers)


def test_shipped_config_builds():
    config = yaml.safe_load(CONFIG.read_text())
    groups = build_groups(config, config['robot_namespace'])
    ids = [c.id for _, checks in groups for c in checks]
    assert len(ids) == len(set(ids))
    assert any(c.topic == '/a300_00095/platform/bms/state' for _, checks in groups for c in checks)


def test_describe_does_not_shadow_live_fields():
    # The dashboard reads id/name/level/value/detail/metric from the same row as describe()
    config = yaml.safe_load(CONFIG.read_text())
    live = {'id', 'name', 'level', 'value', 'detail', 'metric'}
    for _, checks in build_groups(config, config['robot_namespace']):
        for check in checks:
            assert not live & check.describe().keys(), check.name


def test_resolve_name():
    assert resolve_name('platform/odom', NS) == '/a300_00095/platform/odom'
    assert resolve_name('/tf', NS) == '/tf'
    assert resolve_name('*atak*', NS) == '*atak*'


def test_get_field_paths():
    msg = Msg(status=Msg(status=2), buttons=[0, 1, 0])
    assert get_field(msg, 'status.status') == 2
    assert get_field(msg, 'buttons[1]') == 1


def test_rate_ok_warn_error():
    check = make({'name': 'IMU', 'type': 'rate', 'topic': 'imu', 'expected_hz': 100})
    last = feed(check, 100, 6)
    result = check.evaluate(Context(now=last + 0.01))
    assert result.level == OK and result.value == '100 Hz'

    slow = make({'name': 'IMU', 'type': 'rate', 'topic': 'imu', 'expected_hz': 100})
    last = feed(slow, 70, 6)
    assert slow.evaluate(Context(now=last + 0.01)).level == WARN

    slower = make({'name': 'IMU', 'type': 'rate', 'topic': 'imu', 'expected_hz': 100})
    last = feed(slower, 30, 6)
    result = slower.evaluate(Context(now=last + 0.01))
    assert result.level == ERROR and 'rate' in result.detail


def test_stale_and_missing():
    check = make({'name': 'GPS', 'type': 'rate', 'topic': 'fix', 'expected_hz': 5})
    assert check.evaluate(Context(now=1.0, in_grace=True)).level == WAITING
    no_pub = check.evaluate(Context(now=20.0, graph=graph()))
    assert (no_pub.level, no_pub.value) == (ERROR, 'NO PUBLISHER')
    no_data = check.evaluate(Context(now=20.0, graph=graph(**{'/a300_00095/fix': 1})))
    assert no_data.value == 'NO DATA'

    last = feed(check, 5, 3)
    stale = check.evaluate(Context(now=last + 5))
    assert (stale.level, stale.value) == (ERROR, 'STALE')


def test_missing_label_and_level():
    check = make({'name': 'Joy', 'topic': 'joy', 'timeout_s': 2, 'ok_label': 'CONNECTED',
                  'missing_label': 'DISCONNECTED', 'missing_level': 'warn'})
    last = feed(check, 20, 2)
    assert check.evaluate(Context(now=last + 0.1)).value == 'CONNECTED'
    result = check.evaluate(Context(now=last + 3))
    assert (result.level, result.value) == (WARN, 'DISCONNECTED')


def test_idle_topic():
    check = make({'name': 'cmd_vel', 'topic': 'cmd_vel', 'idle_ok': True, 'ok_label': 'ACTIVE',
                  'timeout_s': 1})
    assert check.evaluate(Context(now=30.0)).level == IDLE
    last = feed(check, 20, 2)
    assert check.evaluate(Context(now=last + 0.1)).value == 'ACTIVE'
    assert check.evaluate(Context(now=last + 5)).value == 'IDLE'


def test_latched_topic_never_stale():
    check = make({'name': 'Desc', 'topic': 'robot_description', 'timeout_s': 1})
    check.latched = True
    check.on_message(0.0)
    assert check.evaluate(Context(now=100.0)).level == OK


@pytest.mark.parametrize('pct, level', [(0.78, OK), (0.25, WARN), (0.10, ERROR), (78.0, OK)])
def test_battery_levels(pct, level):
    check = make({'name': 'Battery', 'type': 'battery', 'topic': 'bms/state', 'timeout_s': 10})
    check.on_message(0.0, Msg(percentage=pct, voltage=25.6, current=-3.0, power_supply_status=2))
    result = check.evaluate(Context(now=1.0))
    assert result.level == level
    assert result.value == f'{pct * 100 if pct <= 1 else pct:.0f}%'


def test_stale_value_check_leaves_trend_gap():
    battery = make({'name': 'Battery', 'type': 'battery', 'topic': 'bms/state', 'timeout_s': 10})
    battery.on_message(0.0, Msg(percentage=0.8, voltage=25.6, current=-3.0))
    stale = battery.evaluate(Context(now=30.0))
    assert stale.value == 'STALE' and stale.metric is None

    rate = make({'name': 'IMU', 'topic': 'imu', 'expected_hz': 100})
    feed(rate, 100, 1)
    assert rate.evaluate(Context(now=30.0)).metric == 0.0


def test_battery_without_percentage_reports_voltage():
    check = make({'name': 'Battery', 'type': 'battery', 'topic': 'bms/state'})
    check.on_message(0.0, Msg(percentage=float('nan'), voltage=25.1, current=float('nan')))
    result = check.evaluate(Context(now=1.0))
    assert (result.level, result.value) == (WARN, '25.1 V')


def test_navsat_fix_quality():
    check = make({'name': 'GPS', 'type': 'navsat', 'topic': 'fix', 'expected_hz': 5})
    last = feed(check, 5, 3, msg=Msg(status=Msg(status=2)))
    assert check.evaluate(Context(now=last + 0.1)).value == '5.0 Hz · RTK FIXED'
    check.on_message(last + 0.2, Msg(status=Msg(status=-1)))
    assert check.evaluate(Context(now=last + 0.3)).level == WARN


def test_bool_estop():
    check = make({'name': 'E-stop', 'type': 'bool', 'topic': 'estop', 'ok_value': False,
                  'true_label': 'ENGAGED', 'false_label': 'RELEASED'})
    check.on_message(0.0, Msg(data=False))
    assert check.evaluate(Context(now=1.0)).value == 'RELEASED'
    check.on_message(1.0, Msg(data=True))
    result = check.evaluate(Context(now=1.5))
    assert (result.level, result.value) == (WARN, 'ENGAGED')


def test_field_thresholds():
    check = make({'name': 'Temp', 'type': 'field', 'topic': 'mcu/temp', 'field': 'pcb',
                  'unit': ' °C', 'warn_above': 60, 'error_above': 75})
    for value, level in ((40.0, OK), (65.0, WARN), (80.0, ERROR)):
        check.on_message(0.0, Msg(pcb=value))
        assert check.evaluate(Context(now=0.1)).level == level


def test_diagnostics_array_uses_leaves():
    check = make({'name': 'Diag', 'type': 'diagnostics', 'topic': 'diagnostics_agg'})
    status = [Msg(name='/Platform', level=b'\x01', message=''),
              Msg(name='/Platform/Motors', level=b'\x01', message='hot'),
              Msg(name='/Sensors/IMU', level=b'\x00', message='OK')]
    check.on_message(0.0, Msg(status=status))
    result = check.evaluate(Context(now=0.1))
    assert result.level == WARN
    assert [item['name'] for item in result.items] == ['/Platform/Motors']


def test_node_patterns():
    check = make({'name': 'ATAK', 'type': 'node', 'node': '*atak*'})
    found = check.evaluate(Context(now=10, graph=GraphInfo(nodes={'/a300_00095/atak_bridge'})))
    assert found.level == OK
    lost = check.evaluate(Context(now=10, graph=GraphInfo(nodes={'/a300_00095/other'})))
    assert (lost.level, lost.value) == (ERROR, 'NOT RUNNING')

    relative = make({'name': 'CM', 'type': 'node', 'node': 'controller_manager'})
    ok = relative.evaluate(Context(now=10, graph=GraphInfo(nodes={'/a300_00095/controller_manager'})))
    assert ok.level == OK


def test_ros_graph_flags_empty_graph():
    check = make({'name': 'Graph', 'type': 'ros_graph', 'min_nodes': 3})
    lonely = GraphInfo(nodes={'/me'}, own_node='/me')
    result = check.evaluate(Context(now=20, graph=lonely))
    assert result.level == ERROR and 'ROS_SUPER_CLIENT' in result.detail


def test_system_check():
    check = make({'name': 'CPU', 'type': 'system', 'metric': 'cpu'})
    assert check.evaluate(Context(now=0, system={'cpu': 50.0})).level == OK
    assert check.evaluate(Context(now=0, system={'cpu': 90.0})).level == WARN
    assert check.evaluate(Context(now=0, system={})).level == IDLE


def test_interface_check():
    check = make({'name': 'Tailscale', 'type': 'interface', 'interface': 'tailscale0'})
    up = {'interfaces': {'tailscale0': {'up': True, 'ipv4': '100.64.0.10'}}}
    assert check.evaluate(Context(now=0, system=up)).value == '100.64.0.10'
    down = {'interfaces': {'tailscale0': {'up': False, 'ipv4': ''}}}
    assert check.evaluate(Context(now=0, system=down)).value == 'DOWN'


def test_non_critical_errors_only_warn():
    check = make({'name': 'Cam', 'topic': 'cam', 'expected_hz': 10, 'critical': False})
    result = check.evaluate(Context(now=30, graph=graph()))
    assert result.level == ERROR
    assert effective_level(check, result) == WARN


@pytest.mark.parametrize('spec, message', [
    ({'name': 'X', 'type': 'bogus'}, 'unknown check type'),
    ({'name': 'X', 'type': 'rate'}, 'need a topic'),
    ({'type': 'rate', 'topic': 't'}, 'has no name'),
    ({'name': 'X', 'type': 'system', 'metric': 'gpu'}, 'metric must be'),
])
def test_config_errors(spec, message):
    with pytest.raises(ConfigError, match=message):
        make(spec)


def test_navsat_custom_fix_labels_and_min_status():
    check = make({'name': 'GPS', 'type': 'navsat', 'topic': 'fix', 'min_status': 2,
                  'fix_labels': {2: 'RTK'}})
    check.on_message(0.0, Msg(status=Msg(status=1)))
    result = check.evaluate(Context(now=0.1))
    assert (result.level, result.value) == (WARN, 'RTK FLOAT')
    check.on_message(0.2, Msg(status=Msg(status=2)))
    assert check.evaluate(Context(now=0.3)).value == 'RTK'


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return Msg(w=cr * cp * cy + sr * sp * sy, x=sr * cp * cy - cr * sp * sy,
               y=cr * sp * cy + sr * cp * sy, z=cr * cp * sy - sr * sp * cy)


def odometry(roll, pitch, yaw):
    return Msg(pose=Msg(pose=Msg(orientation=quaternion(roll, pitch, yaw))))


def test_orientation_reports_yaw_roll_pitch():
    check = make({'name': 'IMU', 'type': 'orientation', 'topic': 'odom', 'warn_tilt_deg': 25})
    check.on_message(0.0, odometry(-1.0, 2.0, 90.0))
    result = check.evaluate(Context(now=0.1))
    assert (result.level, result.value) == (OK, 'yaw 90°')
    assert result.detail == 'roll -1.0° · pitch +2.0°'
    assert result.metric == pytest.approx(2.0)


def test_orientation_tilt_and_empty_quaternion():
    check = make({'name': 'IMU', 'type': 'orientation', 'topic': 'odom',
                  'warn_tilt_deg': 25, 'error_tilt_deg': 35})
    check.on_message(0.0, odometry(0.0, 30.0, 0.0))
    assert check.evaluate(Context(now=0.1)).level == WARN
    check.on_message(0.2, odometry(40.0, 0.0, 0.0))
    assert check.evaluate(Context(now=0.3)).level == ERROR
    check.on_message(0.4, Msg(pose=Msg(pose=Msg(orientation=Msg(x=0.0, y=0.0, z=0.0, w=0.0)))))
    assert check.evaluate(Context(now=0.5)).value == 'NO ORIENTATION'


def test_bool_any_value_is_never_a_fault():
    check = make({'name': 'Tracking', 'type': 'bool', 'topic': 'object_tracking/enabled',
                  'ok_value': 'any', 'true_label': 'ENABLED', 'false_label': 'DISABLED',
                  'idle_ok': True, 'idle_label': 'NOT RUNNING'})
    assert check.evaluate(Context(now=30, graph=graph())).value == 'NOT RUNNING'
    for value, label in ((True, 'ENABLED'), (False, 'DISABLED')):
        check.on_message(1.0, Msg(data=value))
        assert check.evaluate(Context(now=1.1)).level == OK
        assert check.evaluate(Context(now=1.1)).value == label


def test_idle_detail_counts_publishers():
    check = make({'name': 'Steam Deck', 'topic': 'operators/steamdeck_1/joy', 'idle_ok': True,
                  'idle_label': 'NOT CONNECTED', 'timeout_s': 2})
    result = check.evaluate(Context(now=30, graph=graph(**{'/a300_00095/operators/steamdeck_1/joy': 1})))
    assert (result.level, result.value) == (IDLE, 'NOT CONNECTED')
    assert '1 publisher' in result.detail


SYSTEMD = {'LoadState': 'loaded', 'Result': 'success', 'StatusText': '',
           'ActiveEnterTimestamp': 'Mon 2026-11-02 16:43:05 EST'}


@pytest.mark.parametrize('active, sub, level, value', [
    ('active', 'running', OK, 'ACTIVE'),
    ('activating', 'start', WARN, 'ACTIVATING'),
    ('failed', 'failed', ERROR, 'FAILED'),
    ('inactive', 'dead', ERROR, 'INACTIVE'),
])
def test_systemd_states(active, sub, level, value):
    check = make({'name': 'ATAK', 'type': 'systemd', 'unit': 'mors@atak_bridge'})
    assert check.unit == 'mors@atak_bridge.service'
    assert check.evaluate(Context(now=0)).level == WAITING
    check.record({**SYSTEMD, 'ActiveState': active, 'SubState': sub})
    result = check.evaluate(Context(now=1))
    assert (result.level, result.value) == (level, value)
    if active == 'active':
        assert 'since 2026-11-02 16:43' in result.detail


def test_systemd_missing_unit_and_status_text():
    check = make({'name': 'Tailscale', 'type': 'systemd', 'unit': 'tailscaled.service',
                  'missing_level': 'warn'})
    check.record({'LoadState': 'not-found', 'ActiveState': 'inactive'})
    assert check.evaluate(Context(now=1)).value == 'NOT FOUND'
    check.record({**SYSTEMD, 'ActiveState': 'active', 'SubState': 'running',
                  'StatusText': 'Connected; user@; 100.64.0.10'})
    assert 'Connected; user@; 100.64.0.10' in check.evaluate(Context(now=2)).detail
    check.record(None, 'could not run systemctl')
    assert check.evaluate(Context(now=3)).value == 'UNKNOWN'


def fake_systemctl(load_state):
    def run(cmd, **kwargs):
        return subprocess.CompletedProcess(cmd, 0, f'LoadState={load_state}\nActiveState=inactive\n', '')
    return run


def test_probe_simulates_missing_units_in_local_test_mode(monkeypatch):
    monkeypatch.setattr(probes.subprocess, 'run', fake_systemctl('not-found'))
    monkeypatch.delenv(probes.SIMULATE_ENV, raising=False)
    monkeypatch.delenv(probes.FAILED_ENV, raising=False)
    props, _ = probes.systemd_state('mors@camera_streams.service')
    assert props['LoadState'] == 'not-found'

    monkeypatch.setenv(probes.SIMULATE_ENV, '1')
    props, _ = probes.systemd_state('mors@camera_streams.service')
    assert props['ActiveState'] == 'active' and 'simulated' in props['StatusText']

    monkeypatch.setenv(probes.FAILED_ENV, 'mors@atak_bridge')
    props, _ = probes.systemd_state('mors@atak_bridge.service')
    assert props['ActiveState'] == 'failed'


def test_probe_never_fakes_installed_units(monkeypatch):
    monkeypatch.setattr(probes.subprocess, 'run', fake_systemctl('loaded'))
    monkeypatch.setenv(probes.SIMULATE_ENV, '1')
    props, _ = probes.systemd_state('tailscaled.service')
    assert props['ActiveState'] == 'inactive'
