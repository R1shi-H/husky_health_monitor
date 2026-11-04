"""ROS 2 node that runs the configured health checks and serves the dashboard.

Passive by design: it only subscribes (best effort, so publishers never wait on
it), never publishes on robot topics, and rate-only checks use raw subscriptions
so messages are counted without being deserialized.
"""

import os
import sys
import threading
import time
from functools import partial
from pathlib import Path

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from .checks import (ERROR, IDLE, OK, OVERALL_LABEL, SEVERITY, WAITING, WARN, ConfigError,
                     Context, GraphInfo, PingCheck, Result, SystemCheck, SystemdCheck,
                     TopicCheck, TopicGraphCheck, build_groups, effective_level, overall_level)
from .history import History
from .probes import SIMULATE_ENV, ping, systemd_state
from .system_stats import SystemSampler
from .web_server import DashboardServer

PACKAGE = 'husky_health_monitor'
DIAG_LEVEL = {OK: 0, IDLE: 0, WARN: 1, ERROR: 2, WAITING: 3}

STREAM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(PACKAGE))
    except (ImportError, KeyError, ValueError):
        return Path(__file__).resolve().parent.parent  # running from the source tree


class HealthMonitor(Node):

    def __init__(self):
        super().__init__('health_monitor')
        share = share_dir()
        config_path = self.declare_parameter(
            'config_file', str(share / 'config' / 'a300_00095.yaml')).value
        namespace = self.declare_parameter('robot_namespace', '').value
        port = self.declare_parameter('web_port', -1).value

        self.config_path = os.path.expanduser(config_path)
        with open(self.config_path, encoding='utf-8') as f:
            config = yaml.safe_load(f) or {}
        self.namespace = (namespace or config.get('robot_namespace') or self.get_namespace()).strip('/')
        self.groups = build_groups(config, self.namespace)
        self.checks = [check for _, checks in self.groups for check in checks]
        self.update_period = 1.0 / float(config.get('update_rate_hz', 1.0))
        self.grace_s = float(config.get('startup_grace_s', 8.0))
        self.executor_kind = config.get('executor', 'events')
        self.started = time.monotonic()

        self.topic_checks = {}
        for check in self.checks:
            if isinstance(check, TopicCheck):
                self.topic_checks.setdefault(check.topic, []).append(check)
        self.monitored_topics = set(self.topic_checks) | {
            c.graph_topic for c in self.checks if isinstance(c, TopicGraphCheck)}
        self._topic_subs = {}      # topic -> Subscription, or None if its type can't be loaded
        self.graph = None
        self.graph_list = {'nodes': [], 'topics': []}
        self.levels = {check.id: WAITING for check in self.checks}
        self._snapshot = {}
        self._stop = threading.Event()

        self.sampler = SystemSampler(
            [c.path for c in self.checks if isinstance(c, SystemCheck) and c.metric == 'disk'])
        hist = config.get('history') or {}
        self.history = History(
            [check.id for check in self.checks], self.update_period,
            memory_minutes=float(hist.get('memory_minutes', 15)),
            log_dir=hist.get('log_dir', '~/.ros/husky_health'),
            log_period_s=float(hist.get('log_period_s', 10)),
            keep_days=int(hist.get('keep_days', 7)),
            warn=self.get_logger().warning)

        self.status_pub = self.create_publisher(DiagnosticArray, 'health_monitor/status', 10)
        self.create_timer(float(config.get('graph_period_s', 3.0)), self.refresh_graph)
        self.create_timer(self.update_period, self.update)
        self._start_probes()

        self.web = None
        web = config.get('web') or {}
        if web.get('enabled', True):
            host = str(web.get('host', '0.0.0.0'))
            port = port if port > 0 else int(web.get('port', 8088))
            try:
                self.web = DashboardServer(host, port, share / 'web' / 'index.html', self)
                self.web.start()
                shown = '<robot-ip>' if host == '0.0.0.0' else host
                self.get_logger().info(f'dashboard: http://{shown}:{port}/')
            except OSError as exc:
                self.get_logger().error(f'could not start the dashboard on {host}:{port}: {exc}')

        self._check_environment()
        self.get_logger().info(
            f'monitoring {len(self.checks)} checks in {len(self.groups)} groups '
            f'(namespace /{self.namespace}, config {self.config_path})')
        self.refresh_graph()

    # ---- ROS graph and subscriptions -------------------------------------------------------

    def refresh_graph(self):
        topics = dict(self.get_topic_names_and_types())
        nodes = {('' if ns == '/' else ns) + '/' + name
                 for name, ns in self.get_node_names_and_namespaces()}
        mine = {topic for topic, sub in self._topic_subs.items() if sub is not None}
        publishers = {t: self.count_publishers(t) for t in topics}
        subscribers = {t: self.count_subscribers(t) - (t in mine) for t in topics}
        self.graph = GraphInfo(nodes, topics, publishers, subscribers,
                               self.get_fully_qualified_name())
        self._ensure_subscriptions()
        self.graph_list = {
            'nodes': sorted(nodes),
            'topics': [{'name': t, 'type': ', '.join(types), 'publishers': publishers[t],
                        'subscribers': subscribers[t], 'monitored': t in self.monitored_topics}
                       for t, types in sorted(topics.items())],
        }

    def _ensure_subscriptions(self):
        for topic, checks in self.topic_checks.items():
            if topic in self._topic_subs:
                continue
            # An explicit msg_type lets us subscribe before the topic shows up in discovery
            type_name = next((c.msg_type for c in checks if c.msg_type), None)
            if type_name is None:
                types = self.graph.topics.get(topic)
                if not types:
                    continue
                type_name = types[0]
            self._subscribe(topic, checks, type_name)

    def _subscribe(self, topic, checks, type_name):
        try:
            msg_class = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            for check in checks:
                check.type_error = (f'{type_name} is not installed on this computer; '
                                    'source the workspace that provides it')
            self._topic_subs[topic] = None
            self.get_logger().warning(f'cannot subscribe to {topic}: {type_name} is not installed')
            return
        latched = self._is_latched(topic)
        raw = not any(check.needs_message for check in checks)
        self._topic_subs[topic] = self.create_subscription(
            msg_class, topic, partial(self._on_message, checks, raw),
            LATCHED_QOS if latched else STREAM_QOS, raw=raw)
        for check in checks:
            check.msg_type = type_name
            check.latched = latched
            check.type_error = None

    def _is_latched(self, topic):
        infos = self.get_publishers_info_by_topic(topic)
        return bool(infos) and all(
            info.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
            and info.qos_profile.reliability == ReliabilityPolicy.RELIABLE for info in infos)

    def _on_message(self, checks, raw, msg):
        now = time.monotonic()
        if raw:
            for check in checks:
                check.on_message(now, None, len(msg))
        else:
            for check in checks:
                check.on_message(now, msg)

    # ---- evaluation ------------------------------------------------------------------------

    def update(self):
        now = time.monotonic()
        system = self.sampler.sample()
        ctx = Context(now, now - self.started < self.grace_s, self.graph, system)
        results = {}
        for check in self.checks:
            try:
                result = check.evaluate(ctx)
            except Exception as exc:  # one broken check must not take the monitor down
                result = Result(ERROR, 'CHECK FAILED', f'{type(exc).__name__}: {exc}')
            results[check.id] = (check, result)

        wall = time.time()
        levels = {cid: effective_level(check, result) for cid, (check, result) in results.items()}
        overall = overall_level(list(levels.values()))
        self._log_transitions(wall, results, levels)
        self.history.record(wall, overall, {cid: (levels[cid], result.value, result.metric)
                                            for cid, (_, result) in results.items()}, system)
        self._snapshot = self._build_snapshot(wall, now, overall, results, levels, system)
        self._publish(overall, results, levels)

    def _log_transitions(self, wall, results, levels):
        for cid, (check, result) in results.items():
            old, new = self.levels[cid], levels[cid]
            if old == new:
                continue
            self.levels[cid] = new
            # ok <-> idle (e.g. cmd_vel starting and stopping) and normal startup aren't events
            if SEVERITY[new] == 0 and SEVERITY[old] <= SEVERITY[WAITING]:
                continue
            self.history.event(wall, check, old, new, result)
            text = f'{check.group} / {check.name}: {old.upper()} -> {new.upper()} ({result.value})'
            if result.detail:
                text += f' {result.detail}'
            if SEVERITY[new] >= SEVERITY[WARN]:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    def _build_snapshot(self, wall, now, overall, results, levels, system):
        counts = dict.fromkeys((OK, IDLE, WAITING, WARN, ERROR), 0)
        groups = []
        for name, checks in self.groups:
            rows = []
            for check in checks:
                result = results[check.id][1]
                counts[levels[check.id]] += 1
                # live fields last, so a check setting can never hide them
                row = {**check.describe(), 'id': check.id, 'name': check.name,
                       'level': levels[check.id], 'value': result.value,
                       'detail': result.detail, 'metric': result.metric}
                if isinstance(check, TopicCheck):
                    age = check.tracker.age(now)
                    row.update(
                        msg_type=check.msg_type, latched=check.latched,
                        hz=round(check.tracker.rate(now), 2),
                        age_s=None if age is None else round(age, 2),
                        bandwidth=round(check.tracker.bandwidth(now)),
                        received=check.tracker.total,
                        publishers=self.graph.publishers.get(check.topic, 0) if self.graph else None)
                if result.items:
                    row['items'] = result.items
                rows.append(row)
            groups.append({'name': name, 'level': overall_level([levels[c.id] for c in checks]),
                           'checks': rows})
        return {
            'robot': self.namespace,
            'hostname': system.get('hostname'),
            'time': wall,
            'update_period_s': self.update_period,
            'overall': {'level': overall, 'label': OVERALL_LABEL[overall], 'counts': counts},
            'groups': groups,
            'system': {k: v for k, v in system.items() if k not in ('interfaces', 'monitor')},
            'monitor': {**system.get('monitor', {}), 'uptime_s': round(now - self.started),
                        'subscriptions': sum(1 for s in self._topic_subs.values() if s is not None),
                        'checks': len(self.checks), 'config': self.config_path},
            'graph': {'nodes': len(self.graph.nodes) if self.graph else 0,
                      'topics': len(self.graph.topics) if self.graph else 0},
            'events': self.history.recent_events(20),
        }

    def _publish(self, overall, results, levels):
        """Also publish everything as a DiagnosticArray for rqt_robot_monitor / Foxglove."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(DiagnosticStatus(
            level=bytes([DIAG_LEVEL[overall]]), name='health_monitor: overall',
            message=OVERALL_LABEL[overall], hardware_id=self.namespace))
        for cid, (check, result) in results.items():
            values = [KeyValue(key='value', value=result.value)]
            if result.metric is not None:
                values.append(KeyValue(key='metric', value=f'{result.metric:.3f}'))
            msg.status.append(DiagnosticStatus(
                level=bytes([DIAG_LEVEL[levels[cid]]]),
                name=f'health_monitor: {check.group}/{check.name}',
                message=' · '.join(p for p in (result.value, result.detail) if p),
                hardware_id=check.topic or '', values=values))
        self.status_pub.publish(msg)

    # ---- pings and systemd services ----------------------------------------------------------

    def _start_probes(self):
        probes = [check for check in self.checks if isinstance(check, (PingCheck, SystemdCheck))]
        if probes:
            threading.Thread(target=self._probe_loop, args=(probes,), name='health-probes',
                             daemon=True).start()

    def _probe_loop(self, probes):
        due = dict.fromkeys((check.id for check in probes), 0.0)
        while not self._stop.is_set():
            for check in probes:
                if time.monotonic() >= due[check.id]:
                    if isinstance(check, PingCheck):
                        check.record(ping(check.host, check.timeout_s))
                    else:
                        check.record(*systemd_state(check.unit, check.user))
                    due[check.id] = time.monotonic() + check.period_s
            self._stop.wait(0.5)

    # ---- API used by the web server --------------------------------------------------------

    def snapshot(self):
        return self._snapshot

    def history_query(self, seconds):
        return self.history.query(seconds)

    def graph_listing(self):
        return self.graph_list

    # ---------------------------------------------------------------------------------------

    def _check_environment(self):
        if (os.environ.get('ROS_DISCOVERY_SERVER')
                and os.environ.get('ROS_SUPER_CLIENT', '').lower() not in ('true', '1')):
            self.get_logger().warning(
                'ROS_DISCOVERY_SERVER is set but ROS_SUPER_CLIENT is not True, so this node '
                'cannot see the whole ROS graph and node/topic checks will report missing. '
                'Run `export ROS_SUPER_CLIENT=True` before launching.')
        if os.environ.get(SIMULATE_ENV):
            self.get_logger().warning(
                'local test mode: systemd services that are not installed on this computer '
                'are shown as active (simulated)')

    def shutdown(self):
        self._stop.set()
        self.history.close()
        if self.web:
            self.web.stop()


def make_executor(kind):
    """The events executor costs far less CPU per message than the default one."""
    if kind == 'events':
        try:
            from rclpy.experimental import EventsExecutor
            return EventsExecutor()
        except ImportError:
            pass
    return SingleThreadedExecutor()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HealthMonitor()
        executor = make_executor(node.executor_kind)
        node.get_logger().info(f'using {type(executor).__name__}')
        executor.add_node(node)
        executor.spin()
    except (ConfigError, OSError, yaml.YAMLError) as exc:
        print(f'[health_monitor] cannot start: {exc}', file=sys.stderr)
        raise SystemExit(1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.shutdown()
                node.destroy_node()
            except KeyboardInterrupt:
                pass  # Ctrl-C reaches the node twice: from the terminal and from ros2 launch
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
