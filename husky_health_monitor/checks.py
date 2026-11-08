"""Health checks: what to watch and how to judge it.

This module has no rclpy dependency so the evaluation logic can be unit tested
without a ROS graph. The monitor node feeds each check messages, graph info and
system stats, then calls evaluate() once per update cycle.
"""

from __future__ import annotations

import fnmatch
import math
import re
from collections import deque
from dataclasses import dataclass, field

OK = 'ok'
IDLE = 'idle'
WAITING = 'waiting'
WARN = 'warn'
ERROR = 'error'

SEVERITY = {OK: 0, IDLE: 0, WAITING: 1, WARN: 2, ERROR: 3}
OVERALL_LABEL = {OK: 'HEALTHY', IDLE: 'HEALTHY', WAITING: 'STARTING', WARN: 'WARNING', ERROR: 'ERROR'}

# diagnostic_msgs/DiagnosticStatus levels
DIAG_OK, DIAG_WARN, DIAG_ERROR, DIAG_STALE = 0, 1, 2, 3
DIAG_NAMES = {DIAG_OK: 'OK', DIAG_WARN: 'WARN', DIAG_ERROR: 'ERROR', DIAG_STALE: 'STALE'}

# NavSatStatus.status as the A300's Fixposition receiver uses it (override with fix_labels)
FIX_LABELS = {-1: 'NO FIX', 0: 'GNSS FIX', 1: 'RTK FLOAT', 2: 'RTK FIXED'}


class ConfigError(ValueError):
    """The YAML config describes a check that cannot be built."""


@dataclass
class Result:
    level: str
    value: str
    detail: str = ''
    metric: float | None = None
    items: list = field(default_factory=list)


@dataclass
class GraphInfo:
    nodes: set = field(default_factory=set)
    topics: dict = field(default_factory=dict)       # topic -> [type, ...]
    publishers: dict = field(default_factory=dict)   # topic -> count
    subscribers: dict = field(default_factory=dict)  # topic -> count, not counting the monitor
    own_node: str = ''


@dataclass
class Context:
    now: float
    in_grace: bool = False
    graph: GraphInfo | None = None
    system: dict = field(default_factory=dict)


def worst(*levels):
    return max(levels, key=lambda level: SEVERITY[level])


def fmt_hz(hz):
    return f'{hz:.0f} Hz' if hz >= 20 else f'{hz:.1f} Hz'


def slug(text):
    return re.sub(r'[^a-z0-9]+', '_', text.lower()).strip('_')


def resolve_name(name, namespace):
    """Make a topic or node name absolute. Names starting with / or * are left alone."""
    if name.startswith(('/', '*')):
        return name
    ns = namespace.strip('/')
    return f'/{ns}/{name}' if ns else f'/{name}'


_INDEXED = re.compile(r'^(\w+)\[(-?\d+)\]$')


def get_field(msg, path):
    """Read a dotted field path such as 'status.status' or 'buttons[3]' from a message."""
    value = msg
    for part in path.split('.'):
        match = _INDEXED.match(part)
        key = match[1] if match else part
        value = value[key] if isinstance(value, dict) else getattr(value, key)
        if match:
            value = value[int(match[2])]
    return value


def _as_int(value):
    # ROS 2 'byte' fields (e.g. DiagnosticStatus.level) arrive as length-1 bytes
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


def _float(value):
    return None if value is None else float(value)


def _level(spec, key, default):
    level = spec.get(key, default)
    if level not in (WARN, ERROR):
        raise ConfigError(f'{key} must be "warn" or "error", got {level!r}')
    return level


def _join(*parts):
    return ' · '.join(p for p in parts if p)


class RateTracker:
    """Message receive times over a sliding window."""

    def __init__(self, window_s):
        self.window_s = window_s
        self._stamps = deque()
        self._sizes = deque()
        self.last_stamp = None
        self.total = 0

    def tick(self, now, nbytes=0):
        self._stamps.append(now)
        self._sizes.append(nbytes)
        self.last_stamp = now
        self.total += 1
        self._trim(now)

    def _trim(self, now):
        cutoff = now - self.window_s
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()
            self._sizes.popleft()

    def rate(self, now):
        self._trim(now)
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else 0.0

    def bandwidth(self, now):
        """Bytes per second over the window (only known for raw subscriptions)."""
        self._trim(now)
        return sum(self._sizes) / self.window_s

    def age(self, now):
        return None if self.last_stamp is None else now - self.last_stamp


class Check:
    kind = ''
    topic = None

    def __init__(self, spec, group, namespace):
        if not spec.get('name'):
            raise ConfigError(f'a check in group "{group}" has no name')
        self.name = spec['name']
        self.group = group
        self.id = f'{slug(group)}.{slug(self.name)}'
        self.critical = bool(spec.get('critical', True))
        self.description = spec.get('description', '')
        self.ok_label = spec.get('ok_label')
        self.missing_level = _level(spec, 'missing_level', ERROR)
        self.missing_label = spec.get('missing_label')
        self.show_detail = bool(spec.get('show_detail', False))

    def evaluate(self, ctx: Context) -> Result:
        raise NotImplementedError

    def missing(self, value, detail, metric=None):
        return Result(self.missing_level, self.missing_label or value, detail, metric)

    def describe(self):
        """Static facts about the check for the dashboard's detail view."""
        return {'kind': self.kind, 'critical': self.critical, 'description': self.description,
                'show_detail': self.show_detail}


class TopicCheck(Check):
    """Base for checks that subscribe to a topic. Handles 'is data arriving, and fast enough'."""

    needs_message = False
    # Rate-based metrics drop to 0 when data stops; value metrics (e.g. battery %) leave a gap
    metric_is_rate = True

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('topic'):
            raise ConfigError(f'{group} / {self.name}: "{self.kind}" checks need a topic')
        self.topic = resolve_name(spec['topic'], namespace)
        self.msg_type = spec.get('msg_type')
        hz = _float(spec.get('expected_hz'))
        self.expected_hz = hz
        self.warn_below_hz = _float(spec.get('warn_below_hz', 0.8 * hz if hz else None))
        self.error_below_hz = _float(spec.get('error_below_hz', 0.5 * hz if hz else None))
        self.timeout_s = _float(spec.get('timeout_s', max(2.0, 3.0 / hz) if hz else None))
        self.idle_ok = bool(spec.get('idle_ok', False))
        self.idle_label = spec.get('idle_label', 'IDLE')
        self.tracker = RateTracker(_float(spec.get('window_s', max(5.0, 3.0 / hz) if hz else 10.0)))
        self.last_msg = None
        self.latched = False
        self.type_error = None

    def on_message(self, now, msg=None, nbytes=0):
        self.tracker.tick(now, nbytes)
        if msg is not None:
            self.last_msg = msg

    def evaluate(self, ctx):
        stalled = self._check_flow(ctx)
        if stalled:
            return stalled
        hz = self.tracker.rate(ctx.now)
        try:
            result = self.judge(ctx, hz)
        except (AttributeError, KeyError, IndexError, TypeError, ValueError) as exc:
            return Result(WARN, 'BAD FIELD', f'could not read message: {exc}')
        rate_level, rate_note = self._rate_level(hz)
        result.level = worst(result.level, rate_level)
        result.detail = _join(rate_note, result.detail)
        return result

    def judge(self, ctx, hz) -> Result:
        raise NotImplementedError

    def _check_flow(self, ctx):
        age = self.tracker.age(ctx.now)
        if age is None:
            if self.type_error:
                return Result(WARN, 'NO TYPE', self.type_error)
            if ctx.in_grace:
                return Result(WAITING, 'WAITING', 'waiting for first message')
            if self.idle_ok:
                return Result(IDLE, self.idle_label, _join('no messages yet', self._publishers(ctx)),
                              self._no_data_metric())
            graph = ctx.graph
            if graph is not None and not graph.publishers.get(self.topic):
                return self.missing('NO PUBLISHER', f'nothing is publishing {self.topic}')
            return self.missing('NO DATA', 'publisher found but no messages received')
        if self.timeout_s and not self.latched and age > self.timeout_s:
            if self.idle_ok:
                return Result(IDLE, self.idle_label,
                              _join(f'last message {age:.1f} s ago', self._publishers(ctx)),
                              self._no_data_metric())
            return self.missing('STALE', f'last message {age:.1f} s ago', self._no_data_metric())
        return None

    def _publishers(self, ctx):
        if ctx.graph is None:
            return ''
        count = ctx.graph.publishers.get(self.topic, 0)
        return f'{count} publisher{"" if count == 1 else "s"}'

    def _no_data_metric(self):
        return 0.0 if self.metric_is_rate else None

    def _rate_level(self, hz):
        # A rate needs a few samples, and latched / on-change topics have none
        if self.latched or self.tracker.total < 3:
            return OK, ''
        if self.error_below_hz is not None and hz < self.error_below_hz:
            return ERROR, f'rate {fmt_hz(hz)} < {fmt_hz(self.error_below_hz)}'
        if self.warn_below_hz is not None and hz < self.warn_below_hz:
            return WARN, f'rate {fmt_hz(hz)} < {fmt_hz(self.warn_below_hz)}'
        return OK, ''

    def describe(self):
        return {
            **super().describe(),
            'topic': self.topic,
            'expected_hz': self.expected_hz,
            'warn_below_hz': self.warn_below_hz,
            'error_below_hz': self.error_below_hz,
            'timeout_s': self.timeout_s,
        }


class RateCheck(TopicCheck):
    """Data is arriving at the expected rate. Uses raw subscriptions, so no deserialization."""

    kind = 'rate'

    def judge(self, ctx, hz):
        if self.latched or self.tracker.total < 2:
            return Result(OK, self.ok_label or 'OK', 'latched' if self.latched else 'receiving')
        if self.ok_label:
            return Result(OK, self.ok_label, fmt_hz(hz), hz)
        return Result(OK, fmt_hz(hz), '', hz)


class BatteryCheck(TopicCheck):
    """sensor_msgs/BatteryState: charge percentage against warn/error thresholds."""

    kind = 'battery'
    needs_message = True
    metric_is_rate = False

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.warn_below = float(spec.get('warn_below_percent', 30))
        self.error_below = float(spec.get('error_below_percent', 15))
        self.scale = spec.get('percentage_scale', 'auto')
        if self.scale not in ('auto', 'fraction', 'percent'):
            raise ConfigError(f'{group} / {self.name}: percentage_scale must be auto, fraction or percent')

    def judge(self, ctx, hz):
        msg = self.last_msg
        if not hasattr(msg, 'percentage'):
            return Result(WARN, 'BAD TYPE', f'{type(msg).__name__} is not a sensor_msgs/BatteryState')
        voltage = float(getattr(msg, 'voltage', math.nan))
        current = float(getattr(msg, 'current', math.nan))
        notes = []
        if getattr(msg, 'power_supply_status', 0) == 1:  # POWER_SUPPLY_STATUS_CHARGING
            notes.append('charging')
        if math.isfinite(voltage):
            notes.append(f'{voltage:.1f} V')
        if math.isfinite(current):
            notes.append(f'{current:+.1f} A')

        pct = float(msg.percentage)
        if math.isnan(pct):
            value = f'{voltage:.1f} V' if math.isfinite(voltage) else 'NO %'
            return Result(WARN, value, _join('battery percentage not reported', *notes))
        # BatteryState defines percentage as 0..1, but some drivers publish 0..100
        if self.scale == 'fraction' or (self.scale == 'auto' and pct <= 1.0):
            pct *= 100.0
        if pct < self.error_below:
            level, note = ERROR, f'below {self.error_below:.0f}%'
        elif pct < self.warn_below:
            level, note = WARN, f'below {self.warn_below:.0f}%'
        else:
            level, note = OK, ''
        return Result(level, f'{pct:.0f}%', _join(note, *notes), pct)

    def describe(self):
        return {**super().describe(), 'warn_below_percent': self.warn_below,
                'error_below_percent': self.error_below}


class NavSatCheck(TopicCheck):
    """sensor_msgs/NavSatFix: rate plus fix quality."""

    kind = 'navsat'
    needs_message = True

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.no_fix_level = _level(spec, 'no_fix_level', WARN)
        self.min_status = int(spec.get('min_status', 0))
        self.fix_labels = {**FIX_LABELS, **{int(k): str(v) for k, v in
                                            (spec.get('fix_labels') or {}).items()}}

    def judge(self, ctx, hz):
        status = int(get_field(self.last_msg, 'status.status'))
        label = self.fix_labels.get(status, f'STATUS {status}')
        value = _join(fmt_hz(hz), label) if self.tracker.total >= 3 else label
        if status < 0:
            return Result(self.no_fix_level, value, 'receiver reports no fix', hz)
        if status < self.min_status:
            wanted = self.fix_labels.get(self.min_status, str(self.min_status))
            return Result(WARN, value, f'fix type {label}, want {wanted} or better', hz)
        return Result(OK, value, '', hz)


class BoolCheck(TopicCheck):
    """A boolean field (std_msgs/Bool by default) that should hold a given value."""

    kind = 'bool'
    needs_message = True
    metric_is_rate = False

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.field = spec.get('field', 'data')
        ok_value = spec.get('ok_value', True)
        self.ok_value = None if ok_value == 'any' else bool(ok_value)  # any: just report it
        self.bad_level = _level(spec, 'bad_level', WARN)
        self.bad_detail = spec.get('bad_detail', '')
        self.true_label = str(spec.get('true_label', 'TRUE'))
        self.false_label = str(spec.get('false_label', 'FALSE'))

    def judge(self, ctx, hz):
        value = bool(get_field(self.last_msg, self.field))
        label = self.true_label if value else self.false_label
        if self.ok_value is None or value == self.ok_value:
            return Result(OK, label, '', float(value))
        return Result(self.bad_level, label, self.bad_detail, float(value))


class OrientationCheck(TopicCheck):
    """Roll, pitch and yaw from a quaternion (nav_msgs/Odometry by default), with optional
    tilt limits. Its trend chart follows tilt: the larger of |roll| and |pitch|."""

    kind = 'orientation'
    needs_message = True
    metric_is_rate = False

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.field = spec.get('field', 'pose.pose.orientation')
        self.warn_tilt = _float(spec.get('warn_tilt_deg'))
        self.error_tilt = _float(spec.get('error_tilt_deg'))

    def judge(self, ctx, hz):
        q = get_field(self.last_msg, self.field)
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1e-6:
            return Result(WARN, 'NO ORIENTATION', 'quaternion is empty (sensor not initialised?)')
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        tilt = max(abs(roll), abs(pitch))
        value, detail = f'yaw {yaw:.0f}°', f'roll {roll:+.1f}° · pitch {pitch:+.1f}°'
        for level, limit in ((ERROR, self.error_tilt), (WARN, self.warn_tilt)):
            if limit is not None and tilt > limit:
                return Result(level, value, _join(f'tilted {tilt:.0f}° (limit {limit:g}°)', detail), tilt)
        return Result(OK, value, detail, tilt)

    def describe(self):
        return {**super().describe(), 'warn_tilt_deg': self.warn_tilt,
                'error_tilt_deg': self.error_tilt}


class FieldCheck(TopicCheck):
    """Any message field, judged by numeric thresholds or by allowed / disallowed values."""

    kind = 'field'
    needs_message = True
    metric_is_rate = False

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('field'):
            raise ConfigError(f'{group} / {self.name}: field checks need a field')
        self.field = spec['field']
        self.unit = spec.get('unit', '')
        self.scale = float(spec.get('scale', 1.0))
        self.decimals = int(spec.get('decimals', 1))
        self.limits = {k: _float(spec.get(k)) for k in
                       ('warn_above', 'error_above', 'warn_below', 'error_below')}
        self.ok_values = spec.get('ok_values')
        self.warn_values = spec.get('warn_values') or []
        self.error_values = spec.get('error_values') or []

    def judge(self, ctx, hz):
        raw = get_field(self.last_msg, self.field)
        if isinstance(raw, (bytes, bytearray)) and len(raw) == 1:
            raw = raw[0]
        if raw in self.error_values:
            return Result(ERROR, str(raw), f'{self.field} = {raw}')
        if raw in self.warn_values:
            return Result(WARN, str(raw), f'{self.field} = {raw}')
        if self.ok_values is not None and raw not in self.ok_values:
            return Result(WARN, str(raw), f'{self.field} = {raw}, expected one of {self.ok_values}')
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return Result(OK, str(raw))

        value = raw * self.scale
        text = f'{value:.{self.decimals}f}{self.unit}'
        lim = self.limits
        for level, key, test in ((ERROR, 'error_above', value > (lim['error_above'] or 0)),
                                 (ERROR, 'error_below', value < (lim['error_below'] or 0)),
                                 (WARN, 'warn_above', value > (lim['warn_above'] or 0)),
                                 (WARN, 'warn_below', value < (lim['warn_below'] or 0))):
            if lim[key] is not None and test:
                return Result(level, text, f'{key.replace("_", " ")} {lim[key]:g}{self.unit}', value)
        return Result(OK, text, '', value)


class DiagnosticsCheck(TopicCheck):
    """diagnostic_msgs: a single DiagnosticStatus (e.g. the aggregator's top-level state)
    or a DiagnosticArray (e.g. diagnostics_agg), reduced to its worst entry."""

    kind = 'diagnostics'
    needs_message = True
    metric_is_rate = False

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.include = spec.get('include') or ['*']
        self.exclude = spec.get('exclude') or []
        self.stale_level = _level(spec, 'stale_level', WARN)
        self.max_items = int(spec.get('max_items', 8))

    def _map(self, diag_level):
        return {DIAG_OK: OK, DIAG_WARN: WARN, DIAG_ERROR: ERROR}.get(diag_level, self.stale_level)

    def judge(self, ctx, hz):
        msg = self.last_msg
        if hasattr(msg, 'level'):
            level = _as_int(msg.level)
            return Result(self._map(level), DIAG_NAMES.get(level, str(level)), msg.message)

        entries = [(s.name, _as_int(s.level), s.message) for s in msg.status
                   if any(fnmatch.fnmatchcase(s.name, p) for p in self.include)
                   and not any(fnmatch.fnmatchcase(s.name, p) for p in self.exclude)]
        names = [name for name, _, _ in entries]
        # The aggregator publishes parents (/Platform) and children (/Platform/Battery); judge leaves
        leaves = [e for e in entries if not any(n.startswith(e[0] + '/') for n in names)]
        bad = [e for e in leaves if e[1] != DIAG_OK]
        if not bad:
            return Result(OK, 'OK', f'{len(leaves)} entries')

        levels = [self._map(lvl) for _, lvl, _ in bad]
        counts = {name: sum(1 for _, lvl, _ in bad if DIAG_NAMES.get(lvl) == name)
                  for name in ('ERROR', 'WARN', 'STALE')}
        value = ' · '.join(f'{n} {name}' for name, n in counts.items() if n)
        bad.sort(key=lambda e: -SEVERITY[self._map(e[1])])
        items = [{'name': name, 'level': self._map(lvl), 'status': DIAG_NAMES.get(lvl, str(lvl)),
                  'message': text} for name, lvl, text in bad[:self.max_items]]
        detail = '; '.join(f'{i["name"].rsplit("/", 1)[-1]}: {i["message"]}' for i in items[:3])
        return Result(worst(*levels), value, detail, items=items)


class NodeCheck(Check):
    """One or more nodes matching a name or glob pattern are in the ROS graph."""

    kind = 'node'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        patterns = spec.get('node')
        if not patterns:
            raise ConfigError(f'{group} / {self.name}: node checks need a node name or pattern')
        self.patterns = [resolve_name(p, namespace) for p in
                         (patterns if isinstance(patterns, list) else [patterns])]
        self.min_count = int(spec.get('min_count', 1))

    def evaluate(self, ctx):
        if ctx.graph is None:
            return Result(WAITING, 'WAITING', 'waiting for graph discovery')
        matched = sorted(n for n in ctx.graph.nodes
                         if any(fnmatch.fnmatchcase(n, p) for p in self.patterns))
        if len(matched) >= self.min_count:
            shown = ', '.join(matched[:3]) + (f' +{len(matched) - 3} more' if len(matched) > 3 else '')
            return Result(OK, self.ok_label or 'RUNNING', shown, float(len(matched)))
        if ctx.in_grace:
            return Result(WAITING, 'WAITING', 'waiting for graph discovery')
        return self.missing('NOT RUNNING', f'no node matching {", ".join(self.patterns)}', 0.0)

    def describe(self):
        return {**super().describe(), 'nodes': self.patterns}


class TopicGraphCheck(Check):
    """A topic has enough publishers / subscribers. Graph only, no subscription."""

    kind = 'topic'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('topic'):
            raise ConfigError(f'{group} / {self.name}: topic checks need a topic')
        self.graph_topic = resolve_name(spec['topic'], namespace)
        self.min_publishers = int(spec.get('min_publishers', 1))
        self.min_subscribers = int(spec.get('min_subscribers', 0))

    def evaluate(self, ctx):
        if ctx.graph is None:
            return Result(WAITING, 'WAITING', 'waiting for graph discovery')
        pubs = ctx.graph.publishers.get(self.graph_topic, 0)
        subs = ctx.graph.subscribers.get(self.graph_topic, 0)
        counts = f'{pubs} pub · {subs} sub'
        if pubs >= self.min_publishers and subs >= self.min_subscribers:
            return Result(OK, self.ok_label or 'RUNNING', counts)
        if ctx.in_grace:
            return Result(WAITING, 'WAITING', counts)
        if pubs < self.min_publishers:
            return self.missing('NO PUBLISHER', f'{counts}, want ≥ {self.min_publishers} publisher(s)')
        return self.missing('NO SUBSCRIBER', f'{counts}, want ≥ {self.min_subscribers} subscriber(s)')

    def describe(self):
        return {**super().describe(), 'topic': self.graph_topic,
                'min_publishers': self.min_publishers, 'min_subscribers': self.min_subscribers}


class RosGraphCheck(Check):
    """This monitor can see the rest of the ROS graph (catches DDS / discovery server problems)."""

    kind = 'ros_graph'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.min_nodes = int(spec.get('min_nodes', 1))

    def evaluate(self, ctx):
        if ctx.graph is None:
            return Result(WAITING, 'WAITING', 'waiting for graph discovery')
        others = len(ctx.graph.nodes - {ctx.graph.own_node})
        value = f'{others} nodes · {len(ctx.graph.topics)} topics'
        if others >= self.min_nodes:
            return Result(OK, self.ok_label or 'HEALTHY', value, float(others))
        if ctx.in_grace:
            return Result(WAITING, 'WAITING', value)
        return self.missing('NO NODES' if others == 0 else 'FEW NODES',
                            f'{value}; check ROS_DOMAIN_ID, ROS_DISCOVERY_SERVER and '
                            f'ROS_SUPER_CLIENT', float(others))


SYSTEM_METRICS = {
    'cpu': ('%', 85, 95),
    'memory': ('%', 85, 95),
    'disk': ('%', 85, 95),
    'temperature': (' °C', 80, 90),
}


class SystemCheck(Check):
    """CPU, memory, disk or temperature of the computer the monitor runs on."""

    kind = 'system'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        self.metric = spec.get('metric')
        if self.metric not in SYSTEM_METRICS:
            raise ConfigError(f'{group} / {self.name}: metric must be one of {", ".join(SYSTEM_METRICS)}')
        self.unit, warn, error = SYSTEM_METRICS[self.metric]
        self.warn_above = float(spec.get('warn_above', warn))
        self.error_above = float(spec.get('error_above', error))
        self.path = spec.get('path', '/')

    def evaluate(self, ctx):
        if self.metric == 'disk':
            value = ctx.system.get('disks', {}).get(self.path)
        else:
            value = ctx.system.get(self.metric)
        if value is None:
            return Result(IDLE, 'N/A', 'not available on this computer')
        text = f'{value:.0f}{self.unit}'
        if value > self.error_above:
            return Result(ERROR, text, f'above {self.error_above:g}{self.unit}', value)
        if value > self.warn_above:
            return Result(WARN, text, f'above {self.warn_above:g}{self.unit}', value)
        return Result(OK, text, '', value)

    def describe(self):
        return {**super().describe(), 'system_metric': self.metric, 'path': self.path,
                'warn_above': self.warn_above, 'error_above': self.error_above}


class InterfaceCheck(Check):
    """A network interface (e.g. tailscale0, wlan0) is up and has an IPv4 address."""

    kind = 'interface'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('interface'):
            raise ConfigError(f'{group} / {self.name}: interface checks need an interface name')
        self.interface = spec['interface']

    def evaluate(self, ctx):
        interfaces = ctx.system.get('interfaces')
        if interfaces is None:
            return Result(IDLE, 'N/A', 'interface info not available (install python3-psutil)')
        info = interfaces.get(self.interface)
        if info is None:
            return self.missing('MISSING', f'no interface named {self.interface}')
        if not info['up']:
            return self.missing('DOWN', f'{self.interface} is down')
        if not info['ipv4']:
            return Result(WARN, 'NO IP', f'{self.interface} is up but has no IPv4 address')
        return Result(OK, self.ok_label or info['ipv4'], self.interface)


class PingCheck(Check):
    """A host answers ping. The monitor node runs the pings on a background thread."""

    kind = 'ping'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('host'):
            raise ConfigError(f'{group} / {self.name}: ping checks need a host')
        self.host = str(spec['host'])
        self.period_s = float(spec.get('period_s', 5.0))
        self.timeout_s = float(spec.get('timeout_s', 2.0))
        self.warn_ms = float(spec.get('warn_ms', 250))
        self.rtt_ms = None
        self.attempts = 0

    def record(self, rtt_ms):
        self.rtt_ms = rtt_ms
        self.attempts += 1

    def evaluate(self, ctx):
        if self.attempts == 0:
            return Result(WAITING, 'WAITING', f'pinging {self.host}')
        if self.rtt_ms is None:
            return self.missing('UNREACHABLE', f'{self.host} did not answer within {self.timeout_s:g} s')
        value = f'{self.rtt_ms:.0f} ms'
        if self.rtt_ms > self.warn_ms:
            return Result(WARN, value, f'{self.host} slower than {self.warn_ms:g} ms', self.rtt_ms)
        return Result(OK, value, self.host, self.rtt_ms)

    def describe(self):
        return {**super().describe(), 'host': self.host, 'warn_ms': self.warn_ms}


class SystemdCheck(Check):
    """A systemd service is active, e.g. mors@camera_streams.service. The monitor node asks
    systemctl on a background thread; this check only judges the answer."""

    kind = 'systemd'

    def __init__(self, spec, group, namespace):
        super().__init__(spec, group, namespace)
        if not spec.get('unit'):
            raise ConfigError(f'{group} / {self.name}: systemd checks need a unit')
        unit = str(spec['unit'])
        self.unit = unit if '.' in unit else unit + '.service'
        self.user = bool(spec.get('user', False))
        self.period_s = float(spec.get('period_s', 5.0))
        # StatusText prefixes that mean healthy, for services (like tailscaled) that can be
        # running without working; the first part of the status becomes the shown value
        self.status_ok = spec.get('status_ok')
        self.props = None
        self.error = None
        self.attempts = 0

    def record(self, props, error=None):
        self.props, self.error = props, error
        self.attempts += 1

    def evaluate(self, ctx):
        if self.attempts == 0:
            return Result(WAITING, 'WAITING', f'asking systemd about {self.unit}')
        if self.props is None:
            return Result(WARN, 'UNKNOWN', self.error or 'no answer from systemctl')
        props = self.props
        active, sub = props.get('ActiveState', ''), props.get('SubState', '')
        status = props.get('StatusText', '')
        if props.get('LoadState') == 'not-found':
            return self.missing('NOT FOUND', f'no unit named {self.unit} on this computer')
        if active == 'active' and self.status_ok and status:
            # e.g. 'Connected; user@example.com; 100.64.0.10 fd7a::1' -> CONNECTED
            state, _, rest = status.partition(';')
            value = state.strip().rstrip('.').upper()[:24]
            if status.startswith(tuple(self.status_ok)):
                return Result(OK, self.ok_label or value, _join(_since(props), rest.strip()))
            return Result(WARN, value, f'{self.unit} is running but reports: {status}')
        if active == 'active':
            return Result(OK, self.ok_label or 'ACTIVE', _join(sub, _since(props), status))
        if active in ('activating', 'reloading', 'deactivating'):
            return Result(WARN, active.upper(), _join(sub, status))
        result = props.get('Result', '')
        return self.missing(active.upper() or 'UNKNOWN',
                            _join(f'{self.unit} is {active or "in an unknown state"}',
                                  f'result: {result}' if result not in ('', 'success') else '',
                                  status))

    def describe(self):
        return {**super().describe(), 'unit': self.unit, 'user': self.user,
                'status_ok': self.status_ok}


def _since(props):
    # ActiveEnterTimestamp looks like 'Mon 2026-11-02 16:43:05 EST'
    parts = props.get('ActiveEnterTimestamp', '').split()
    return f'since {parts[1]} {parts[2][:5]}' if len(parts) >= 3 else ''


CHECK_TYPES = {cls.kind: cls for cls in (
    RateCheck, BatteryCheck, NavSatCheck, BoolCheck, OrientationCheck, FieldCheck,
    DiagnosticsCheck, NodeCheck, TopicGraphCheck, RosGraphCheck, SystemCheck, InterfaceCheck,
    PingCheck, SystemdCheck,
)}


def build_groups(config, namespace):
    """Turn the 'groups' section of the config into [(group name, [Check, ...]), ...]."""
    groups = []
    seen = set()
    for group in config.get('groups') or []:
        group_name = group.get('name')
        if not group_name:
            raise ConfigError('every group needs a name')
        checks = []
        for spec in group.get('checks') or []:
            kind = spec.get('type', 'rate')
            cls = CHECK_TYPES.get(kind)
            if cls is None:
                raise ConfigError(f'{group_name} / {spec.get("name")}: unknown check type "{kind}" '
                                  f'(expected one of {", ".join(CHECK_TYPES)})')
            check = cls(spec, group_name, namespace)
            if check.id in seen:
                raise ConfigError(f'two checks are named "{check.name}" in group "{group_name}"')
            seen.add(check.id)
            checks.append(check)
        groups.append((group_name, checks))
    if not seen:
        raise ConfigError('the config defines no checks')
    return groups


def effective_level(check, result):
    """Errors in non-critical checks only pull the overall status down to a warning."""
    if result.level == ERROR and not check.critical:
        return WARN
    return result.level


def overall_level(levels):
    return worst(*levels) if levels else OK
