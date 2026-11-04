"""Recent health history in memory (for the dashboard) and on disk (JSON lines)."""

import datetime
import json
import os
import threading
from collections import deque
from pathlib import Path

from .checks import SEVERITY


def _round(value):
    return None if value is None else round(value, 2)


class History:

    def __init__(self, check_ids, period_s, memory_minutes=15, log_dir=None,
                 log_period_s=10.0, keep_days=7, warn=print):
        self.ids = list(check_ids)
        self.samples = deque(maxlen=int(memory_minutes * 60 / period_s) + 1)
        self.events = deque(maxlen=200)
        self._lock = threading.Lock()
        self._warn = warn
        self.log_dir = Path(os.path.expanduser(log_dir)) if log_dir else None
        self.log_period_s = log_period_s
        self.keep_days = keep_days
        self._next_log = 0.0
        self._day = None
        self._file = None

    def record(self, t, overall, results, system):
        """results: {check id: (level, value, metric)}"""
        sample = (
            t,
            SEVERITY[overall],
            tuple(_round(results[i][2]) for i in self.ids),
            bytes(SEVERITY[results[i][0]] for i in self.ids),
            (_round(system.get('cpu')), _round(system.get('memory')), _round(system.get('temperature'))),
        )
        with self._lock:
            self.samples.append(sample)
        if self.log_period_s and t >= self._next_log:
            self._next_log = t + self.log_period_s
            self._write({'t': round(t, 3), 'kind': 'sample', 'overall': overall,
                         'checks': {i: [results[i][0], results[i][1]] for i in self.ids},
                         'cpu': sample[4][0], 'memory': sample[4][1], 'temperature': sample[4][2]})

    def event(self, t, check, old, new, result):
        event = {'t': round(t, 3), 'id': check.id, 'group': check.group, 'name': check.name,
                 'from': old, 'to': new, 'value': result.value, 'detail': result.detail}
        with self._lock:
            self.events.append(event)
        self._write({'kind': 'event', **event})

    def recent_events(self, limit=50):
        with self._lock:
            return list(self.events)[-limit:]

    def query(self, seconds):
        with self._lock:
            samples = list(self.samples)
        if samples:
            cutoff = samples[-1][0] - seconds
            samples = [s for s in samples if s[0] >= cutoff]
        return {
            't': [round(s[0], 2) for s in samples],
            'overall': [s[1] for s in samples],
            'checks': {cid: {'metric': [s[2][k] for s in samples], 'level': [s[3][k] for s in samples]}
                       for k, cid in enumerate(self.ids)},
            'system': {name: [s[4][k] for s in samples]
                       for k, name in enumerate(('cpu', 'memory', 'temperature'))},
            'events': self.recent_events(),
        }

    def _write(self, record):
        if self.log_dir is None:
            return
        day = datetime.date.today()
        try:
            if day != self._day:
                self._open(day)
            self._file.write(json.dumps(record, separators=(',', ':')) + '\n')
            self._file.flush()
        except OSError as exc:
            self._warn(f'history logging disabled: {exc}')
            self.log_dir = None

    def _open(self, day):
        if self._file:
            self._file.close()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._file = open(self.log_dir / f'health-{day.isoformat()}.jsonl', 'a', encoding='utf-8')
        self._day = day
        oldest = day - datetime.timedelta(days=self.keep_days)
        for path in self.log_dir.glob('health-*.jsonl'):
            try:
                if datetime.date.fromisoformat(path.stem[len('health-'):]) < oldest:
                    path.unlink()
            except ValueError:
                pass

    def close(self):
        if self._file:
            self._file.close()
            self._file = None
