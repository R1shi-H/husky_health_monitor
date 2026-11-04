"""CPU, memory, disk, temperature and network interface stats for the onboard computer."""

import os
import socket
import time

try:
    import psutil
except ImportError:  # the monitor still runs, it just can't report these
    psutil = None

# Sensor chips that report the CPU package temperature, in order of preference
_CPU_SENSORS = ('coretemp', 'k10temp', 'zenpower', 'cpu_thermal', 'soc_thermal', 'acpitz')


class SystemSampler:

    def __init__(self, disk_paths=('/',)):
        self.disk_paths = sorted({'/', *disk_paths})
        self.hostname = socket.gethostname()
        self._proc = None
        if psutil:
            self._proc = psutil.Process()
            # cpu_percent(None) reports usage since the previous call; prime both counters
            psutil.cpu_percent(None)
            self._proc.cpu_percent(None)

    def sample(self):
        if psutil is None:
            return {'available': False, 'hostname': self.hostname}
        disks = {}
        for path in self.disk_paths:
            try:
                disks[path] = psutil.disk_usage(os.path.expanduser(path)).percent
            except OSError:
                pass
        mem = self._proc.memory_info()
        return {
            'available': True,
            'hostname': self.hostname,
            'cpu': psutil.cpu_percent(None),
            'cores': psutil.cpu_count(),
            'load': os.getloadavg()[0],
            'memory': psutil.virtual_memory().percent,
            'disks': disks,
            'temperature': self._temperature(),
            'uptime_s': time.time() - psutil.boot_time(),
            'interfaces': self._interfaces(),
            'monitor': {'cpu': self._proc.cpu_percent(None), 'rss_mb': mem.rss / 1e6},
        }

    @staticmethod
    def _temperature():
        try:
            temps = psutil.sensors_temperatures()
        except (AttributeError, OSError):
            return None
        for chip in _CPU_SENSORS:
            if temps.get(chip):
                return max(t.current for t in temps[chip])
        readings = [t.current for chip in temps.values() for t in chip]
        return max(readings) if readings else None

    @staticmethod
    def _interfaces():
        addrs = psutil.net_if_addrs()
        interfaces = {}
        for name, stats in psutil.net_if_stats().items():
            ipv4 = next((a.address for a in addrs.get(name, []) if a.family == socket.AF_INET), '')
            interfaces[name] = {'up': stats.isup, 'ipv4': ipv4}
        return interfaces
