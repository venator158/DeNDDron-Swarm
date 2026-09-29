"""Per-agent instrumentation, summarized into each heartbeat.

Everything is windowed: ``snapshot()`` reports the interval since the last
snapshot and starts a new window, so the numbers are live rates and latencies
rather than lifetime averages.  Radio rates are per simulated second (comparable at any
SIM_RTF); loop timing and CPU are real time.
"""

import threading
import time
from collections import defaultdict

import simclock


def _pct(values, q):
    if not values:
        return None
    s = sorted(values)
    return s[min(len(s) - 1, int(q * len(s)))]


class Telemetry:
    def __init__(self, control_period_s: float):
        self.control_period_s = control_period_s
        self._lock = threading.Lock()
        self._reset(time.monotonic(), time.process_time())

    def _reset(self, wall, cpu):
        self._wall0, self._cpu0 = wall, cpu
        self._loop_periods = []
        self._loop_work = []
        self._overruns = 0
        self._sensor_ages = []
        self._perception = []
        self._planner = []
        self._rx = defaultdict(int)
        self._tx = defaultdict(int)
        self._tx_bytes = 0

    # -- recording (cheap, called from hot paths) ------------------------
    def loop_tick(self, period_s: float, work_s: float):
        with self._lock:
            self._loop_periods.append(period_s)
            self._loop_work.append(work_s)
            if period_s > 1.5 * self.control_period_s:
                self._overruns += 1

    def sensor_age(self, age_s: float):
        with self._lock:
            self._sensor_ages.append(age_s)

    def perception(self, seconds: float):
        with self._lock:
            self._perception.append(seconds)

    def planner(self, seconds: float):
        with self._lock:
            self._planner.append(seconds)

    def rx(self, topic: str):
        with self._lock:
            self._rx[topic] += 1

    def tx(self, topic: str, nbytes: int):
        with self._lock:
            self._tx[topic] += 1
            self._tx_bytes += nbytes

    # -- reporting -------------------------------------------------------
    def snapshot(self) -> dict:
        wall, cpu = time.monotonic(), time.process_time()
        with self._lock:
            span = max(1e-6, wall - self._wall0)
            sim_span = span * simclock.RTF
            ms = lambda v: None if v is None else round(v * 1000.0, 2)
            out = {
                "window_s": round(span, 2),
                "cpu_pct": round(100.0 * (cpu - self._cpu0) / span, 1),
                "loop_hz": round(len(self._loop_periods) / span, 1),
                "loop_p50_ms": ms(_pct(self._loop_periods, 0.5)),
                "loop_p99_ms": ms(_pct(self._loop_periods, 0.99)),
                "loop_work_p99_ms": ms(_pct(self._loop_work, 0.99)),
                "overruns": self._overruns,
                "sensor_age_p50_ms": ms(_pct(self._sensor_ages, 0.5)),
                "sensor_age_max_ms": ms(max(self._sensor_ages) if self._sensor_ages else None),
                "perception_p99_ms": ms(_pct(self._perception, 0.99)),
                "planner_p99_ms": ms(_pct(self._planner, 0.99)),
                "rx_per_s": {k: round(v / sim_span, 1) for k, v in self._rx.items()},
                "tx_per_s": {k: round(v / sim_span, 1) for k, v in self._tx.items()},
                "tx_bytes_per_s": round(self._tx_bytes / sim_span),
            }
            self._reset(wall, cpu)
        return out
