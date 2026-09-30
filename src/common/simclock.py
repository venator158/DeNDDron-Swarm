"""Protocol clock that follows simulated time, at whatever speed the simulator actually runs.

SIM_RTF is the *target* real-time factor (sim seconds per wall second).  Protocol timers
(heartbeats, link and roster timeouts, re-announce delays, award latency) use now() and sleep()
from here, so they keep their length in simulated time.  Each process feeds the clock the sim
times it observes (observe()): drones from their sensor frames, the ship from sim/clock.
Between observations the clock extrapolates at the measured sim rate, so it stays right when
the simulator falls behind its target (it did at 25+ drones), and it keeps running (at the
last measured rate) if observations stop, so no timer ever blocks forever.  Before the first
observation it runs at the target rate.  Readings are simulated time plus a constant offset
(chosen so the clock never jumps), so use differences between readings, not absolute values.

Compute telemetry (loop timing, CPU) stays on the real clock.  Radio impairments must be scaled
to match (delay / RTF, rate x RTF); the degradation sweep does that with the target factor.
"""

import os
import threading
import time

RTF = max(0.1, float(os.environ.get("SIM_RTF") or 1.0))   # target

_RATE_WINDOW_S = 1.0     # wall seconds between rate measurements
_lock = threading.Lock()
_sim = None              # last observed sim time
_wall = None             # monotonic time of that observation
_rate = RTF              # measured sim seconds per wall second
_rate_ref = None         # (wall, sim) at the start of the current rate window
_out = 0.0               # last value returned
_shift = 0.0             # constant offset keeping the clock continuous across (re)synchronisation


def _estimate(wall):
    return wall * RTF if _sim is None else _sim + (wall - _wall) * _rate + _shift


def observe(sim_time) -> None:
    """Report a sim time seen on the onboard link (cheap; call on every frame)."""
    global _sim, _wall, _rate, _rate_ref, _shift
    if sim_time is None:
        return
    sim_time, wall = float(sim_time), time.monotonic()
    with _lock:
        if _sim is None or sim_time + 1.0 < _sim:
            # First observation or simulator restart: continue from the current reading.
            _shift = _estimate(wall) - sim_time
            _rate_ref = None
        _sim, _wall = sim_time, wall
        if _rate_ref is None:
            _rate_ref = (wall, sim_time)
        elif wall - _rate_ref[0] >= _RATE_WINDOW_S:
            measured = (sim_time - _rate_ref[1]) / (wall - _rate_ref[0])
            if measured > 0:
                _rate = 0.5 * _rate + 0.5 * measured
            _rate_ref = (wall, sim_time)


def now() -> float:
    """Simulated time in seconds, plus a constant offset; never goes backwards."""
    global _out
    with _lock:
        _out = max(_out, _estimate(time.monotonic()))
        return _out


def truth_now():
    """Current sim time, extrapolated from the last observation (no shift), or None before one.
    For clock-sync timestamps, which need finer resolution than sensor frames (localclock.py)."""
    with _lock:
        return None if _sim is None else _sim + (time.monotonic() - _wall) * _rate


def rate() -> float:
    """Measured sim seconds per wall second (the target until measured)."""
    with _lock:
        return _rate


def sleep(seconds: float) -> None:
    """Sleep for `seconds` of simulated time."""
    target = now() + seconds
    while True:
        remaining = target - now()
        if remaining <= 0:
            return
        time.sleep(min(remaining / max(rate(), 0.1), 0.05))
