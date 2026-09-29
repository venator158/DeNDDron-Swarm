"""Protocol clock that keeps pace with a simulator running faster than real time.

SIM_RTF (real-time factor, default 1) is how many simulated seconds pass per wall second.
Protocol timers (heartbeats, link and roster timeouts, re-announce delays, award latency) use
now() and sleep() from here, so they keep their length in simulated time at any RTF and the
swarm behaves as it would in real time.  Compute telemetry (loop timing, CPU) stays on the real
clock.  Radio impairments must be scaled to match (delay / RTF, rate x RTF); the degradation
sweep does that.
"""

import os
import time

RTF = max(0.1, float(os.environ.get("SIM_RTF") or 1.0))


def now() -> float:
    """Monotonic time in simulated seconds."""
    return time.monotonic() * RTF


def sleep(seconds: float) -> None:
    """Sleep for `seconds` of simulated time."""
    time.sleep(seconds / RTF)
