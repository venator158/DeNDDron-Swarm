"""Clock synchronization: a node's estimate of ship time from its own local clock (localclock.py).

No zenoh dependency, so it is unit-tested.  Every absolute time the protocol exchanges is in the
ship's timebase; a drone converts its local reading with ship_time() before comparing it with
one.  The sync mode is set by CLOCK_SYNC:

- none: trust the local clock as ship time (the default; with perfect clocks this is exact).
"""

import os
from typing import Optional

MODES = ("none",)


class NoSync:
    """No synchronization: the local clock is taken as ship time."""
    mode = "none"

    def ship_time(self, local: float) -> float:
        return local

    def error_bound(self) -> Optional[float]:
        """This node's own estimate of |ship_time error| (s); None if it has none."""
        return None


def make(mode: Optional[str] = None):
    mode = (mode or os.environ.get("CLOCK_SYNC") or "none").lower()
    if mode not in MODES:
        raise ValueError(f"CLOCK_SYNC={mode!r}: expected one of {MODES}")
    return NoSync()
