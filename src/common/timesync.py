"""Clock synchronization: how a node relates its local clock (localclock.py) to ship time.

No zenoh dependency, so it is unit-tested.  Absolute times on the radio are in the ship's
timebase.  A drone keeps its protocol times in a *protocol timebase* and compares them with
proto_time(local):

- inbound(t, sent, local_rx): an absolute ship time from a message (stamped `sent` by the ship
  when the mode needs it), converted to the protocol timebase on receipt;
- to_ship(p): a protocol time back as an estimate of ship time, for stamps the drone sends
  (detonation sync_time, heartbeat time) and for comparing them with other drones' stamps.

Modes (CLOCK_SYNC):

- none:   trust the local clock as ship time (default; with perfect clocks this is exact).
- ttg:    time-to-go.  The ship stamps each message with its send time; the drone anchors every
          absolute time at its own receipt, local_rx + (t - sent).  The protocol timebase is the
          local clock, and every job update re-anchors.  The unknown one-way delay is error.
- master: two-way exchange with the ship (NTP-style, 4 timestamps: t1 drone send, t2 ship
          receive, t3 ship send, t4 drone receive), filtered by a Kalman filter on offset and
          rate (OffsetRateFilter).  The protocol timebase is the estimate of ship time.
"""

import math
import os
from typing import Optional, Tuple

MODES = ("none", "ttg", "master")


class NoSync:
    """No synchronization: the local clock is taken as ship time."""
    mode = "none"

    def proto_time(self, local: float) -> float:
        return local

    def inbound(self, t_ship: float, sent: Optional[float], local_rx: Optional[float]) -> float:
        return t_ship

    def on_sent(self, sent: Optional[float], local_rx: Optional[float]) -> None:
        """A message stamped with the ship's send time arrived (used by ttg)."""

    def to_ship(self, proto: float) -> float:
        return proto

    def offset(self, local: float) -> Optional[float]:
        """Estimated ship time - local time (s); None if the mode has no estimate."""
        return None

    def error_bound(self, local: float) -> Optional[float]:
        """This node's own bound on |ship time error| (s); None if it has none."""
        return None


class TimeToGo(NoSync):
    """Absolute times are re-anchored on receipt: t -> local_rx + (t - sent)."""
    mode = "ttg"

    def __init__(self):
        self._anchor = None          # sent - local_rx of the latest message: ship - local, delay included

    def on_sent(self, sent, local_rx) -> None:
        """Any stamped message re-anchors, also one that carries no times (e.g. an empty zone list)."""
        if sent is not None and local_rx is not None:
            self._anchor = float(sent) - local_rx

    def inbound(self, t_ship, sent, local_rx):
        if sent is None or local_rx is None:
            return t_ship
        self.on_sent(sent, local_rx)
        return local_rx + (float(t_ship) - float(sent))

    def to_ship(self, proto):
        return proto if self._anchor is None else proto + self._anchor

    def offset(self, local):
        return self._anchor


def exchange(t1: float, t2: float, t3: float, t4: float) -> Tuple[float, float]:
    """(offset, round-trip delay) of one two-way exchange: offset = ship - local at the exchange,
    assuming equal delays both ways (an asymmetry a biases it by a/2, and |bias| <= delay/2)."""
    return ((t2 - t1) + (t3 - t4)) / 2.0, (t4 - t1) - (t3 - t2)


class OffsetRateFilter:
    """Kalman filter on x = (offset, rate) of ship time relative to local time.

    offset(t) = x0 + x1 * (t - t_ref); x1 is the relative rate error (s/s).  An exchange's offset is
    wrong by half the asymmetry of its delays, at most half its round trip, so each sample is weighed
    by R = floor^2 + (delay / 2)^2: long round trips (queueing, a congested start) count for little.
    Judging a sample only against the recent minimum round trip is not enough: at startup every
    round trip was long, and lopsided ones with the same total were trusted fully and corrupted the
    rate for a minute.  Once settled (`settle` samples), a sample more than `gate` standard
    deviations from the prediction is rejected as an outlier.  An asymmetry common to all samples
    cannot be observed; the self-reported error bound includes half the best round trip for it.
    """

    def __init__(self, floor_s: float = 1e-3, q_offset: float = 1e-8, q_rate: float = 1e-12,
                 p0_rate: float = 1e-6, window: int = 32, gate: float = 5.0, settle: int = 8):
        self.floor_s, self.q_offset, self.q_rate, self.p0_rate = floor_s, q_offset, q_rate, p0_rate
        self.window, self.gate, self.settle = window, gate, settle
        self.rejected = 0
        self.x = None                # [offset at t_ref, rate]
        self.P = None                # 2x2 covariance
        self.t_ref = None
        self.delays = []             # recent round trips
        self.samples = 0

    def _predict(self, t: float):
        dt = t - self.t_ref
        if dt <= 0.0:
            return
        x0, x1 = self.x
        (p00, p01), (p10, p11) = self.P
        self.x = [x0 + x1 * dt, x1]
        # F = [[1, dt], [0, 1]];  Q = diag(q_offset, q_rate) * dt
        self.P = [[p00 + dt * (p01 + p10) + dt * dt * p11 + self.q_offset * dt, p01 + dt * p11],
                  [p10 + dt * p11, p11 + self.q_rate * dt]]
        self.t_ref = t

    def update(self, t_local: float, offset: float, delay: float) -> None:
        delay = max(0.0, delay)
        self.delays = (self.delays + [delay])[-self.window:]
        self.samples += 1
        if self.x is None:
            self.x, self.t_ref = [offset, 0.0], t_local
            r0 = self.floor_s ** 2 + (delay / 2.0) ** 2
            self.P = [[r0, 0.0], [0.0, self.p0_rate]]
            return
        self._predict(t_local)
        r = self.floor_s ** 2 + (delay / 2.0) ** 2
        (p00, p01), (p10, p11) = self.P
        s = p00 + r
        innov = offset - self.x[0]
        if self.samples > self.settle and innov * innov > self.gate * self.gate * s:
            self.rejected += 1
            return
        k0, k1 = p00 / s, p10 / s
        self.x = [self.x[0] + k0 * innov, self.x[1] + k1 * innov]
        self.P = [[(1 - k0) * p00, (1 - k0) * p01], [p10 - k1 * p00, p11 - k1 * p01]]

    def offset(self, t_local: float) -> Optional[float]:
        if self.x is None:
            return None
        return self.x[0] + self.x[1] * (t_local - self.t_ref)

    def rate(self) -> Optional[float]:
        return None if self.x is None else self.x[1]

    def error_bound(self, t_local: float) -> Optional[float]:
        """3 sigma of the offset estimate, extrapolated to t_local, plus half the best round trip."""
        if self.x is None:
            return None
        dt = max(0.0, t_local - self.t_ref)
        (p00, p01), (p10, p11) = self.P
        var = p00 + dt * (p01 + p10) + dt * dt * p11 + self.q_offset * dt
        return 3.0 * math.sqrt(max(var, 0.0)) + min(self.delays) / 2.0


class ShipMaster(NoSync):
    """Two-way exchanges with the ship (t1 in the heartbeat, t2/t3 echoed in the roster)."""
    mode = "master"

    def __init__(self, **filter_kw):
        self.filter = OffsetRateFilter(**filter_kw)
        self._last_t1 = None

    def on_exchange(self, t1: float, t2: float, t3: float, t4: float) -> bool:
        """Feed one exchange; a repeat of the last one (no newer heartbeat reached the ship) is ignored."""
        if t1 == self._last_t1:
            return False
        self._last_t1 = t1
        offset, delay = exchange(t1, t2, t3, t4)
        self.filter.update(t4, offset, delay)
        return True

    def proto_time(self, local):
        off = self.filter.offset(local)
        return local if off is None else local + off

    def offset(self, local):
        return self.filter.offset(local)

    def error_bound(self, local):
        return self.filter.error_bound(local)


def make(mode: Optional[str] = None):
    mode = (mode or os.environ.get("CLOCK_SYNC") or "none").lower()
    if mode not in MODES:
        raise ValueError(f"CLOCK_SYNC={mode!r}: expected one of {MODES}")
    return {"none": NoSync, "ttg": TimeToGo, "master": ShipMaster}[mode]()
