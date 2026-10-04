"""GNSS as a comparator for UWB, and a fallback when UWB fails (README next steps, item 5).
No zenoh dependency, so it is unit-tested.

Owner's decisions (2026-10-04): GNSS is on by default and never fused while UWB is available; it is
a comparator that can flag a spoof; once a spoof is detected the drone reports it to the ship and
ignores GNSS; no RTK; when UWB fails, GNSS may stand in, with inflated noise, the IMU check and the
comparator still active.

- Ship-relative fixes: the ship broadcasts its own fix and heading (ship/gnss).  Our fix relative to
  the ship is R(-heading) (ours - ship's): the receivers' common-mode error (atmosphere, orbits)
  cancels, leaving both receivers' own errors (rel_sigma) and the heading error times range.
- UWB available = fixes from min_anchors (3) distinct anchors of our own within anchor_fresh_s (1 s;
  anchors range at 2 Hz; 2D, so three ranges leave one to check).  Peer
  chains do not count (coop turns to peers after 3 s without anchors, by then the IMU-only reference
  below has stopped following the filter): beyond
  anchor range they were tens of metres off while claiming sub-metre accuracy (uwb_short), the case
  GNSS is meant to catch.
- The IMU check: a shadow copy of the filter follows the main one while anchored, then keeps
  predicting on the IMU alone (no peers, no GNSS).  It is the reference GNSS is checked against
  when UWB is not available: a spoofer can move a fix, not the measured acceleration.
- Comparator (anchored): the mean normalized innovation (NIS, 2 dof) of the last `window` fixes
  against the UWB estimate; beyond its 99.9 % bound, GNSS disagrees with good UWB: spoofed (or
  faulty).  Latched: reported, and GNSS ignored from then on.
- Fallback (not anchored): each fix is checked against the shadow; `spoof_count` fixes in a row
  beyond the 99.9 % gate is a spoof (latched).  Otherwise it is fused into the filter every
  fuse_every_s with its noise inflated by `inflate`.  Its error is a slowly varying bias that
  repeated updates cannot average away, so the position variance is floored at that bias's
  variance afterwards (coop's lesson for peers).
"""

import math
from collections import deque
from typing import Optional, Tuple

import numpy as np

CHI2_2_999 = 13.82                     # chi-square, 2 dof, 99.9 %


class Gnss:
    def __init__(self, loc, errors: dict, window: int = 10, anchor_fresh_s: float = 1.0,
                 fuse_every_s: float = 1.0, inflate: float = 2.0, spoof_count: int = 3,
                 ship_stale_s: float = 3.0, fallback: bool = True, min_anchors: int = 3):
        self.loc = loc
        self.err = errors
        self.window = deque(maxlen=window)
        # The relative fix's error is mostly a slowly varying bias, so `window` fixes are nearly one
        # sample: their mean NIS faces the single-sample bound (an independent-samples bound,
        # chi-square(2n)/n, flagged clean flights).  The window averages out the white noise.
        self.mean_bound = CHI2_2_999
        self.anchor_fresh_s, self.fuse_every_s, self.inflate = anchor_fresh_s, fuse_every_s, inflate
        self.spoof_count, self.ship_stale_s, self.fallback = spoof_count, ship_stale_s, fallback
        self.min_anchors = min_anchors
        self.ship: Optional[Tuple[float, float, float, float]] = None   # (t, E, N, heading rad)
        self.shadow = None                   # IMU-only reference (a Localizer), synced while anchored
        self.state = "no_fix"                # no_fix | ok | fallback | spoofed
        self.spoof_t: Optional[float] = None
        self.last_nis: Optional[float] = None
        self._run = 0
        self._last_fuse = -1e9
        self.fixes = self.fused = 0

    # --- the ship's fix ----------------------------------------------------------
    def on_ship(self, t: float, enu: Tuple[float, float], heading_rad: float, fix: bool) -> None:
        self.ship = (t, float(enu[0]), float(enu[1]), float(heading_rad)) if fix else None

    def relative(self, enu: Tuple[float, float]) -> Tuple[float, float]:
        """Our fix in the ship frame."""
        _, se, sn, h = self.ship
        de, dn = float(enu[0]) - se, float(enu[1]) - sn
        c, s = math.cos(h), math.sin(h)
        return c * de + s * dn, -s * de + c * dn

    # --- the IMU-only reference ---------------------------------------------------
    def anchored(self, t: float) -> bool:
        """UWB is available: min_anchors distinct anchors gave a fix within anchor_fresh_s.  One or two
        (a drone at the edge of anchor range) leave the estimate weak along the tangent and often
        overconfident: live, GNSS disagreeing with it was taken for a spoof (5 drones in uwb_short)."""
        n = sum(1 for ts in self.loc.anchor_seen.values() if t - ts <= self.anchor_fresh_s)
        return n >= self.min_anchors

    def sync_shadow(self) -> None:
        """While anchored, the reference is the filter itself."""
        if self.loc.x is None:
            return
        if self.shadow is None:
            from localization import Localizer
            self.shadow = Localizer(range_sigma=self.loc.range_sigma, imu=self.loc.imu)
        self.shadow.x, self.shadow.P = self.loc.x.copy(), self.loc.P.copy()

    def shadow_predict_imu(self, dt: float, dv, dt_imu) -> None:
        if self.shadow is not None and self.shadow.x is not None:
            self.shadow.predict_imu(dt, dv, dt_imu)

    def shadow_predict_cmd(self, dt: float, cmd) -> None:
        if self.shadow is not None and self.shadow.x is not None:
            self.shadow.predict(dt, cmd)

    # --- a fix --------------------------------------------------------------------
    def on_fix(self, t: float, enu: Tuple[float, float], fix: bool = True, sats: int = 99) -> str:
        """Check (and, in fallback, maybe fuse) one fix.  Returns the state."""
        if self.state == "spoofed":
            return self.state
        if (not fix or sats < self.err["min_sats"] or self.ship is None or t - self.ship[0] > self.ship_stale_s
                or self.loc.x is None):
            self.state = "no_fix"
            return self.state
        self.fixes += 1
        z = np.array(self.relative(enu))
        rng = float(np.hypot(*z))
        r2 = self.err["rel_sigma"] ** 2 + (self.err["heading_sigma"] * rng) ** 2
        R = np.eye(2) * r2
        anchored = self.anchored(t)
        if anchored or self.shadow is None:
            self.sync_shadow()
        ref = self.loc if anchored else self.shadow
        d = z - ref.x[:2]
        nis = float(d @ np.linalg.solve(ref.P[:2, :2] + R, d))
        self.last_nis = nis
        if anchored:
            # Comparator: GNSS against good UWB.
            self._run = 0
            self.window.append(nis)
            if len(self.window) == self.window.maxlen and sum(self.window) / len(self.window) > self.mean_bound:
                return self._spoofed(t)
            self.state = "ok"
            return self.state
        # No UWB: check against the IMU-only reference, then stand in for UWB.
        self.window.clear()
        self._run = self._run + 1 if nis > CHI2_2_999 else 0
        if self._run >= self.spoof_count:
            return self._spoofed(t)
        self.state = "fallback"
        if self.fallback and nis <= CHI2_2_999 and t - self._last_fuse >= self.fuse_every_s:
            self._last_fuse = t
            self.loc.update_position(z, R * self.inflate ** 2)
            self._floor(self.err["rel_bias_sigma"] ** 2 + (self.err["heading_sigma"] * rng) ** 2)
            self.fused += 1
        return self.state

    def _floor(self, var: float) -> None:
        """Keep the position variance at least the fix's slowly varying bias in every direction."""
        P = self.loc.P[:2, :2]
        w, V = np.linalg.eigh(P)
        if w.min() < var:
            self.loc.P[:2, :2] = V @ np.diag(np.maximum(w, var)) @ V.T

    def _spoofed(self, t: float) -> str:
        self.state = "spoofed"
        self.spoof_t = t
        return self.state
