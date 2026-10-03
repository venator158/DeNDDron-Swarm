"""Drone self-localization in the ground plane from UWB ranges (LOCALIZATION=anchors|coop).

No zenoh dependency, so it is unit-tested.  Altitude (z) and attitude are known (barometer, IMU,
compass: hardware record, taken as perfect); x, y come from here.

State: [x, y, vx, vy, wx, wy] with covariance P (an EKF).  v is the airframe's own velocity, w
the drift the wind adds to it, which the drone cannot sense directly.

- Prediction, every control tick: the drone knows the velocity it commands, and the airframe
  follows it through a first-order response, v <- v + alpha (cmd - v) per tick (the simulator's
  command filter; on hardware, the identified response of the flight controller), then
  p <- p + (v + w) dt.  Process noise: accel_sigma on v (what the response model misses) and a
  random walk wind_walk on w.  Without w, a steady 0.5 m/s wind made the filter 600x overconfident
  (NEES ~1200) and re-lock 290 times in one run; the anchors make w observable, and dead reckoning
  during an anchor outage then carries the wind along.
- Anchor ranges (ship-mounted UWB anchors, known positions): r = |p - a| in 3D with our z known.
  Not converted to horizontal ranges: near-vertical geometry would amplify that conversion; the 3D
  Jacobian [(x-ax)/r, (y-ay)/r] simply carries little horizontal information there.
- Gating: an innovation beyond `gate` sigmas (chi-square, 1 dof) is rejected (blocked paths,
  outliers).  `relock` rejections in a row mean the estimate, not the measurements, is wrong: it is
  re-initialized from the anchors (the clock filter's lesson: a gated filter can lock out).
- Initialization: a prior (the launch position, sigma init_sigma), or a least-squares fix from
  three or more anchors: subtracting one squared range equation from the others gives linear
  equations in x, y, refined by Gauss-Newton.

Peer ranges (LOCALIZATION=coop) are handled in coop.py on top of this.
"""

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


class Localizer:
    def __init__(self, range_sigma: float = 0.1, accel_sigma: float = 0.5, alpha: float = 0.35,
                 gate: float = 3.29, relock: int = 5, init_sigma: float = 5.0, tick_s: float = 0.02,
                 wind_walk: float = 0.05, wind_sigma0: float = 0.5):
        self.range_sigma = range_sigma
        self.accel_sigma = accel_sigma       # m/s^2, unmodelled acceleration
        self.wind_walk = wind_walk           # m/s per sqrt(s): how fast the wind drift can change
        self.wind_sigma0 = wind_sigma0       # m/s: prior uncertainty of the wind drift
        self.alpha = alpha                   # command response per control tick of tick_s
        self.tick_s = tick_s
        self.gate2 = gate * gate             # 3.29 sigma = chi-square(1) 99.9 %
        self.relock = relock
        self.init_sigma = init_sigma
        self.x: Optional[np.ndarray] = None
        self.P: Optional[np.ndarray] = None
        self.rejected = 0
        self.relocks = 0
        self.updates = 0
        self._run = 0
        self.last_update_t: Optional[float] = None

    # --- state ----------------------------------------------------------
    @property
    def ready(self) -> bool:
        return self.x is not None

    def init_prior(self, x: float, y: float, sigma: Optional[float] = None) -> None:
        s = self.init_sigma if sigma is None else sigma
        w = self.wind_sigma0 ** 2
        self.x = np.array([x, y, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([s * s, s * s, 0.25, 0.25, w, w])

    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    def velocity(self) -> Tuple[float, float]:
        """Velocity over the ground: the airframe's own plus the wind drift."""
        return float(self.x[2] + self.x[4]), float(self.x[3] + self.x[5])

    def wind(self) -> Tuple[float, float]:
        return float(self.x[4]), float(self.x[5])

    def pos_cov(self) -> np.ndarray:
        return self.P[:2, :2].copy()

    def sigma_max(self) -> float:
        """Largest 1-sigma axis of the horizontal position error ellipse (m)."""
        return float(math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2]).max(), 0.0)))

    # --- prediction -----------------------------------------------------
    def predict(self, dt: float, cmd_xy: Tuple[float, float]) -> None:
        """One control tick of dt seconds after commanding horizontal velocity cmd_xy."""
        if self.x is None or dt <= 0.0:
            return
        a = 1.0 - (1.0 - self.alpha) ** (dt / self.tick_s)     # the same response over any step
        F = np.eye(6)
        F[0, 2] = F[1, 3] = (1 - a) * dt
        F[0, 4] = F[1, 5] = dt
        F[2, 2] = F[3, 3] = 1 - a
        u = np.array([a * dt * cmd_xy[0], a * dt * cmd_xy[1], a * cmd_xy[0], a * cmd_xy[1], 0.0, 0.0])
        self.x = F @ self.x + u
        q = (self.accel_sigma * dt) ** 2               # velocity random walk per tick
        G = np.array([[0.5 * dt, 0], [0, 0.5 * dt], [1, 0], [0, 1], [0, 0], [0, 0]])
        Q = q * (G @ G.T)
        Q[4, 4] = Q[5, 5] = self.wind_walk ** 2 * dt
        self.P = F @ self.P @ F.T + Q

    # --- measurements ---------------------------------------------------
    def update_range(self, anchor: Vec3, r: float, z: float, sigma: Optional[float] = None,
                     t: Optional[float] = None) -> bool:
        """A range to a known point (an anchor).  z: our altitude.  Returns False if gated out."""
        if self.x is None:
            return False
        s = self.range_sigma if sigma is None else sigma
        dx, dy, dz = self.x[0] - anchor[0], self.x[1] - anchor[1], z - anchor[2]
        h = math.sqrt(dx * dx + dy * dy + dz * dz)
        if h < 1e-6:
            return False
        H = np.array([[dx / h, dy / h, 0.0, 0.0, 0.0, 0.0]])
        S = float((H @ self.P @ H.T)[0, 0]) + s * s
        y = r - h
        if y * y > self.gate2 * S:
            self.rejected += 1
            self._run += 1
            return False
        self._run = 0
        K = (self.P @ H.T) / S
        self.x = self.x + K[:, 0] * y
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + (K @ K.T) * s * s     # Joseph form: stays symmetric, positive
        self.updates += 1
        if t is not None:
            self.last_update_t = t
        return True

    def needs_relock(self) -> bool:
        return self._run >= self.relock

    # --- initialization from anchors --------------------------------------
    @staticmethod
    def multilaterate(anchors: Sequence[Vec3], ranges: Sequence[float], z: float,
                      guess: Optional[Tuple[float, float]] = None, iters: int = 10):
        """x, y from >= 3 anchor ranges with our altitude z: linear least squares (unless a guess is
        given), then Gauss-Newton.  Returns ((x, y), 2x2 covariance per unit range variance) or None."""
        A = np.array(anchors, dtype=float)
        r = np.array(ranges, dtype=float)
        if len(A) < 3:
            return None
        d2 = np.maximum(r * r - (z - A[:, 2]) ** 2, 0.0)       # squared horizontal ranges
        if guess is None:
            M = 2.0 * (A[1:, :2] - A[0, :2])
            b = d2[0] - d2[1:] + (A[1:, 0] ** 2 - A[0, 0] ** 2) + (A[1:, 1] ** 2 - A[0, 1] ** 2)
            p, *_ = np.linalg.lstsq(M, b, rcond=None)
        else:
            p = np.array(guess, dtype=float)
        H = None
        for _ in range(iters):
            diff = np.column_stack([p[0] - A[:, 0], p[1] - A[:, 1], z - A[:, 2]])
            h = np.linalg.norm(diff, axis=1)
            if np.any(h < 1e-6):
                return None
            H = diff[:, :2] / h[:, None]
            step, *_ = np.linalg.lstsq(H, r - h, rcond=None)
            p = p + step
            if np.linalg.norm(step) < 1e-6:
                break
        HtH = H.T @ H
        if np.linalg.cond(HtH) > 1e8:
            return None
        return (float(p[0]), float(p[1])), np.linalg.inv(HtH)

    def init_from_anchors(self, anchors: Sequence[Vec3], ranges: Sequence[float], z: float,
                          guess: Optional[Tuple[float, float]] = None) -> bool:
        fix = self.multilaterate(anchors, ranges, z, guess)
        if fix is None:
            return False
        (x, y), C = fix
        self.x = np.array([x, y, 0.0, 0.0, 0.0, 0.0])
        self.P = np.zeros((6, 6))
        self.P[:2, :2] = C * self.range_sigma ** 2 + np.eye(2) * 1e-4
        self.P[2, 2] = self.P[3, 3] = 0.25
        self.P[4, 4] = self.P[5, 5] = self.wind_sigma0 ** 2
        self._run = 0
        return True

    def relock_from(self, anchors: Sequence[Vec3], ranges: Sequence[float], z: float) -> bool:
        """Re-initialize from a fresh anchor set after a lock-out (keeps the velocity and wind estimates)."""
        v = self.x[2:].copy() if self.x is not None else np.zeros(4)
        if not self.init_from_anchors(anchors, ranges, z):
            return False
        self.x[2:] = v
        self.relocks += 1
        return True

    def status(self, good: float = 1.0, degraded: float = 3.0) -> str:
        if self.x is None:
            return "lost"
        s = self.sigma_max()
        return "good" if s <= good else "degraded" if s <= degraded else "lost"


def nees(err_xy: Sequence[float], cov: np.ndarray) -> float:
    """Normalized estimation error squared (2 dof): ~2 on average for a consistent estimator."""
    e = np.asarray(err_xy, dtype=float)
    return float(e @ np.linalg.solve(np.asarray(cov, dtype=float), e))


def anchors_from_record(hw: Dict) -> Dict[str, Vec3]:
    """Anchor id -> position (ship frame) from the hardware record."""
    return {f"A{i + 1}": tuple(float(c) for c in p) for i, p in enumerate(hw["uwb_anchors"]["positions_m"])}
