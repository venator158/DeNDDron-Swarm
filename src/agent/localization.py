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

Robust mode (robust=True, UWB_FILTER=robust): the environment can be worse than the record says
(sensing sweep: with 3x the record's noise the plain filter was overconfident, NEES 25, and re-locked
86 times; NLOS ranges drove a re-lock to a 14 m excursion).
- Adaptive noise: the range noise is estimated from the last `noise_window` anchor innovations,
  robustly (median of y^2, minus the part the state uncertainty explains, over the chi-square(1)
  median 0.455), floored at the record's figure and capped at `sigma_cap`.  Gated-out ranges count
  too, or a gate that is too tight would never let the estimate grow.
- Huber update: an innovation beyond `huber_k` sigmas is down-weighted (its noise inflated by
  |e|/k) instead of rejected.  A blocked path (NLOS) only ever lengthens a range, so a range too long
  beyond `gate` is still rejected; one too short only beyond `far_gate`.  With a symmetric Huber,
  moderate NLOS biases (3-8 sigma) pulled the estimate: worst error 1.0 -> 2.8 m in simulation.
- Safe re-lock: a fix from 3+ anchors must pass a residual (chi-square) test at the estimated noise;
  with 4+ anchors, leaving one out is tried, so one blocked-path anchor cannot pull the fix.

IMU mode (imu=hardware.imu_errors(...), NAV_PREDICT=imu): the prediction comes from the flight
controller's accelerometer instead of the command response.  State [x, y, vx, vy, bx, by]: v is the
velocity over the ground (wind included: the accelerometer measures what the drone actually does)
and b the accelerometer bias, its own plus the tilt-equivalent one (g sin tilt).  Each frame brings
the delta-velocity dv over dt_imu: v <- v + dv - b dt, p <- p + v dt + (dv - b dt) dt / 2.  Process
noise: the accelerometer's noise density and scale factor, and the bias as a Gauss-Markov process
(the tilt's correlation time, the shorter one).  The bias is learned while the anchors are good and
carried through an outage, so dead-reckoning error grows from what the bias does since, not from an
unknown wind or command response.

Consistency monitor: every range's normalized innovation (before gating) goes into a window per
source (anchor or peer).  A source whose innovations keep one sign (|mean| sqrt(n) beyond `z`) is
flagged: against the IMU-propagated track, a slowly drifting range (a blocked path that persists, a
mis-surveyed anchor, later a spoofed GNSS) shows up as a bias, while noise does not.  Flags are
reported (telemetry, heartbeats), not acted on.

Peer ranges (LOCALIZATION=coop) are handled in coop.py on top of this.
"""

import math
from collections import deque
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


class InnovationMonitor:
    """Per-source window of normalized innovations e = y / sqrt(S); flags a persistent bias."""

    def __init__(self, window: int = 20, min_n: int = 10, z: float = 3.3):
        self.window, self.min_n, self.z = window, min_n, z
        self._e: Dict[str, deque] = {}

    def record(self, source: str, e: float) -> None:
        self._e.setdefault(source, deque(maxlen=self.window)).append(e)

    def bias(self, source: str) -> Optional[float]:
        """Mean normalized innovation of the window, in units of its standard error (None: too few)."""
        e = self._e.get(source)
        if e is None or len(e) < self.min_n:
            return None
        return float(np.mean(e)) * math.sqrt(len(e))

    def flagged(self) -> list:
        return sorted(s for s in self._e if (b := self.bias(s)) is not None and abs(b) > self.z)


class Localizer:
    CHI2_1_MEDIAN = 0.4549         # median of chi-square with 1 dof
    CHI2_99 = {1: 6.63, 2: 9.21, 3: 11.34, 4: 13.28}   # residual test for a re-lock fix, by dof
    SF_CORRELATION = 10.0          # IMU scale-factor variance multiplier (see predict_imu)

    def __init__(self, range_sigma: float = 0.1, accel_sigma: float = 0.5, alpha: float = 0.35,
                 gate: float = 3.29, relock: int = 5, init_sigma: float = 5.0, tick_s: float = 0.02,
                 wind_walk: float = 0.05, wind_sigma0: float = 0.5, robust: bool = False,
                 huber_k: float = 2.5, far_gate: float = 8.0, noise_window: int = 40, sigma_cap: float = 2.0,
                 imu: Optional[Dict] = None, imu_accel_floor: float = 2e-3, exclude_flagged: bool = False):
        self.imu = imu                       # sim-unit IMU errors (hardware.imu_errors): IMU mode
        if imu is not None:
            self.bias_sigma = math.hypot(imu["accel_bias"], imu["tilt_bias"])
            self.bias_tau = min(imu["accel_bias_tau"], imu["tilt_tau"])
        self.imu_accel_floor = imu_accel_floor   # m/s^2 white, for what the IMU model misses
        self.monitor = InnovationMonitor()
        self.exclude_flagged = exclude_flagged   # stop using a lone flagged source (still watched)
        self.excluded = 0
        self.range_sigma = range_sigma       # the record's figure (the floor in robust mode)
        self.robust = robust
        self.huber_k = huber_k
        self.far_gate2 = far_gate * far_gate
        self.sigma_cap = sigma_cap
        self._innov = deque(maxlen=noise_window)   # anchor innovations: (y^2, H P H')
        self._noise = range_sigma
        self.downweighted = 0
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

    def _tail_var(self) -> float:
        """Prior variance of states 4-5: the wind drift, or in IMU mode the accelerometer bias."""
        return self.bias_sigma ** 2 if self.imu is not None else self.wind_sigma0 ** 2

    def init_prior(self, x: float, y: float, sigma: Optional[float] = None) -> None:
        s = self.init_sigma if sigma is None else sigma
        w = self._tail_var()
        self.x = np.array([x, y, 0.0, 0.0, 0.0, 0.0])
        self.P = np.diag([s * s, s * s, 0.25, 0.25, w, w])

    def position(self) -> Tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    def velocity(self) -> Tuple[float, float]:
        """Velocity over the ground: the airframe's own plus the wind drift (IMU mode: the state)."""
        if self.imu is not None:
            return float(self.x[2]), float(self.x[3])
        return float(self.x[2] + self.x[4]), float(self.x[3] + self.x[5])

    def wind(self) -> Tuple[float, float]:
        """The learned wind drift (IMU mode: not a state; the ground velocity carries it)."""
        return (0.0, 0.0) if self.imu is not None else (float(self.x[4]), float(self.x[5]))

    def accel_bias(self) -> Tuple[float, float]:
        return (float(self.x[4]), float(self.x[5])) if self.imu is not None else (0.0, 0.0)

    def pos_cov(self) -> np.ndarray:
        return self.P[:2, :2].copy()

    def noise_sigma(self) -> float:
        """Range noise (1 sigma) the filter uses: the record's, or in robust mode the estimate."""
        return self._noise if self.robust else self.range_sigma

    def _learn_noise(self, y2: float, hph: float) -> None:
        self._innov.append((y2, hph))
        if len(self._innov) < 12:
            return
        r = float(np.median([v - self.CHI2_1_MEDIAN * h for v, h in self._innov])) / self.CHI2_1_MEDIAN
        self._noise = min(max(math.sqrt(max(r, 0.0)), self.range_sigma), self.sigma_cap)

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

    def predict_imu(self, dt: float, dv: Tuple[float, float], dt_imu: Optional[float] = None) -> None:
        """IMU mode: advance dt seconds with the accelerometer's delta-velocity dv measured over
        dt_imu (normally the same interval; a longer dt, e.g. after a lost frame, is flown at
        constant velocity for the rest)."""
        if self.x is None or dt <= 0.0:
            return
        dti = dt if dt_imu is None or dt_imu <= 0.0 else min(dt_imu, dt)
        imu = self.imu
        k = math.exp(-dt / self.bias_tau)
        F = np.eye(6)
        F[0, 2] = F[1, 3] = dt
        F[0, 4] = F[1, 5] = -dti * (dt - 0.5 * dti)     # the bias over the IMU interval, then carried
        F[2, 4] = F[3, 5] = -dti
        F[4, 4] = F[5, 5] = k
        dvx, dvy = float(dv[0]), float(dv[1])
        u = np.array([dvx * (dt - 0.5 * dti), dvy * (dt - 0.5 * dti), dvx, dvy, 0.0, 0.0])
        self.x = F @ self.x + u
        # Velocity noise: the noise density over the interval, the scale factor on what was measured,
        # and a floor for what the model misses; it enters position through the same interval.  The
        # scale factor is a constant per axis, so its errors add up over a manoeuvre instead of
        # averaging out; counted as white noise with SF_CORRELATION x its variance (calibrated in
        # simulation: x1 gave NEES 3.1, x10 1.7 with 96 % inside the 95 % ellipse, more is pessimistic).
        qv = imu["noise_density"] ** 2 * dti \
            + self.SF_CORRELATION * (imu["scale_factor"] * math.hypot(dvx, dvy)) ** 2 \
            + (self.imu_accel_floor * dt) ** 2
        G = np.array([[0.5 * dt, 0], [0, 0.5 * dt], [1, 0], [0, 1], [0, 0], [0, 0]])
        Q = qv * (G @ G.T)
        # When within the frame the velocity changed is unknown (the airframe responds to commands as
        # they arrive): the mid-frame assumption is off by up to dv dt / 2, uniform: variance (dv dt)^2/12.
        Q[0, 0] += (dvx * dti) ** 2 / 12.0
        Q[1, 1] += (dvy * dti) ** 2 / 12.0
        Q[4, 4] = Q[5, 5] = self.bias_sigma ** 2 * (1.0 - k * k)
        self.P = F @ self.P @ F.T + Q

    # --- measurements ---------------------------------------------------
    def update_range(self, anchor: Vec3, r: float, z: float, sigma: Optional[float] = None,
                     t: Optional[float] = None, source: Optional[str] = None) -> bool:
        """A range to a known point.  z: our altitude.  sigma: its noise if not the filter's own
        (peers: theirs added); only ranges with sigma=None (anchors) train the noise estimate.
        source: the anchor's or peer's id, for the consistency monitor.  Returns False if gated out."""
        if self.x is None:
            return False
        s = self.noise_sigma() if sigma is None else sigma
        dx, dy, dz = self.x[0] - anchor[0], self.x[1] - anchor[1], z - anchor[2]
        h = math.sqrt(dx * dx + dy * dy + dz * dz)
        if h < 1e-6:
            return False
        H = np.array([[dx / h, dy / h, 0.0, 0.0, 0.0, 0.0]])
        hph = float((H @ self.P @ H.T)[0, 0])
        S = hph + s * s
        y = r - h
        if source is not None:
            self.monitor.record(source, y / math.sqrt(S))
            # One source drifting against the track: watched, not used, until it agrees again.  Several
            # at once is the environment (widespread NLOS) or our own estimate, not a culprit: keep them.
            if self.exclude_flagged and self.monitor.flagged() == [source]:
                self.excluded += 1
                return False
        # Robust handling is for anchor ranges only.  A peer's error is its estimate's, not a blocked
        # path: with Huber and the far gate on peers, peer-only drones beyond anchor range were pulled
        # far off (uwb_short: NEES 50 -> 399); consistent peer fusion is RDL's job.
        robust = self.robust and sigma is None
        if robust:
            self._learn_noise(y * y, hph)
        # Robust: a blocked path only ever lengthens a range, so a range too long beyond the gate is
        # rejected as in the plain filter; one too short is only rejected beyond far_gate.
        gate2 = self.far_gate2 if robust and y < 0 else self.gate2
        if y * y > gate2 * S:
            self.rejected += 1
            self._run += 1
            return False
        self._run = 0
        if robust and y * y > self.huber_k ** 2 * S:
            # Huber: beyond k sigmas the range counts as if its noise were |e|/k times larger
            w = self.huber_k / math.sqrt(y * y / S)
            s = s / math.sqrt(w)
            S = hph + s * s
            self.downweighted += 1
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

    def consistent_fix(self, anchors: Sequence[Vec3], ranges: Sequence[float], z: float):
        """A fix whose residuals fit the noise: all anchors, else (4+) the best leave-one-out.
        Returns ((x, y), C) or None.  Needs 3+ anchors (2 unknowns: one range left to check)."""
        s2 = self.noise_sigma() ** 2
        n = len(anchors)
        sets = [list(range(n))] + ([[j for j in range(n) if j != i] for i in range(n)] if n >= 4 else [])
        best = None
        for idx in sets:
            if len(idx) < 3:
                continue
            A = [anchors[i] for i in idx]
            r = [ranges[i] for i in idx]
            fix = self.multilaterate(A, r, z)
            if fix is None:
                continue
            (x, y), _ = fix
            chi2 = sum((ri - math.dist((x, y, z), a)) ** 2 for a, ri in zip(A, r)) / s2
            if chi2 <= self.CHI2_99.get(len(idx) - 2, 13.28) and (best is None or chi2 < best[0]):
                best = (chi2, fix)
            if best is not None and idx is sets[0]:
                break                      # all anchors agree: no need to leave one out
        return None if best is None else best[1]

    def init_from_anchors(self, anchors: Sequence[Vec3], ranges: Sequence[float], z: float,
                          guess: Optional[Tuple[float, float]] = None) -> bool:
        if self.robust:
            fix = self.consistent_fix(anchors, ranges, z)
        else:
            fix = self.multilaterate(anchors, ranges, z, guess)
        if fix is None:
            return False
        (x, y), C = fix
        self.x = np.array([x, y, 0.0, 0.0, 0.0, 0.0])
        self.P = np.zeros((6, 6))
        self.P[:2, :2] = C * self.noise_sigma() ** 2 + np.eye(2) * 1e-4
        self.P[2, 2] = self.P[3, 3] = 0.25
        self.P[4, 4] = self.P[5, 5] = self._tail_var()
        self._run = 0
        return True

    def relock_from(self, anchors: Sequence[Vec3], ranges: Sequence[float], z: float) -> bool:
        """Re-initialize from a fresh anchor set after a lock-out (keeps the velocity and wind estimates).
        Robust mode: only from a fix that passes the residual test (else False: keep the filter)."""
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
