"""Drone self-localization from UWB anchor ranges (src/agent/localization.py)."""
import math
import random
import unittest

import numpy as np

import hardware
from localization import InnovationMonitor, Localizer, nees

IMU = hardware.imu_errors(hardware.load(), "dilated")   # the flight controller's IMU, simulation units

ANCHORS = [(15.0, 5.0, 8.0), (15.0, -5.0, 8.0), (-15.0, 5.0, 8.0), (-15.0, -5.0, 8.0)]   # hardware record


def fly(seconds=120.0, start=(70.0, 20.0), z=20.0, sigma=0.1, rate_hz=2.0, seed=1, disturb=0.2,
        outlier=None, dropout=0.0, alpha=0.35, waypoints=((90.0, -30.0), (60.0, 40.0), (100.0, 10.0)),
        wind=(0.0, 0.0), jam=None, true_sigma=None, nlos=None, robust=False, relock=False,
        imu=None, imu_bias_extra=0.0, anchor_bias=None, errs_from=10.0, exclude=False):
    """A drone flying waypoints under the simulator's command response, plus an unmodelled gust.
    sigma: the record's range noise (what the filter assumes); true_sigma: the environment's (default
    the same); nlos: (probability, mean bias m) of a blocked path (positive, exponential); relock:
    re-lock from the round's anchors after a lock-out, as the drone does.
    imu: IMU errors (hardware.imu_errors) to predict with the accelerometer instead of the command
    (the truth then includes the flight controller's accelerometer and tilt biases, noise and scale
    factor); imu_bias_extra: m/s^2 added to the true bias; anchor_bias: (index, metres) on one anchor.
    Returns (errors, nees values, localizer)."""
    rng = random.Random(seed)
    dt = 0.02
    true_sigma = sigma if true_sigma is None else true_sigma
    loc = Localizer(range_sigma=sigma, robust=robust, imu=imu, exclude_flagged=exclude)
    loc.init_prior(start[0] + rng.gauss(0, 3), start[1] + rng.gauss(0, 3))
    p = np.array(start, float)
    v = np.zeros(2)
    if imu is not None:
        b_acc = np.array([rng.gauss(0, imu["accel_bias"]) for _ in range(2)])
        b_tilt = np.array([rng.gauss(0, imu["tilt_bias"]) for _ in range(2)])
        sf = np.array([rng.gauss(0, imu["scale_factor"]) for _ in range(2)])
        vg_last = v + np.array(wind)
    errs, ns = [], []
    wp = 0
    t = 0.0
    next_range = 0.0
    while t < seconds:
        goal = np.array(waypoints[wp % len(waypoints)])
        if np.linalg.norm(goal - p) < 2.0:
            wp += 1
        d = goal - p
        cmd = d / max(np.linalg.norm(d), 1e-6) * min(4.0, np.linalg.norm(d))
        gust = np.array([rng.gauss(0, disturb), rng.gauss(0, disturb)])
        v_old = v
        v = v + alpha * (cmd - v) + gust * dt          # the truth: response + unmodelled gust
        if imu is None:
            p = p + (v + np.array(wind)) * dt          # wind the drone cannot sense
        else:
            # as in the simulator, the velocity changes when the command arrives, somewhere in the frame
            u = rng.random()
            p = p + ((1 - u) * v_old + u * v + np.array(wind)) * dt
        if imu is None:
            loc.predict(dt, (float(cmd[0]), float(cmd[1])))
        else:
            # the flight controller's accelerometer: true delta-velocity over the ground, with errors
            for b, tau, sig in ((b_acc, imu["accel_bias_tau"], imu["accel_bias"]),
                                (b_tilt, imu["tilt_tau"], imu["tilt_bias"])):
                k = math.exp(-dt / tau)
                b *= k
                b += np.array([rng.gauss(0, sig * math.sqrt(1 - k * k)) for _ in range(2)])
            vg = v + np.array(wind)
            dv = (1 + sf) * (vg - vg_last) + (b_acc + b_tilt + imu_bias_extra) * dt \
                + np.array([rng.gauss(0, imu["noise_density"] * math.sqrt(dt)) for _ in range(2)])
            vg_last = vg
            loc.predict_imu(dt, (float(dv[0]), float(dv[1])), dt)
        t += dt
        if t >= next_range:
            next_range += 1.0 / rate_hz
            got = []
            for i, a in enumerate(ANCHORS):
                if jam and jam[0] <= t < jam[1]:
                    break
                if rng.random() < dropout:
                    continue
                r = math.dist((p[0], p[1], z), a) + rng.gauss(0, true_sigma)
                if outlier and outlier[0] <= t < outlier[1] and i == 0:
                    r += 5.0                            # a blocked path: +5 m bias on one anchor
                if nlos and rng.random() < nlos[0]:
                    r += rng.expovariate(1.0 / nlos[1])
                if anchor_bias and anchor_bias[0] == i:
                    # (index, metres) from the start, or (index, m/s, start) a drift ramping in
                    r += anchor_bias[1] if len(anchor_bias) == 2 else anchor_bias[1] * max(0.0, t - anchor_bias[2])
                loc.update_range(a, r, z, t=t, source=f"A{i + 1}")
                got.append((a, r))
            if relock and loc.needs_relock() and len(got) >= 3:
                loc.relock_from([a for a, _ in got], [r for _, r in got], z)
            e = np.array(loc.position()) - p
            if t > errs_from:
                errs.append(float(np.linalg.norm(e)))
                ns.append(nees(e, loc.pos_cov()))
    return errs, ns, loc


class TestLocalizer(unittest.TestCase):
    def test_multilaterate_exact(self):
        for p in ((60.0, 10.0), (-80.0, 45.0), (5.0, -120.0)):
            ranges = [math.dist((p[0], p[1], 20.0), a) for a in ANCHORS]
            (x, y), C = Localizer.multilaterate(ANCHORS, ranges, 20.0)
            self.assertAlmostEqual(x, p[0], places=5)
            self.assertAlmostEqual(y, p[1], places=5)
            self.assertEqual(C.shape, (2, 2))

    def test_tracks_a_moving_drone_consistently(self):
        errs, ns, loc = fly()
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        self.assertLess(rms, 1.0)
        mean_nees = sum(ns) / len(ns)
        self.assertGreater(mean_nees, 0.7)          # not wildly pessimistic
        self.assertLess(mean_nees, 4.0)             # not overconfident (2 for a perfect filter)
        within = sum(1 for n in ns if n <= 5.99) / len(ns)
        self.assertGreater(within, 0.85)            # 95 % ideally

    def test_outlier_anchor_is_gated(self):
        errs, _, loc = fly(outlier=(40.0, 60.0))
        self.assertGreater(loc.rejected, 10)
        self.assertLess(max(errs), 2.5)

    def test_dropouts_degrade_gracefully(self):
        errs, _, _ = fly(dropout=0.5)
        self.assertLess(math.sqrt(sum(e * e for e in errs) / len(errs)), 1.5)

    def test_wind_is_learned(self):
        # A steady 0.5 m/s wind the drone cannot sense (live, a filter without a wind state was
        # 600x overconfident and re-locked 290 times in one run).
        errs, ns, loc = fly(wind=(0.5, -0.2))
        self.assertLess(math.sqrt(sum(e * e for e in errs) / len(errs)), 0.5)
        self.assertLess(sum(ns) / len(ns), 4.0)
        self.assertEqual(loc.relocks, 0)
        self.assertAlmostEqual(loc.wind()[0], 0.5, delta=0.1)
        self.assertAlmostEqual(loc.wind()[1], -0.2, delta=0.1)

    def test_outage_dead_reckons_with_the_wind(self):
        # anchors jammed for 20 s in a steady wind: the learned wind keeps the drift small
        errs, _, loc = fly(wind=(0.5, 0.0), jam=(60.0, 80.0), seconds=85.0)
        self.assertLess(max(errs), 3.0)

    def test_relock_after_a_corrupted_estimate(self):
        loc = Localizer()
        loc.init_prior(70.0, 20.0, sigma=0.3)
        loc.P[:2, :2] = np.eye(2) * 0.01                 # confident ...
        loc.x[:2] += np.array([25.0, 0.0])               # ... and 25 m wrong
        ranges = [math.dist((70.0, 20.0, 20.0), a) for a in ANCHORS]
        for _ in range(3):
            for a, r in zip(ANCHORS, ranges):
                loc.update_range(a, r, 20.0)
        self.assertTrue(loc.needs_relock())
        self.assertTrue(loc.relock_from(ANCHORS, ranges, 20.0))
        self.assertLess(math.dist(loc.position(), (70.0, 20.0)), 0.1)

    def test_uncertainty_grows_without_ranges(self):
        loc = Localizer()
        loc.init_prior(70.0, 20.0, sigma=0.3)
        s0 = loc.sigma_max()
        for _ in range(500):                              # 10 s, no measurements
            loc.predict(0.02, (2.0, 0.0))
        self.assertGreater(loc.sigma_max(), s0)
        self.assertEqual(loc.status(), "lost" if loc.sigma_max() > 3 else loc.status())


class TestRobustLocalizer(unittest.TestCase):
    """UWB_FILTER=robust: adaptive noise, Huber update, safe re-locks (environment worse than the record)."""

    def run_seeds(self, seeds=(1, 2, 3), **kw):
        out = [fly(seed=s, relock=True, **kw) for s in seeds]
        nees_mean = sum(sum(n) / len(n) for _, n, _ in out) / len(out)
        return nees_mean, max(max(e) for e, _, _ in out), sum(l.relocks for _, _, l in out), out

    def test_tracks_like_the_plain_filter_at_the_record_noise(self):
        nees_mean, worst, relocks, out = self.run_seeds(robust=True)
        self.assertGreater(nees_mean, 0.7)
        self.assertLess(nees_mean, 4.0)
        self.assertLess(worst, 1.5)
        self.assertEqual(relocks, 0)
        for _, _, loc in out:
            self.assertAlmostEqual(loc.noise_sigma(), 0.1, delta=0.03)   # floored at the record

    def test_noise_three_times_the_record(self):
        # The plain filter keeps assuming 0.1 m: overconfident and re-locking (live: NEES 25, 86 re-locks).
        plain, _, plain_relocks, _ = self.run_seeds(true_sigma=0.3)
        self.assertGreater(plain, 8.0)
        nees_mean, worst, relocks, out = self.run_seeds(true_sigma=0.3, robust=True)
        self.assertLess(nees_mean, 4.0)
        self.assertEqual(relocks, 0)
        self.assertLess(worst, 3.0)
        for _, _, loc in out:
            self.assertGreater(loc.noise_sigma(), 0.2)                    # learned, not the record
            self.assertLess(loc.noise_sigma(), 0.5)

    def test_noise_and_blocked_paths(self):
        # 3x noise and 10 % NLOS (+1 m mean): the plain filter re-locked onto bad fixes (13.9 m)
        nees_mean, worst, relocks, _ = self.run_seeds(true_sigma=0.3, nlos=(0.1, 1.0), robust=True)
        self.assertLess(nees_mean, 4.0)
        self.assertLess(worst, 5.0)
        self.assertEqual(relocks, 0)

    def corrupted(self, robust):
        loc = Localizer(robust=robust)
        loc.init_prior(70.0, 20.0, sigma=0.3)
        loc.P[:2, :2] = np.eye(2) * 0.01                 # confident and 25 m wrong
        loc.x[:2] += np.array([25.0, 0.0])
        return loc

    def test_relock_leaves_out_a_blocked_anchor(self):
        ranges = [math.dist((70.0, 20.0, 20.0), a) for a in ANCHORS]
        ranges[0] += 5.0                                 # one anchor's path blocked
        plain = self.corrupted(robust=False)
        self.assertTrue(plain.relock_from(ANCHORS, ranges, 20.0))
        self.assertGreater(math.dist(plain.position(), (70.0, 20.0)), 1.0)   # pulled by the bad range
        loc = self.corrupted(robust=True)
        self.assertTrue(loc.relock_from(ANCHORS, ranges, 20.0))
        self.assertLess(math.dist(loc.position(), (70.0, 20.0)), 0.3)

    def test_relock_refused_when_three_anchors_disagree(self):
        ranges = [math.dist((70.0, 20.0, 20.0), a) for a in ANCHORS[:3]]
        ranges[0] += 5.0
        loc = self.corrupted(robust=True)
        before = loc.position()
        self.assertFalse(loc.relock_from(ANCHORS[:3], ranges, 20.0))      # residual test fails
        self.assertEqual(loc.position(), before)
        self.assertEqual(loc.relocks, 0)

    def test_relock_from_three_consistent_anchors(self):
        ranges = [math.dist((70.0, 20.0, 20.0), a) + 0.05 for a in ANCHORS[:3]]
        loc = self.corrupted(robust=True)
        self.assertTrue(loc.relock_from(ANCHORS[:3], ranges, 20.0))
        self.assertLess(math.dist(loc.position(), (70.0, 20.0)), 0.5)


def summary(runs):
    """(mean NEES, mean p95 error, worst error) over fly() runs."""
    nees_mean = sum(sum(n) / len(n) for _, n, _ in runs) / len(runs)
    p95 = sum(sorted(e)[int(0.95 * len(e))] for e, _, _ in runs) / len(runs)
    return nees_mean, p95, max(max(e) for e, _, _ in runs)


class TestImuPrediction(unittest.TestCase):
    """NAV_PREDICT=imu: the flight controller's accelerometer (bias, tilt, noise, scale factor) drives
    the prediction instead of the command response."""

    def test_consistent_and_tighter_than_the_command_model(self):
        cmd = summary([fly(seed=s) for s in (1, 2, 3)])
        imu = summary([fly(seed=s, imu=IMU) for s in (1, 2, 3)])
        self.assertGreater(imu[0], 0.7)
        self.assertLess(imu[0], 4.0)
        self.assertLess(imu[1], cmd[1])                     # p95 0.32 vs 0.59 m

    def test_dead_reckons_through_a_60s_anchor_outage(self):
        kw = dict(jam=(60.0, 120.0), seconds=125.0, wind=(0.5, 0.0))
        cmd = summary([fly(seed=s, **kw) for s in (1, 2, 3)])
        imu = summary([fly(seed=s, imu=IMU, **kw) for s in (1, 2, 3)])
        self.assertLess(imu[0], 4.0)                        # honest through the outage
        self.assertLess(imu[2], cmd[2])                     # worst 5.7 vs 8.5 m (5 seeds)

    def test_unscaled_errors_are_a_stress_setting(self):
        # The record's figures unscaled: ~100x the drift per mission phase at speed_scale 0.1
        real = hardware.imu_errors(hardware.load(), "real")
        _, _, worst = summary([fly(seed=1, imu=real, jam=(60.0, 120.0), seconds=125.0)])
        self.assertGreater(worst, 20.0)


class TestDriftMonitor(unittest.TestCase):
    """InnovationMonitor: a source whose innovations keep one sign against the track is flagged."""

    def test_monitor_flags_a_persistent_bias_only(self):
        m = InnovationMonitor()
        rng = random.Random(3)
        for _ in range(20):
            m.record("A1", rng.gauss(1.5, 1.0))             # biased by 1.5 sigma
            m.record("A2", rng.gauss(0.0, 1.0))
        self.assertEqual(m.flagged(), ["A1"])
        self.assertIsNone(InnovationMonitor().bias("A1"))  # too few samples

    def test_no_flags_in_clean_flights(self):
        for s in (1, 2, 3, 4):
            _, _, loc = fly(seed=s, imu=IMU, robust=True, seconds=240.0)
            self.assertEqual(loc.monitor.flagged(), [])

    def test_a_drifting_anchor_is_flagged_and_left_out(self):
        drift = (0, 0.02, 60.0)                             # anchor 1: 2 cm/s from t = 60 s (3.6 m by 240 s)
        kept = summary([fly(seed=s, imu=IMU, robust=True, seconds=240.0, anchor_bias=drift) for s in (1, 2, 3)])
        runs = [fly(seed=s, imu=IMU, robust=True, seconds=240.0, anchor_bias=drift, exclude=True) for s in (1, 2, 3)]
        for _, _, loc in runs:
            self.assertEqual(loc.monitor.flagged(), ["A1"])
            self.assertGreater(loc.excluded, 100)
        left_out = summary(runs)
        self.assertGreater(kept[0], 5.0)                    # used, it pulls the estimate (NEES 9.6)
        self.assertLess(left_out[0], 4.0)
        self.assertLess(left_out[2], 1.5)                   # worst 0.74 m (8 seeds) vs 4.2 m


if __name__ == "__main__":
    unittest.main()
