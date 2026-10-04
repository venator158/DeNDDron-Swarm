"""Drone self-localization from UWB anchor ranges (src/agent/localization.py)."""
import math
import random
import unittest

import numpy as np

from localization import Localizer, nees

ANCHORS = [(15.0, 5.0, 8.0), (15.0, -5.0, 8.0), (-15.0, 5.0, 8.0), (-15.0, -5.0, 8.0)]   # hardware record


def fly(seconds=120.0, start=(70.0, 20.0), z=20.0, sigma=0.1, rate_hz=2.0, seed=1, disturb=0.2,
        outlier=None, dropout=0.0, alpha=0.35, waypoints=((90.0, -30.0), (60.0, 40.0), (100.0, 10.0)),
        wind=(0.0, 0.0), jam=None, true_sigma=None, nlos=None, robust=False, relock=False):
    """A drone flying waypoints under the simulator's command response, plus an unmodelled gust.
    sigma: the record's range noise (what the filter assumes); true_sigma: the environment's (default
    the same); nlos: (probability, mean bias m) of a blocked path (positive, exponential); relock:
    re-lock from the round's anchors after a lock-out, as the drone does.
    Returns (errors, nees values, localizer)."""
    rng = random.Random(seed)
    dt = 0.02
    true_sigma = sigma if true_sigma is None else true_sigma
    loc = Localizer(range_sigma=sigma, robust=robust)
    loc.init_prior(start[0] + rng.gauss(0, 3), start[1] + rng.gauss(0, 3))
    p = np.array(start, float)
    v = np.zeros(2)
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
        v = v + alpha * (cmd - v) + gust * dt          # the truth: response + unmodelled gust
        p = p + (v + np.array(wind)) * dt              # wind the drone cannot sense
        loc.predict(dt, (float(cmd[0]), float(cmd[1])))
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
                loc.update_range(a, r, z, t=t)
                got.append((a, r))
            if relock and loc.needs_relock() and len(got) >= 3:
                loc.relock_from([a for a, _ in got], [r for _, r in got], z)
            e = np.array(loc.position()) - p
            if t > 10.0:
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


if __name__ == "__main__":
    unittest.main()
