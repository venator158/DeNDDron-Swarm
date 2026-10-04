"""GNSS comparator and fallback (src/agent/gnss.py) on a simulated drone with the flight controller's IMU."""
import math
import random
import unittest

import numpy as np

import hardware
from gnss import Gnss
from localization import Localizer

ANCHORS = [(15.0, 5.0, 8.0), (15.0, -5.0, 8.0), (-15.0, 5.0, 8.0), (-15.0, -5.0, 8.0)]
HW = hardware.load()
IMU = hardware.imu_errors(HW, "dilated")
ERR = hardware.gnss_errors(HW)
ORIGIN = np.array(HW["ship_gnss"]["geo_origin_enu_m"])
HEADING = math.radians(HW["ship_gnss"]["heading_deg"])


class GM:
    """Gauss-Markov error, 2 axes."""

    def __init__(self, rng, sigma, tau):
        self.rng, self.sigma, self.tau = rng, sigma, tau
        self.v = np.array([rng.gauss(0, sigma), rng.gauss(0, sigma)])

    def step(self, dt):
        k = math.exp(-dt / self.tau)
        s = self.sigma * math.sqrt(1 - k * k)
        self.v = self.v * k + np.array([self.rng.gauss(0, s), self.rng.gauss(0, s)])
        return self.v


def fly(seconds=240.0, seed=1, jam=None, spoof=None, offset=None, use_gnss=True, fallback=True,
        waypoints=((90.0, -30.0), (60.0, 40.0), (100.0, 10.0)), start=(70.0, 20.0), z=20.0):
    """jam: (t0, t1) anchors jammed; spoof: (t0, direction rad, rate m/s, step m) on the drone's fix;
    offset: (t, metres) pulls the estimate off at t, as a bad peer chain would.
    Returns (errors, gnss states over time, gnss, localizer)."""
    rng = random.Random(seed)
    dt = 0.02
    loc = Localizer(robust=True, imu=IMU)
    loc.init_prior(start[0], start[1], sigma=3.0)
    g = Gnss(loc, ERR, fallback=fallback)
    common = GM(rng, ERR["common_sigma"], ERR["common_tau"])
    own, ship_rx = GM(rng, ERR["receiver_sigma"], ERR["receiver_tau"]), GM(rng, ERR["receiver_sigma"], ERR["receiver_tau"])
    heading_err = rng.gauss(0, ERR["heading_sigma"])
    p, v = np.array(start, float), np.zeros(2)
    b = np.array([rng.gauss(0, IMU["tilt_bias"]), rng.gauss(0, IMU["tilt_bias"])])
    errs, states = [], []
    wp, t, next_uwb, next_gnss = 0, 0.0, 0.0, 0.0
    c, s = math.cos(HEADING), math.sin(HEADING)
    while t < seconds:
        goal = np.array(waypoints[wp % len(waypoints)])
        if np.linalg.norm(goal - p) < 2.0:
            wp += 1
        d = goal - p
        cmd = d / max(np.linalg.norm(d), 1e-6) * min(4.0, np.linalg.norm(d))
        v_old = v
        v = v + 0.35 * (cmd - v)
        u = rng.random()
        p = p + ((1 - u) * v_old + u * v) * dt
        dv = (v - v_old) + b * dt + np.array([rng.gauss(0, IMU["noise_density"] * math.sqrt(dt)) for _ in range(2)])
        loc.predict_imu(dt, (float(dv[0]), float(dv[1])), dt)
        g.shadow_predict_imu(dt, (float(dv[0]), float(dv[1])), dt)
        t += dt
        if offset and abs(t - offset[0]) < dt / 2:
            loc.x[0] += offset[1]                       # a bad peer chain pulled us off, confidently
        if t >= next_uwb:
            next_uwb += 0.5
            if not (jam and jam[0] <= t < jam[1]):
                for a in ANCHORS:
                    loc.update_range(a, math.dist((p[0], p[1], z), a) + rng.gauss(0, 0.1), z, t=t, source=str(a))
        if t >= next_gnss and use_gnss:
            next_gnss += 1.0 / ERR["rate_hz"]
            gd = 1.0 / ERR["rate_hz"]
            cm = common.step(gd)
            ship_fix = ORIGIN + cm + ship_rx.step(gd) + rng.gauss(0, ERR["white_sigma"])
            g.on_ship(t, ship_fix, HEADING + heading_err, True)
            world = ORIGIN + np.array([c * p[0] - s * p[1], s * p[0] + c * p[1]])
            fix = world + cm + own.step(gd) + np.array([rng.gauss(0, ERR["white_sigma"]) for _ in range(2)])
            if spoof and t >= spoof[0]:
                fix = fix + (spoof[3] + spoof[2] * (t - spoof[0])) * np.array([math.cos(spoof[1]), math.sin(spoof[1])])
            states.append((t, g.on_fix(t, fix)))
        if t > 10.0:
            errs.append((t, float(np.linalg.norm(np.array(loc.position()) - p))))
    return errs, states, g, loc


def worst(errs, t0=0.0, t1=1e9):
    return max(e for t, e in errs if t0 <= t < t1)


class TestGnss(unittest.TestCase):
    def test_relative_fix_cancels_the_common_error_and_rotates(self):
        loc = Localizer(imu=IMU)
        loc.init_prior(0.0, 0.0)
        g = Gnss(loc, ERR)
        g.on_ship(0.0, ORIGIN + np.array([7.0, -3.0]), HEADING, True)
        c, s = math.cos(HEADING), math.sin(HEADING)
        world = ORIGIN + np.array([c * 60.0 - s * 20.0, s * 60.0 + c * 20.0]) + np.array([7.0, -3.0])
        x, y = g.relative(world)
        self.assertAlmostEqual(x, 60.0, places=6)
        self.assertAlmostEqual(y, 20.0, places=6)

    def test_no_false_spoof_with_good_uwb(self):
        for seed in (1, 2, 3, 4):
            _, states, g, _ = fly(seed=seed, seconds=300.0)
            self.assertNotEqual(g.state, "spoofed", seed)
            self.assertEqual(g.fused, 0)                # never fused while UWB is available

    def test_step_spoof_detected_while_anchored(self):
        _, _, g, _ = fly(spoof=(60.0, 0.5, 0.0, 10.0))
        self.assertEqual(g.state, "spoofed")
        self.assertLess(g.spoof_t - 60.0, 5.0)

    def test_ramp_spoof_detected_while_anchored(self):
        errs, _, g, _ = fly(spoof=(60.0, 1.0, 0.1, 0.0))
        self.assertEqual(g.state, "spoofed")
        self.assertLess(g.spoof_t - 60.0, 60.0)
        self.assertLess(worst(errs, 60.0), 1.5)          # UWB carried on; GNSS was never fused

    def test_fallback_holds_position_through_a_uwb_outage(self):
        jam = (60.0, 180.0)
        with_gnss, _, g, _ = fly(jam=jam)
        without, _, _, _ = fly(jam=jam, use_gnss=False)
        self.assertGreater(g.fused, 50)
        self.assertNotEqual(g.state, "spoofed")
        self.assertLess(worst(with_gnss, *jam), worst(without, *jam))
        self.assertLess(worst(with_gnss, *jam), 4.0)

    def test_spoof_during_the_outage_is_caught_by_the_imu_check(self):
        jam = (60.0, 180.0)
        errs, _, g, _ = fly(jam=jam, spoof=(100.0, 2.0, 0.0, 15.0))
        self.assertEqual(g.state, "spoofed")
        self.assertLess(g.spoof_t - 100.0, 5.0)
        self.assertLess(worst(errs, 100.0, 180.0), 6.0)

    def test_gnss_corrects_a_confidently_wrong_estimate_without_anchors(self):
        # Beyond anchor range a peer chain pulled the estimate 15 m off (uwb_short); the IMU-only
        # reference still agrees with GNSS, so GNSS stands in and brings it back.
        errs, _, g, _ = fly(jam=(60.0, 240.0), offset=(61.0, 15.0))
        self.assertNotEqual(g.state, "spoofed")
        self.assertLess(worst(errs, 120.0, 240.0), 3.0)


if __name__ == "__main__":
    unittest.main()
