"""Proximity fuze (src/agent/fuze.py): one engagement, scanned at 50 Hz with noise."""
import math
import random
import unittest

from fuze import Decision, Fuze, FuzeConfig, enabled

OWN = (0.0, 0.0, 20.0)
MATES = [(0.0, 0.0, 23.0), (0.0, 0.0, 17.0)]           # stacked +-3 m (vertical slots)


def run(cfg=FuzeConfig(), speed=2.5, t_engage=30.0, arrive_late=1.1, lateral=1.0, noise=0.1, true_track=None,
        extra=lambda t: [], seed=1, until=None, predict_track=None, own=OWN):
    """Threat passes over the engagement point; it reaches OWN's x at t_engage + arrive_late
    (drones stop short of the point, so threats arrive ~1.1 s late), `lateral` m to the side.
    Returns (decision, protocol time of the decision)."""
    OWN = own
    rng = random.Random(seed)
    t_pass = t_engage + arrive_late
    track = true_track or (lambda t: (speed * (t - t_pass), lateral, 20.0))
    # The job's track (what the ship predicts): passes the point exactly at t_engage.
    predict = predict_track or (lambda t: (speed * (t - t_engage), 0.0, 20.0))
    fz = Fuze(cfg)
    t = t_engage - 20.0
    until = until or t_engage + cfg.window_s + 1.0
    while t <= until:
        objs = [m for m in MATES] + [track(t)] + extra(t)
        rel = [(o[0] - OWN[0] + rng.gauss(0, noise), o[1] - OWN[1] + rng.gauss(0, noise), o[2] - OWN[2] + rng.gauss(0, noise))
               for o in objs if math.dist(o, OWN) <= cfg.range_m]
        d = fz.update(t, OWN, rel, t_engage, predict)
        if d is not None:
            return d, t, fz
        t += 0.02
    return None, t, fz


class TestFuze(unittest.TestCase):
    def test_fires_at_closest_approach(self):
        for speed in (2.5, 3.5, 4.5):
            d, t, _ = run(speed=speed)
            self.assertEqual((d.action, d.reason), ("fire", "fuze"), speed)
            t_cpa = 30.0 + 1.1
            self.assertAlmostEqual(t, t_cpa, delta=0.12)                     # velocity fit over 0.3 s
            self.assertLess(d.range_m, 1.4)                                  # 1 m lateral + noise

    def test_threat_in_range_before_arming_is_not_taken_for_a_mate(self):
        # at t_engage - 2 s a 2.5 m/s threat is ~7.75 m away: inside range at arming
        _, _, fz = run(speed=2.5, until=30.0 - 2.0)
        self.assertTrue(fz.recorded)
        known = [tr for tr in fz.tracks if tr.known]
        self.assertEqual(len(known), 2)                                      # the two mates only
        d, _, _ = run(speed=2.5)
        self.assertEqual(d.reason, "fuze")

    def test_drone_short_on_the_threat_side(self):
        # Seen live after a manoeuvre: the drone flew to the new point from the threat's side and
        # stopped 3 m short of it, towards the threat.  A mate record timed on the threat's distance
        # to the *point* (12 m) put the threat 9 m from the drone, inside range: taken for a mate.
        for speed in (2.5, 3.5, 4.5):
            d, t, fz = run(speed=speed, own=(-3.0, 0.0, 20.0), arrive_late=0.0, lateral=0.5)
            self.assertEqual(d.reason, "fuze", speed)
            self.assertAlmostEqual(t, 30.0 - 3.0 / speed, delta=0.12)

    def test_mates_never_trigger(self):
        # no threat at all: the mates stay known, the window closes, hold
        d, t, _ = run(true_track=lambda t: (500.0, 0.0, 20.0))
        self.assertEqual((d.action, d.reason), ("hold", "fallback_hold"))
        self.assertGreater(t, 30.0 + 2.0)

    def test_timed_fallback(self):
        d, t, _ = run(cfg=FuzeConfig(fallback="timed"), true_track=lambda t: (500.0, 0.0, 20.0))
        self.assertEqual((d.action, d.reason), ("fire", "fallback_timed"))
        self.assertAlmostEqual(t, 32.0, delta=0.05)

    def test_threat_passing_outside_kill_radius_does_not_fire(self):
        d, _, _ = run(lateral=9.0)          # in range (10 m) but CPA 9 m > kill radius
        self.assertEqual(d.reason, "fallback_hold")

    def test_contact_off_the_predicted_track_is_ignored(self):
        # a new object passing 6 m from us (inside the kill radius) on a line 6 m from the threat's
        # predicted track, so never within the 5 m gate of the predicted position
        stray = lambda t: [(-3.0 * (t - 30.0), -6.0, 20.0)] if 20.0 < t < 40.0 else []
        d, t, _ = run(true_track=lambda t: (500.0, 0.0, 20.0), extra=stray)
        self.assertEqual(d.reason, "fallback_hold")

    def test_radius_mode_fires_on_entering_the_kill_radius(self):
        d, t, _ = run(cfg=FuzeConfig(fire="radius"), speed=2.5)
        self.assertEqual(d.reason, "fuze")
        self.assertLessEqual(d.range_m, 8.0)
        self.assertGreater(d.range_m, 7.0)

    def test_not_before_the_window(self):
        # threat arrives 4 s early (the window opens at t_engage - 2 s): it fires only once armed,
        # if it is still within the kill radius then, otherwise holds
        d, t, _ = run(arrive_late=-4.0)
        self.assertGreaterEqual(t, 28.0)

    def test_manoeuvre_updates_the_prediction(self):
        # the predictor is the job's latest track: with a stale one, the gate rejects the threat
        stale = lambda t: (2.5 * (t - 30.0), 12.0, 20.0)                     # 12 m off: wrong track
        d, _, _ = run(predict_track=stale)
        self.assertEqual(d.reason, "fallback_hold")

    def test_chain_ready(self):
        fz = Fuze(FuzeConfig())
        self.assertTrue(fz.chain_ready(29.0, 30.0, 2.0, 8.0))
        self.assertFalse(fz.chain_ready(27.0, 30.0, 2.0, 8.0))               # not armed yet
        self.assertFalse(fz.chain_ready(30.0, 30.0, 9.0, 8.0))               # off station
        fz.done = True
        self.assertFalse(fz.chain_ready(30.0, 30.0, 2.0, 8.0))

    def test_config_from_env(self):
        cfg = FuzeConfig.from_env({"FUZE_WINDOW_S": "3", "FUZE_FIRE": "radius", "FUZE_FALLBACK": "timed",
                                   "FUZE_RANGE_M": "12"})
        self.assertEqual((cfg.window_s, cfg.fire, cfg.fallback, cfg.range_m), (3.0, "radius", "timed", 12.0))
        with self.assertRaises(ValueError):
            FuzeConfig.from_env({"FUZE_FALLBACK": "maybe"})
        self.assertTrue(enabled({}))
        self.assertFalse(enabled({"FUZE": "0"}))
        self.assertIsInstance(Decision("hold", "fallback_hold"), Decision)


if __name__ == "__main__":
    unittest.main()
