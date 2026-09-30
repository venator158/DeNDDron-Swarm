"""Track: truth segment for the world, ship-time view for C2 (the radar measures in ship time)."""
import math
import unittest

from localclock import LocalClock

try:
    from ship import Track
except ImportError as e:        # ship.py needs zenoh (links.py)
    Track = None
    SKIP = str(e)


@unittest.skipIf(Track is None, "zenoh not installed")
class TestTrackTimebases(unittest.TestCase):
    def make(self, clock):
        return Track("T1", "uav", 1, (150.0, 0.0, 20.0), (-2.5, 0.5, 0.0), 30.0, 45.0, clock)

    def test_perfect_clock_is_identity(self):
        tr = self.make(LocalClock())
        self.assertEqual((tr.p0, tr.v, tr.t0), (tr.true_p0, tr.true_v, tr.true_t0))
        self.assertEqual(tr.track_msg(), tr.track_msg(truth=True))

    def test_ship_time_view_matches_truth_positions(self):
        clock = LocalClock(drift_ppm=500, offset_s=3.0)
        tr = self.make(clock)
        for t in (30.0, 41.7, 80.0):
            a, b = tr.position(clock.read(t)), tr.true_position(t)
            self.assertLess(math.dist(a, b), 1e-9)
        self.assertAlmostEqual(tr.t0, clock.read(30.0))
        # CPA and the engagement point are the same places, stamped in ship time
        self.assertAlmostEqual(clock.to_truth(tr.t_cpa), self.make(LocalClock()).t_cpa, places=6)
        self.assertEqual(tr.track_msg(truth=True)["t0"], 30.0)

    def test_true_closest_approach(self):
        tr = self.make(LocalClock())
        p = tr.true_position(50.0)
        t, d = tr.true_closest_approach((p[0], p[1], p[2] + 3.0))    # 3 m above the track
        self.assertAlmostEqual(t, 50.0, places=6)
        self.assertAlmostEqual(d, 3.0, places=6)


if __name__ == "__main__":
    unittest.main()
