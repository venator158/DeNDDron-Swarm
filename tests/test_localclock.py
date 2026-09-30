import unittest

from localclock import LocalClock, from_env, node_rng


class TestLocalClock(unittest.TestCase):
    def test_default_is_perfect(self):
        c = from_env("drone_1", env={})
        self.assertTrue(c.perfect)
        for t in (0.0, 12.34, 1e4):
            self.assertEqual(c.read(t), t)
            self.assertEqual(c.stamp(t), t)
            self.assertEqual(c.to_truth(t), t)

    def test_drift_and_offset(self):
        c = LocalClock(drift_ppm=500, offset_s=2.0)
        self.assertAlmostEqual(c.read(0.0), 2.0)
        self.assertAlmostEqual(c.read(100.0), 100.05 + 2.0)
        self.assertAlmostEqual(c.to_truth(c.read(73.2)), 73.2)
        # the error between two nodes grows linearly with truth
        a, b = LocalClock(drift_ppm=20), LocalClock(drift_ppm=-20)
        self.assertAlmostEqual(a.read(1000.0) - b.read(1000.0), 0.04)

    def test_jitter_only_in_stamps(self):
        c = LocalClock(jitter_s=0.01)
        self.assertEqual(c.read(5.0), 5.0)
        stamps = [c.stamp(5.0) - 5.0 for _ in range(2000)]
        self.assertTrue(any(s != 0.0 for s in stamps))
        sd = (sum(s * s for s in stamps) / len(stamps)) ** 0.5
        self.assertAlmostEqual(sd, 0.01, delta=0.002)

    def test_per_node_draw_is_reproducible(self):
        env = {"CLOCK_SEED": "7", "CLOCK_DRIFT_SPREAD_PPM": "20", "CLOCK_OFFSET_SPREAD_S": "0.5"}
        a1, a2, b = from_env("drone_1", env=env), from_env("drone_1", env=env), from_env("drone_2", env=env)
        self.assertEqual((a1.drift_ppm, a1.offset_s), (a2.drift_ppm, a2.offset_s))
        self.assertNotEqual((a1.drift_ppm, a1.offset_s), (b.drift_ppm, b.offset_s))
        other_seed = from_env("drone_1", env=dict(env, CLOCK_SEED="8"))
        self.assertNotEqual(a1.drift_ppm, other_seed.drift_ppm)
        for c in (a1, b):
            self.assertLessEqual(abs(c.drift_ppm), 20)
            self.assertLessEqual(abs(c.offset_s), 0.5)

    def test_fixed_values_and_prefix(self):
        env = {"CLOCK_DRIFT_PPM": "100", "SHIP_CLOCK_OFFSET_S": "1.5"}
        drone, ship = from_env("drone_3", env=env), from_env("ship", prefix="SHIP_CLOCK_", env=env)
        self.assertEqual((drone.drift_ppm, drone.offset_s), (100.0, 0.0))
        self.assertEqual((ship.drift_ppm, ship.offset_s), (0.0, 1.5))

    def test_node_rng_stable(self):
        self.assertEqual(node_rng(1, "x").random(), node_rng(1, "x").random())


if __name__ == "__main__":
    unittest.main()
