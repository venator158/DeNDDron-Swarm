import math
import unittest

from threats import (DEFAULT_THREAT_TYPES, aim_velocity, Threat, closest_point_of_approach, engagement_point, eta,
                     parse_threat_types, position_at, priority_order, slot_point, time_to_intercept)

LOC = {"x": 0.0, "y": 30.0, "z": 20.0}


class TestGeometry(unittest.TestCase):
    def test_aim_velocity_passes_through_aim_point_beside_ship(self):
        p = (120.0, -90.0, 20.0)
        v = aim_velocity(p, 12.0, 25.0)
        self.assertAlmostEqual(math.hypot(v[0], v[1]), 12.0)
        self.assertEqual(v[2], 0.0)
        # Aim point: 25 m from the ship, perpendicular to the initial line of sight.
        b = math.atan2(p[1], p[0])
        aim = (-math.sin(b) * 25.0, math.cos(b) * 25.0)
        t = math.hypot(aim[0] - p[0], aim[1] - p[1]) / 12.0
        q = position_at(p, v, 0.0, t)
        self.assertAlmostEqual(q[0], aim[0], places=6)
        self.assertAlmostEqual(q[1], aim[1], places=6)
        cpa, _ = closest_point_of_approach(p, v, 0.0)
        self.assertLessEqual(math.hypot(cpa[0], cpa[1]), 25.0)

    def test_cpa_of_crossing_track(self):
        # Flies along y = 30 from x = -100 at 5 m/s: CPA is (0, 30) after 20 s.
        cpa, t = closest_point_of_approach((-100.0, 30.0, 20.0), (5.0, 0.0, 0.0), t0=10.0)
        self.assertAlmostEqual(t, 30.0)
        self.assertAlmostEqual(cpa[0], 0.0)
        self.assertAlmostEqual(cpa[1], 30.0)

    def test_cpa_of_receding_track_is_now(self):
        cpa, t = closest_point_of_approach((50.0, 0.0, 10.0), (3.0, 0.0, 0.0), t0=5.0)
        self.assertEqual(t, 5.0)
        self.assertEqual(cpa, (50.0, 0.0, 10.0))

    def test_engagement_at_cpa_when_outside_defended_radius(self):
        p, t = engagement_point((-100.0, 60.0, 20.0), (5.0, 0.0, 0.0), 0.0, defended_radius=45.0)
        self.assertAlmostEqual(t, 20.0)
        self.assertAlmostEqual(math.hypot(p[0], p[1]), 60.0)

    def test_engagement_at_defended_radius_for_inbound_threat(self):
        # Aimed straight at the ship: CPA is the ship itself, so engage at 45 m.
        p, t = engagement_point((-150.0, 0.0, 15.0), (3.0, 0.0, 0.0), 0.0, defended_radius=45.0)
        self.assertAlmostEqual(math.hypot(p[0], p[1]), 45.0)
        self.assertAlmostEqual(t, 35.0)
        self.assertAlmostEqual(p[2], 15.0)

    def test_engagement_point_lies_on_track(self):
        p0, v = (120.0, -90.0, 18.0), (-3.0, 2.0, 0.0)
        p, t = engagement_point(p0, v, 2.0, defended_radius=45.0)
        on_track = position_at(p0, v, 2.0, t)
        self.assertTrue(all(abs(a - b) < 1e-9 for a, b in zip(p, on_track)))

    def test_already_inside_defended_radius_engages_now(self):
        p, t = engagement_point((10.0, 0.0, 15.0), (-1.0, 0.0, 0.0), 7.0, defended_radius=45.0)
        self.assertEqual(t, 7.0)

    def test_slots_stacked_through_point(self):
        pts = [slot_point((5.0, 6.0, 20.0), k, 3, 3.0) for k in range(3)]
        self.assertEqual(pts, [(5.0, 6.0, 17.0), (5.0, 6.0, 20.0), (5.0, 6.0, 23.0)])
        self.assertEqual([slot_point((5.0, 6.0, 20.0), k, 2, 3.0)[2] for k in range(2)], [17.0, 23.0])
        self.assertEqual(slot_point((1.0, 2.0, 3.0), 0, 1, 3.0), (1.0, 2.0, 3.0))


class TestEta(unittest.TestCase):
    def test_trapezoid_and_triangle_profiles(self):
        self.assertAlmostEqual(eta(40.0, 4.0, 1.0), 40.0 / 4.0 + 4.0)   # reaches v_max
        self.assertAlmostEqual(eta(4.0, 4.0, 1.0), 4.0)                  # 2*sqrt(4/1)
        self.assertEqual(eta(0.0, 4.0, 1.0), 0.0)

    def test_eta_continuous_at_profile_switch(self):
        d = 4.0 * 4.0 / 1.0
        self.assertAlmostEqual(eta(d - 1e-9, 4.0, 1.0), eta(d, 4.0, 1.0), places=4)

    def test_time_to_intercept_is_level_th_fastest(self):
        self.assertEqual(time_to_intercept([9.0, 3.0, 5.0, 7.0], 2), 5.0)
        self.assertEqual(time_to_intercept([9.0, 3.0], 1), 3.0)
        self.assertIsNone(time_to_intercept([9.0, 3.0], 3))


class TestThreatModel(unittest.TestCase):
    def test_priority_order(self):
        ts = [Threat("b", "t", 1, 1, LOC), Threat("a", "t", 1, 1, LOC), Threat("z", "t", 3, 3, LOC)]
        self.assertEqual([t.threat_id for t in priority_order(ts)], ["z", "a", "b"])

    def test_dict_roundtrip(self):
        t = Threat("T1", "missile", 2, 1, LOC, t_engage=42.5)
        self.assertEqual(Threat.from_dict(t.to_dict()), t)

    def test_required_defaults_to_level(self):
        d = Threat("T1", "t", 3, 3, LOC).to_dict()
        del d["required"]
        self.assertEqual(Threat.from_dict(d).required, 3)

    def test_parse_threat_types(self):
        types = parse_threat_types("uav:1:2.5:0.7, missile:2:3.5:0.3")
        self.assertEqual(types["uav"].level, 1)
        self.assertEqual(types["missile"].speed, 3.5)
        self.assertEqual(types["uav"].min_z, DEFAULT_THREAT_TYPES["uav"].min_z)
        with self.assertRaises(ValueError):
            parse_threat_types("uav:0:1:1")


if __name__ == "__main__":
    unittest.main()
