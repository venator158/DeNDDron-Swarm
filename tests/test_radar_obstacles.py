"""Radar contacts -> lidar-equivalent planner obstacles (src/agent/radar_obstacles.py)."""
import math
import random
import unittest

from radar_obstacles import RadarObstacles


def lidar(own, others, rays=32, radius=3.0, max_range=50.0):
    """Python port of GazeboSimulator::simulate_lidar for drones (planar, inflated circles)."""
    hits = []
    for i in range(rays):
        a = 2 * math.pi * i / rays
        dx, dy = math.cos(a), math.sin(a)
        best = max_range
        for o in others:
            rx, ry = own[0] - o[0], own[1] - o[1]
            b = 2 * (dx * rx + dy * ry)
            c = rx * rx + ry * ry - radius * radius
            disc = b * b - 4 * c
            if disc < 0:
                continue
            t1, t2 = (-b - math.sqrt(disc)) / 2, (-b + math.sqrt(disc)) / 2
            t = t1 if t1 > 0 else (t2 if t2 > 0 else -1)
            if 0 < t < best:
                best = t
        if best < max_range:
            hits.append((own[0] + best * dx, own[1] + best * dy, own[2]))
    return hits


class TestRadarObstacles(unittest.TestCase):
    def test_matches_the_lidar(self):
        rng = random.Random(3)
        for _ in range(50):
            own = (rng.uniform(40, 80), rng.uniform(-30, 30), 20.0)
            others = [(own[0] + rng.uniform(-25, 25), own[1] + rng.uniform(-25, 25), 20 + rng.uniform(-6, 6))
                      for _ in range(rng.randint(0, 5))]
            ro = RadarObstacles()
            ro.update(own, [(o[0] - own[0], o[1] - own[1], o[2] - own[2]) for o in others])
            got = sorted((round(p["x"], 6), round(p["y"], 6)) for p in ro.points)
            want = sorted((round(h[0], 6), round(h[1], 6)) for h in lidar(own, others))
            self.assertEqual(got, want)
            self.assertTrue(all(p["z"] == own[2] for p in ro.points))

    def test_ship_contacts_dropped(self):
        ro = RadarObstacles()
        ro.update((30.0, 0.0, 20.0), [(-14.5, 0.0, -10.0)])    # a hull point 15.5 m from the centre
        self.assertEqual(ro.points, [])
        self.assertEqual(ro.contacts, 0)

    def test_query_radius_and_staleness(self):
        t = [0.0]
        ro = RadarObstacles(clock=lambda: t[0], ttl_s=0.5)
        ro.update((50.0, 0.0, 20.0), [(10.0, 0.0, 0.0)])
        self.assertTrue(ro.get_nearby_obstacles(50.0, 0.0, 20.0, 8.0))       # nearest hit 7 m away
        self.assertFalse(ro.get_nearby_obstacles(50.0, 0.0, 20.0, 6.0))
        t[0] = 0.6
        self.assertFalse(ro.get_nearby_obstacles(50.0, 0.0, 20.0, 8.0))      # stale


if __name__ == "__main__":
    unittest.main()
