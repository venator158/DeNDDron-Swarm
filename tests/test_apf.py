import unittest
import numpy as np
import sys
import os

# Add src/agent to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "agent")))

from path_planning import APFStrategy


class MockVoxelMap:
    def __init__(self, obstacles=None):
        self.obstacles = obstacles or []

    def get_nearby_obstacles(self, x, y, z, radius):
        return self.obstacles


class TestAPF(unittest.TestCase):

    def test_empty_obstacle_set(self):
        apf = APFStrategy()
        vmap = MockVoxelMap([])
        curr_pos = np.array([0.0, 0.0, 10.0])
        f_rep = apf._repulsive_force(curr_pos, vmap)
        np.testing.assert_array_almost_equal(f_rep, np.zeros(3))

    def test_force_magnitude_bounded_under_dense_cluster(self):
        apf = APFStrategy()
        apf.max_repulsive_force = 10.0

        # Dense obstacle cluster: 50 identical/close voxels at (0.5, 0.0, 10.0)
        dense_obs = [{"x": 0.5, "y": 0.0, "z": 10.0} for _ in range(50)]
        vmap_dense = MockVoxelMap(dense_obs)

        curr_pos = np.array([0.0, 0.0, 10.0])
        f_rep = apf._repulsive_force(curr_pos, vmap_dense)
        mag = np.linalg.norm(f_rep)

        self.assertLessEqual(mag, 10.0, f"Repulsive force magnitude {mag} exceeded cap 10.0")

    def test_soft_saturation_smoothness(self):
        apf = APFStrategy()
        apf.max_repulsive_force = 5.0

        vmap = MockVoxelMap([{"x": 0.1, "y": 0.0, "z": 10.0}])
        curr_pos = np.array([0.0, 0.0, 10.0])
        f_rep = apf._repulsive_force(curr_pos, vmap)
        mag = np.linalg.norm(f_rep)

        self.assertLessEqual(mag, 5.0, f"Soft saturated force {mag} must not exceed limit 5.0")

    def test_nan_inf_zero_distance_guards(self):
        apf = APFStrategy()
        # Coincident obstacle (distance 0.0)
        vmap = MockVoxelMap([{"x": 0.0, "y": 0.0, "z": 10.0}])
        curr_pos = np.array([0.0, 0.0, 10.0])

        f_rep = apf._repulsive_force(curr_pos, vmap)
        self.assertFalse(np.isnan(f_rep).any(), "Repulsive force contains NaN")
        self.assertFalse(np.isinf(f_rep).any(), "Repulsive force contains Inf")


if __name__ == "__main__":
    unittest.main()
