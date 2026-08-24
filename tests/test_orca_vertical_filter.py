import unittest
import numpy as np
import sys
import os

# Add src/agent to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "agent")))

from path_planning import ORCAStrategy


class MockVoxelMap:
    def __init__(self, obstacles):
        self.obstacles = obstacles

    def get_nearby_obstacles(self, x, y, z, radius):
        return self.obstacles


class TestORCAVerticalFilter(unittest.TestCase):

    def setUp(self):
        self.orca = ORCAStrategy()
        self.agent_pos_2d = np.array([0.0, 0.0])
        self.agent_vel_2d = np.array([1.0, 0.0])
        self.agent_z = 10.0

    def test_same_altitude_obstacle_generates_constraint(self):
        vmap = MockVoxelMap([{"x": 3.0, "y": 0.0, "z": 10.0}])
        lines = self.orca._compute_obstacle_orca_lines(
            self.agent_pos_2d, self.agent_vel_2d, vmap, self.agent_z
        )
        self.assertEqual(len(lines), 1)

    def test_far_above_obstacle_skipped(self):
        vmap = MockVoxelMap([{"x": 3.0, "y": 0.0, "z": 25.0}])
        lines = self.orca._compute_obstacle_orca_lines(
            self.agent_pos_2d, self.agent_vel_2d, vmap, self.agent_z
        )
        self.assertEqual(len(lines), 0)

    def test_far_below_obstacle_skipped(self):
        vmap = MockVoxelMap([{"x": 3.0, "y": 0.0, "z": 0.0}])
        lines = self.orca._compute_obstacle_orca_lines(
            self.agent_pos_2d, self.agent_vel_2d, vmap, self.agent_z
        )
        self.assertEqual(len(lines), 0)

    def test_boundary_condition_obstacle_included(self):
        # max_v_dist = 0.5 + 0.25 + 1.0 = 1.75m
        # z = 11.7m -> diff = 1.7m <= 1.75m -> included
        vmap = MockVoxelMap([{"x": 3.0, "y": 0.0, "z": 11.7}])
        lines = self.orca._compute_obstacle_orca_lines(
            self.agent_pos_2d, self.agent_vel_2d, vmap, self.agent_z
        )
        self.assertEqual(len(lines), 1)

    def test_missing_or_nan_z_handled_safely(self):
        vmap = MockVoxelMap([
            {"x": 3.0, "y": 0.0, "z": float("nan")},
            {"x": 0.0, "y": 3.0}
        ])
        lines = self.orca._compute_obstacle_orca_lines(
            self.agent_pos_2d, self.agent_vel_2d, vmap, self.agent_z
        )
        self.assertEqual(len(lines), 2)


if __name__ == "__main__":
    unittest.main()
