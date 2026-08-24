import unittest
import time
import math
import sys
import os

from unittest.mock import MagicMock
sys.modules['zenoh'] = MagicMock()

# Add src/metrics to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "metrics")))

from main import MetricsNode, AgentState, COLLISION_RADIUS_M, DEDUP_WINDOW_S


class MockZenohSession:
    def declare_subscriber(self, topic, callback):
        return None
    def declare_publisher(self, topic):
        return None


class TestMetricsSpatialHash(unittest.TestCase):

    def test_spatial_hash_equivalency_to_brute_force(self):
        session = MockZenohSession()
        node = MetricsNode(session)

        # Register 20 agents at known coordinates
        # Pairs within 2.5m: (0, 1), (5, 6)
        positions = {
            "drone_0": (0.0, 0.0, 10.0),
            "drone_1": (1.0, 1.0, 10.0),   # dist = sqrt(2) ~ 1.41m < 2.5m -> COLLISION
            "drone_2": (10.0, 0.0, 10.0),
            "drone_3": (30.0, 0.0, 10.0),
            "drone_4": (50.0, 50.0, 10.0),
            "drone_5": (100.0, 100.0, 10.0),
            "drone_6": (101.0, 100.0, 10.0), # dist = 1.0m < 2.5m -> COLLISION
        }

        # Populate agents
        for aid, (x, y, z) in positions.items():
            agent = AgentState(aid)
            agent.update_pose(x, y, z, 0.0, 0.0, 0.0)
            node._agents[aid] = agent

        # Run spatial hash collision check
        node.check_collisions_spatial_hash()

        # Verify collisions detected: (drone_0, drone_1) and (drone_5, drone_6)
        self.assertEqual(node._total_collisions, 2)
        collision_pairs = set(
            tuple(sorted(evt["agents"]))
            for evt in node._events
            if evt["type"] == "collision"
        )
        expected_pairs = {("drone_0", "drone_1"), ("drone_5", "drone_6")}
        self.assertEqual(collision_pairs, expected_pairs)

    def test_same_cell_multiple_agents(self):
        session = MockZenohSession()
        node = MetricsNode(session)

        # 3 agents in identical coordinates
        for i in range(3):
            aid = f"drone_{i}"
            agent = AgentState(aid)
            agent.update_pose(5.0, 5.0, 5.0, 0.0, 0.0, 0.0)
            node._agents[aid] = agent

        node.check_collisions_spatial_hash()

        # 3 agents at same position -> 3 pairs (0,1), (0,2), (1,2)
        self.assertEqual(node._total_collisions, 3)

    def test_deduplication_window(self):
        session = MockZenohSession()
        node = MetricsNode(session)

        agent0 = AgentState("drone_0")
        agent0.update_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        agent1 = AgentState("drone_1")
        agent1.update_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        node._agents["drone_0"] = agent0
        node._agents["drone_1"] = agent1

        # First check -> collision
        node.check_collisions_spatial_hash()
        self.assertEqual(node._total_collisions, 1)

        # Immediate second check -> deduplicated (no new collision event)
        node.check_collisions_spatial_hash()
        self.assertEqual(node._total_collisions, 1)

    def test_stale_or_dead_agent_handling(self):
        session = MockZenohSession()
        node = MetricsNode(session)

        agent0 = AgentState("drone_0")
        agent0.update_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        agent1 = AgentState("drone_1")
        agent1.update_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        agent1.alive = False  # Agent is dead/despawned

        node._agents["drone_0"] = agent0
        node._agents["drone_1"] = agent1

        node.check_collisions_spatial_hash()
        self.assertEqual(node._total_collisions, 0)


if __name__ == "__main__":
    unittest.main()
