import unittest
import time
import threading
import sys
import os

# Add src/agent to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src", "agent")))

from voxel_map import VoxelMap


class TestVoxelMapBatch(unittest.TestCase):

    def test_batch_update_correctness_and_precedence(self):
        vmap = VoxelMap()

        # Define origin and endpoints for 3 rays
        # Ray 1: hits an obstacle at (5.0, 0.0, 0.0)
        # Ray 2: passes through (5.0, 0.0, 0.0) as free space, hits at (10.0, 0.0, 0.0)
        rays_data = [
            (0.0, 0.0, 0.0, 5.0, 0.0, 0.0, True),    # endpoint at x=5 is OCCUPIED
            (0.0, 0.0, 0.0, 10.0, 0.0, 0.0, False),  # passes through x=5 as FREE
        ]

        # In batch_raytrace, endpoint occupancy must take precedence over free space for the same voxel!
        vmap.batch_raytrace(rays_data, current_time=100.0)

        # Check occupancy at endpoint x=5.0 (should be occupied >= 0.5)
        self.assertGreaterEqual(vmap.get_occupancy(5.0, 0.0, 0.0), 0.5)

        # Check intermediate x=2.0 (should be free < 0.5)
        self.assertLess(vmap.get_occupancy(2.0, 0.0, 0.0), 0.5)

    def test_stale_data_cleanup(self):
        vmap = VoxelMap()
        vmap.mark_occupied(1.0, 1.0, 1.0, confidence=1.0, current_time=10.0)
        vmap.mark_occupied(2.0, 2.0, 2.0, confidence=1.0, current_time=10.4)

        # Clean up at current_time = 10.6 with max_age = 0.5
        # 10.6 - 10.0 = 0.6 > 0.5 -> key (1.0, 1.0, 1.0) deleted
        # 10.6 - 10.4 = 0.2 <= 0.5 -> key (2.0, 2.0, 2.0) kept
        vmap.cleanup_stale_data(max_age=0.5, current_time=10.6)

        self.assertLess(vmap.get_occupancy(1.0, 1.0, 1.0), 0.5)
        self.assertGreaterEqual(vmap.get_occupancy(2.0, 2.0, 2.0), 0.5)

    def test_concurrent_reader_writer(self):
        vmap = VoxelMap()
        running = True
        errors = []

        def writer():
            nonlocal running
            t = 0.0
            while running:
                t += 0.01
                rays = [
                    (0.0, 0.0, 0.0, 10.0 * np.cos(t), 10.0 * np.sin(t), 5.0, True)
                    for _ in range(16)
                ]
                vmap.batch_raytrace(rays, current_time=t)
                time.sleep(0.001)

        def reader():
            nonlocal running
            while running:
                try:
                    obs = vmap.get_nearby_obstacles(0.0, 0.0, 0.0, radius=12.0)
                    stats = vmap.get_stats()
                except Exception as e:
                    errors.append(e)
                time.sleep(0.001)

        import numpy as np
        t_write = threading.Thread(target=writer)
        t_read = threading.Thread(target=reader)

        t_write.start()
        t_read.start()

        time.sleep(0.2)
        running = False

        t_write.join()
        t_read.join()

        self.assertEqual(len(errors), 0, f"Concurrent access errors: {errors}")

    def test_benchmark_report_locking_time(self):
        """Report measurements comparing individual vs batched raytrace processing."""
        import numpy as np

        vmap_single = VoxelMap()
        vmap_batch = VoxelMap()

        # Build 100 sweeps of 16 rays
        sweeps = []
        for s in range(100):
            rays = []
            for r in range(16):
                angle = r * (2 * np.pi / 16)
                rays.append((0.0, 0.0, 5.0, 20.0 * np.cos(angle), 20.0 * np.sin(angle), 5.0, True))
            sweeps.append(rays)

        # Measure single raytrace (per-ray update)
        t0 = time.perf_counter()
        for sweep in sweeps:
            for ray in sweep:
                vmap_single.raytrace(ray[0], ray[1], ray[2], ray[3], ray[4], ray[5], mark_endpoint_occupied=ray[6])
        t_single = time.perf_counter() - t0

        # Measure batch raytrace (per-sweep update)
        t0 = time.perf_counter()
        for sweep in sweeps:
            vmap_batch.batch_raytrace(sweep)
        t_batch = time.perf_counter() - t0

        print(f"\n[BENCHMARK REPORT] 100 LiDAR sweeps (1600 rays):")
        print(f"  Per-ray raytrace time:   {t_single*1000.0:.2f} ms")
        print(f"  Batch raytrace time:     {t_batch*1000.0:.2f} ms")
        print(f"  Speedup ratio:           {t_single / max(t_batch, 1e-9):.2f}x")

    def test_nearby_obstacles_matches_brute_force(self):
        import numpy as np
        rng = np.random.default_rng(3)
        vmap = VoxelMap()
        for _ in range(400):
            x, y, z = rng.uniform(-20, 20), rng.uniform(-20, 20), rng.uniform(5, 30)
            if rng.random() < 0.5:
                vmap.mark_occupied(x, y, z, current_time=1.0)
            else:
                vmap.mark_free(x, y, z, current_time=1.0)
        for _ in range(20):
            q = rng.uniform(-15, 15, 3) + np.array([0, 0, 17])
            r = rng.uniform(1, 15)
            got = sorted((o["x"], o["y"], o["z"]) for o in vmap.get_nearby_obstacles(*q, r))
            want = sorted(
                vmap._voxel_to_world(*k) for k, occ in vmap.voxels.items()
                if occ >= 0.5 and np.linalg.norm(np.array(vmap._voxel_to_world(*k)) - q) <= r
            )
            self.assertEqual(got, want)

    def test_refreshed_voxel_survives_cleanup_of_older_batch(self):
        vmap = VoxelMap()
        vmap.mark_occupied(1.0, 1.0, 1.0, current_time=10.0)
        vmap.mark_occupied(1.0, 1.0, 1.0, current_time=10.4)   # same voxel, refreshed
        vmap.cleanup_stale_data(max_age=0.5, current_time=10.6)
        self.assertGreaterEqual(vmap.get_occupancy(1.0, 1.0, 1.0), 0.5)
        vmap.cleanup_stale_data(max_age=0.5, current_time=11.0)
        self.assertLess(vmap.get_occupancy(1.0, 1.0, 1.0), 0.5)
        self.assertEqual(vmap.get_stats()["total_voxels"], 0)

    def test_clock_reset_expires_future_stamped_voxels(self):
        vmap = VoxelMap()
        vmap.mark_occupied(1.0, 1.0, 1.0, current_time=500.0)  # before a simulator reset
        vmap.mark_occupied(3.0, 3.0, 3.0, current_time=0.1)    # after it
        vmap.cleanup_stale_data(max_age=0.5, current_time=0.2)
        self.assertLess(vmap.get_occupancy(1.0, 1.0, 1.0), 0.5)
        self.assertGreaterEqual(vmap.get_occupancy(3.0, 3.0, 3.0), 0.5)
        self.assertEqual(len(vmap.get_nearby_obstacles(3.0, 3.0, 3.0, 1.0)), 1)


if __name__ == "__main__":
    unittest.main()
