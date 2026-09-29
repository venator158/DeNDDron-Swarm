"""
VoxelMap: 3D probabilistic occupancy grid for obstacle perception.
- World-absolute coordinates
- Sparse representation for memory efficiency
- Probabilistic occupancy: 0.0 (free) to 1.0 (occupied)

Per-frame cost is kept bounded for the 50 Hz control loop:
- occupied voxels are indexed separately, so obstacle queries scan only them
  instead of every cell of a cube around the drone;
- ray traversal is vectorized with numpy;
- stale-data expiry walks update batches oldest-first instead of every voxel.
"""

from collections import deque
import logging
import threading
import time

import numpy as np

logger = logging.getLogger("VoxelMap")


class VoxelMap:
    """
    Sparse 3D occupancy grid using dictionary-based voxel storage.

    Map bounds (world coordinates):
    - X: [-1000, 1000] meters
    - Y: [-1000, 1000] meters
    - Z: [0, 100] meters

    Resolution: 0.5 meters per voxel
    """

    # Map boundaries (world frame)
    MIN_X, MAX_X = -1000.0, 1000.0
    MIN_Y, MAX_Y = -1000.0, 1000.0
    MIN_Z, MAX_Z = 0.0, 100.0

    # Voxel resolution in meters
    RESOLUTION = 0.5

    OCCUPIED_THRESHOLD = 0.5

    def __init__(self):
        """Initialize empty voxel map."""
        # Sparse storage: key = (vx, vy, vz), value = occupancy probability [0.0, 1.0]
        self.voxels = {}
        # Last update time per voxel (None = never expires)
        self.voxel_timestamps = {}
        # Keys whose occupancy >= OCCUPIED_THRESHOLD
        self._occupied = set()
        # (timestamp, keys) per update, oldest first, for incremental expiry
        self._batches = deque()

        # Thread safety between Zenoh callback and Reflex loops
        self.lock = threading.RLock()

        self._max_index = np.array([
            int((self.MAX_X - self.MIN_X) / self.RESOLUTION) + 1,
            int((self.MAX_Y - self.MIN_Y) / self.RESOLUTION) + 1,
            int((self.MAX_Z - self.MIN_Z) / self.RESOLUTION) + 1,
        ])
        self._origin = np.array([self.MIN_X, self.MIN_Y, self.MIN_Z])

        logger.info(f"VoxelMap initialized. Bounds: X[{self.MIN_X}, {self.MAX_X}], "
                    f"Y[{self.MIN_Y}, {self.MAX_Y}], Z[{self.MIN_Z}, {self.MAX_Z}], "
                    f"Resolution: {self.RESOLUTION}m")

    def _world_to_voxel(self, x: float, y: float, z: float) -> tuple:
        """Convert world coordinates to voxel indices."""
        vx = int(np.round((x - self.MIN_X) / self.RESOLUTION))
        vy = int(np.round((y - self.MIN_Y) / self.RESOLUTION))
        vz = int(np.round((z - self.MIN_Z) / self.RESOLUTION))
        return (vx, vy, vz)

    def _voxel_to_world(self, vx: int, vy: int, vz: int) -> tuple:
        """Convert voxel indices back to world coordinates (voxel center)."""
        x = self.MIN_X + vx * self.RESOLUTION
        y = self.MIN_Y + vy * self.RESOLUTION
        z = self.MIN_Z + vz * self.RESOLUTION
        return (x, y, z)

    def _in_bounds(self, vx: int, vy: int, vz: int) -> bool:
        """Check if voxel indices are within map bounds."""
        return 0 <= vx < self._max_index[0] and 0 <= vy < self._max_index[1] and 0 <= vz < self._max_index[2]

    # ------------------------------------------------------------------
    # Writes (all callers hold self.lock)
    # ------------------------------------------------------------------
    def _set(self, key, occupancy: float, ts):
        self.voxels[key] = occupancy
        self.voxel_timestamps[key] = ts
        if occupancy >= self.OCCUPIED_THRESHOLD:
            self._occupied.add(key)
        else:
            self._occupied.discard(key)

    def _delete(self, key):
        self.voxels.pop(key, None)
        self.voxel_timestamps.pop(key, None)
        self._occupied.discard(key)

    def mark_occupied(self, x: float, y: float, z: float, confidence: float = 1.0, current_time: float = None):
        """
        Mark a world coordinate as occupied with given confidence.

        Args:
            x, y, z: World coordinates
            confidence: Occupancy probability [0.0, 1.0]. Default 1.0 for LiDAR detections.
        """
        key = self._world_to_voxel(x, y, z)
        if not self._in_bounds(*key):
            return
        ts = current_time if current_time is not None else time.time()
        with self.lock:
            # Take the max to accumulate evidence
            self._set(key, max(self.voxels.get(key, 0.0), confidence), ts)
            self._batches.append((ts, [key]))

    def mark_free(self, x: float, y: float, z: float, current_time: float = None):
        """
        Mark a world coordinate as free space.

        Args:
            x, y, z: World coordinates
        """
        key = self._world_to_voxel(x, y, z)
        if not self._in_bounds(*key):
            return
        ts = current_time if current_time is not None else time.time()
        with self.lock:
            # Only mark as free if not already occupied
            if self.voxels.get(key, 0.0) < self.OCCUPIED_THRESHOLD:
                self._set(key, 0.0, ts)
                self._batches.append((ts, [key]))

    # ------------------------------------------------------------------
    # Reads
    # ------------------------------------------------------------------
    def is_free(self, x: float, y: float, z: float, threshold: float = 0.5) -> bool:
        """
        Query if a world coordinate is free.

        Args:
            x, y, z: World coordinates
            threshold: Occupancy threshold. Voxel is free if occupancy < threshold.

        Returns:
            True if voxel is free (occupancy < threshold), False otherwise.
        """
        key = self._world_to_voxel(x, y, z)
        if not self._in_bounds(*key):
            return False  # Out of bounds is treated as occupied for safety
        return self.voxels.get(key, 0.0) < threshold

    def get_occupancy(self, x: float, y: float, z: float) -> float:
        """
        Get occupancy probability for a world coordinate.

        Args:
            x, y, z: World coordinates

        Returns:
            Occupancy probability [0.0, 1.0]. Unknown voxels default to 0.0 (free).
        """
        key = self._world_to_voxel(x, y, z)
        if not self._in_bounds(*key):
            return 1.0  # Out of bounds is occupied
        return self.voxels.get(key, 0.0)

    # ------------------------------------------------------------------
    # Ray tracing
    # ------------------------------------------------------------------
    def compute_ray_voxels(self, x_start: float, y_start: float, z_start: float,
                           x_end: float, y_end: float, z_end: float,
                           mark_endpoint_occupied: bool = True) -> tuple:
        """
        Calculates voxel coordinates for a ray traversal without acquiring any lock.
        Returns: (free_voxel_keys: list[tuple], occupied_voxel_keys: list[tuple])
        """
        v_start = np.array(self._world_to_voxel(x_start, y_start, z_start))
        v_end = np.array(self._world_to_voxel(x_end, y_end, z_end))
        diff = v_end - v_start
        steps = int(np.max(np.abs(diff))) + 1

        if steps <= 1:
            end = tuple(int(c) for c in v_end)
            if mark_endpoint_occupied and self._in_bounds(*end):
                return [], [end]
            return [], []

        t = np.arange(steps) / (steps - 1)
        cells = np.round(v_start + t[:, None] * diff).astype(np.int64)
        in_bounds = np.all((cells >= 0) & (cells < self._max_index), axis=1)

        free_cells = cells[:-1][in_bounds[:-1]]
        free_keys = list(map(tuple, free_cells.tolist()))
        occupied_keys = []
        if mark_endpoint_occupied and in_bounds[-1]:
            occupied_keys.append(tuple(cells[-1].tolist()))
        return free_keys, occupied_keys

    def update_batch(self, free_voxel_keys: list, occupied_voxel_keys: list,
                     confidence: float = 1.0, current_time: float = None):
        """
        Applies a batch of free and occupied voxel updates under a SINGLE lock acquisition.
        Occupied endpoints take precedence over intermediate free space for the same voxel key.
        """
        ts = current_time if current_time is not None else time.time()

        occ_set = set(occupied_voxel_keys)
        free_set = set(free_voxel_keys) - occ_set

        with self.lock:
            touched = []
            for key in free_set:
                if self.voxels.get(key, 0.0) < self.OCCUPIED_THRESHOLD:
                    self._set(key, 0.0, ts)
                    touched.append(key)
            for key in occ_set:
                self._set(key, max(self.voxels.get(key, 0.0), confidence), ts)
                touched.append(key)
            if touched:
                self._batches.append((ts, touched))

    def raytrace(self, x_start: float, y_start: float, z_start: float,
                 x_end: float, y_end: float, z_end: float,
                 current_time: float = None,
                 mark_endpoint_occupied: bool = True):
        """
        Simple raytrace using single-acquisition update_batch.
        """
        free_keys, occupied_keys = self.compute_ray_voxels(
            x_start, y_start, z_start, x_end, y_end, z_end, mark_endpoint_occupied
        )
        self.update_batch(free_keys, occupied_keys, confidence=1.0, current_time=current_time)

    def batch_raytrace(self, rays_data: list, current_time: float = None):
        """
        Computes all ray traversals for a LiDAR sweep lock-free, then applies all voxel
        updates under a SINGLE lock acquisition.
        rays_data: list of tuples: (x_start, y_start, z_start, x_end, y_end, z_end, mark_endpoint_occupied)
        """
        all_free = []
        all_occ = []

        for ray in rays_data:
            x_start, y_start, z_start, x_end, y_end, z_end, mark_occ = ray
            f_keys, o_keys = self.compute_ray_voxels(
                x_start, y_start, z_start, x_end, y_end, z_end, mark_endpoint_occupied=mark_occ
            )
            all_free.extend(f_keys)
            all_occ.extend(o_keys)

        self.update_batch(all_free, all_occ, confidence=1.0, current_time=current_time)

    # ------------------------------------------------------------------
    # Maintenance and queries
    # ------------------------------------------------------------------
    def cleanup_stale_data(self, max_age: float = 0.5, current_time: float = None):
        """
        Removes voxels that have not been updated for more than max_age seconds.
        This prevents moving obstacles from leaving trails and ensures the planners use fresh data.

        Walks update batches oldest-first and stops at the first fresh one, so the
        cost is proportional to what expires, not to the map size.  A batch stamped
        more than max_age in the *future* means the clock was reset, so it is expired too.
        """
        if current_time is None:
            current_time = time.time()

        with self.lock:
            while self._batches:
                ts, keys = self._batches[0]
                if ts is not None and abs(current_time - ts) <= max_age:
                    break
                self._batches.popleft()
                if ts is None:
                    continue
                for key in keys:
                    # Skip voxels refreshed by a later batch.
                    if self.voxel_timestamps.get(key) == ts:
                        self._delete(key)

    def get_nearby_obstacles(self, x: float, y: float, z: float, radius: float) -> list:
        """
        Finds all occupied voxels within a spherical radius of the given coordinate.
        Returns a list of their central world coordinates.
        """
        with self.lock:
            if not self._occupied:
                return []
            keys = np.fromiter(
                (c for key in self._occupied for c in key), dtype=np.int64, count=3 * len(self._occupied)
            ).reshape(-1, 3)
            occupancy = [self.voxels[tuple(k)] for k in keys.tolist()]

        centers = self._origin + keys * self.RESOLUTION
        dist = np.linalg.norm(centers - np.array([x, y, z]), axis=1)
        return [
            {"x": float(c[0]), "y": float(c[1]), "z": float(c[2]), "occupancy": occupancy[i]}
            for i, c in enumerate(centers.tolist()) if dist[i] <= radius
        ]

    def clear(self):
        """Clear all voxels."""
        with self.lock:
            self.voxels.clear()
            self.voxel_timestamps.clear()
            self._occupied.clear()
            self._batches.clear()

    def get_stats(self) -> dict:
        """Get map statistics."""
        with self.lock:
            if not self.voxels:
                return {"total_voxels": 0, "occupied": 0, "free": 0}
            occupied = len(self._occupied)
            return {
                "total_voxels": len(self.voxels),
                "occupied": occupied,
                "free": len(self.voxels) - occupied,
                "memory_mb": len(self.voxels) * 32 / (1024 * 1024)  # Rough estimate
            }
