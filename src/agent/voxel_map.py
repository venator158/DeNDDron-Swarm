"""
VoxelMap: 3D probabilistic occupancy grid for obstacle perception.
- World-absolute coordinates
- Sparse representation for memory efficiency
- Probabilistic occupancy: 0.0 (free) to 1.0 (occupied)
"""

import numpy as np
import logging
import time
import threading

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

    def __init__(self):
        """Initialize empty voxel map."""
        # Sparse storage: key = (vx, vy, vz), value = occupancy probability [0.0, 1.0]
        self.voxels = {}

        # For fast spatial queries, maintain a timestamp per voxel for future decay
        self.voxel_timestamps = {}
        
        # Thread safety between Zenoh callback and Reflex loops
        self.lock = threading.RLock()

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
        max_vx = int((self.MAX_X - self.MIN_X) / self.RESOLUTION) + 1
        max_vy = int((self.MAX_Y - self.MIN_Y) / self.RESOLUTION) + 1
        max_vz = int((self.MAX_Z - self.MIN_Z) / self.RESOLUTION) + 1
        return 0 <= vx < max_vx and 0 <= vy < max_vy and 0 <= vz < max_vz

    def mark_occupied(self, x: float, y: float, z: float, confidence: float = 1.0, current_time: float = None):
        """
        Mark a world coordinate as occupied with given confidence.

        Args:
            x, y, z: World coordinates
            confidence: Occupancy probability [0.0, 1.0]. Default 1.0 for LiDAR detections.
        """
        vx, vy, vz = self._world_to_voxel(x, y, z)
        if not self._in_bounds(vx, vy, vz):
            return

        key = (vx, vy, vz)
        with self.lock:
            # Update with new confidence (taking max to accumulate evidence)
            current = self.voxels.get(key, 0.0)
            self.voxels[key] = max(current, confidence)
            self.voxel_timestamps[key] = current_time if current_time else time.time()  # Store actual timestamp

    def mark_free(self, x: float, y: float, z: float, current_time: float = None):
        """
        Mark a world coordinate as free space.

        Args:
            x, y, z: World coordinates
        """
        vx, vy, vz = self._world_to_voxel(x, y, z)
        if not self._in_bounds(vx, vy, vz):
            return

        key = (vx, vy, vz)
        with self.lock:
            # Only mark as free if not already occupied
            if key not in self.voxels or self.voxels[key] < 0.5:
                self.voxels[key] = 0.0
                self.voxel_timestamps[key] = current_time if current_time else time.time()

    def is_free(self, x: float, y: float, z: float, threshold: float = 0.5) -> bool:
        """
        Query if a world coordinate is free.

        Args:
            x, y, z: World coordinates
            threshold: Occupancy threshold. Voxel is free if occupancy < threshold.

        Returns:
            True if voxel is free (occupancy < threshold), False otherwise.
        """
        vx, vy, vz = self._world_to_voxel(x, y, z)
        if not self._in_bounds(vx, vy, vz):
            return False  # Out of bounds is treated as occupied for safety

        key = (vx, vy, vz)
        occupancy = self.voxels.get(key, 0.0)
        return occupancy < threshold

    def get_occupancy(self, x: float, y: float, z: float) -> float:
        """
        Get occupancy probability for a world coordinate.

        Args:
            x, y, z: World coordinates

        Returns:
            Occupancy probability [0.0, 1.0]. Unknown voxels default to 0.0 (free).
        """
        vx, vy, vz = self._world_to_voxel(x, y, z)
        if not self._in_bounds(vx, vy, vz):
            return 1.0  # Out of bounds is occupied

        key = (vx, vy, vz)
        return self.voxels.get(key, 0.0)

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

        free_keys = []
        occupied_keys = []

        if steps <= 1:
            if mark_endpoint_occupied:
                vx, vy, vz = tuple(v_end)
                if self._in_bounds(vx, vy, vz):
                    occupied_keys.append((vx, vy, vz))
            return free_keys, occupied_keys

        for i in range(steps):
            t = i / (steps - 1)
            v_curr = v_start + t * diff
            vx, vy, vz = tuple(int(np.round(c)) for c in v_curr)

            if not self._in_bounds(vx, vy, vz):
                continue

            if i == steps - 1:
                if mark_endpoint_occupied:
                    occupied_keys.append((vx, vy, vz))
            else:
                free_keys.append((vx, vy, vz))

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
            for key in free_set:
                if key not in self.voxels or self.voxels[key] < 0.5:
                    self.voxels[key] = 0.0
                    self.voxel_timestamps[key] = ts

            for key in occ_set:
                current = self.voxels.get(key, 0.0)
                self.voxels[key] = max(current, confidence)
                self.voxel_timestamps[key] = ts

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

    def cleanup_stale_data(self, max_age: float = 0.5, current_time: float = None):
        """
        Removes or decays voxels that have not been updated recently.
        This prevents moving obstacles from leaving trails and ensures APF uses fresh data.
        
        Args:
            max_age: Maximum age in seconds before a voxel is cleared.
        """
        if current_time is None:
            current_time = time.time()
        stale_keys = []
        
        with self.lock:
            for key, ts in list(self.voxel_timestamps.items()):
                if ts is None:
                    continue
                if current_time - ts > max_age:
                    stale_keys.append(key)
                    
            for key in stale_keys:
                if key in self.voxels:
                    del self.voxels[key]
                if key in self.voxel_timestamps:
                    del self.voxel_timestamps[key]

    def get_nearby_obstacles(self, x: float, y: float, z: float, radius: float) -> list:
        """
        Finds all occupied voxels within a spherical radius of the given coordinate.
        Returns a list of their central world coordinates.
        """
        obstacles = []
        cx, cy, cz = self._world_to_voxel(x, y, z)
        v_radius = int(np.ceil(radius / self.RESOLUTION))
        
        # Fast local bounding box check over the voxel sparse grid
        with self.lock:
            for vx in range(cx - v_radius, cx + v_radius + 1):
                for vy in range(cy - v_radius, cy + v_radius + 1):
                    for vz in range(cz - v_radius, cz + v_radius + 1):
                        key = (vx, vy, vz)
                        occupancy = self.voxels.get(key, 0.0)
                        if occupancy >= 0.5:
                            wx, wy, wz = self._voxel_to_world(vx, vy, vz)
                            # Confirm spherical bounds (Euclidean distance)
                            dist = np.sqrt((wx - x)**2 + (wy - y)**2 + (wz - z)**2)
                            if dist <= radius:
                                obstacles.append({"x": wx, "y": wy, "z": wz, "occupancy": occupancy})
                            
        return obstacles

    def clear(self):
        """Clear all voxels."""
        with self.lock:
            self.voxels.clear()
            self.voxel_timestamps.clear()

    def get_stats(self) -> dict:
        """Get map statistics."""
        with self.lock:
            if not self.voxels:
                return {"total_voxels": 0, "occupied": 0, "free": 0}

            occupied = sum(1 for v in self.voxels.values() if v >= 0.5)
            free = sum(1 for v in self.voxels.values() if v < 0.5)

            return {
                "total_voxels": len(self.voxels),
                "occupied": occupied,
                "free": free,
                "memory_mb": len(self.voxels) * 32 / (1024 * 1024)  # Rough estimate
            }

    def export_to_dict(self) -> dict:
        """Export voxel map to dictionary for serialization."""
        with self.lock:
            # Convert tuple keys to strings for JSON serialization
            return {str(k): v for k, v in self.voxels.items()}
        return {
            "voxels": voxels_serializable,
            "bounds": {
                "x": [self.MIN_X, self.MAX_X],
                "y": [self.MIN_Y, self.MAX_Y],
                "z": [self.MIN_Z, self.MAX_Z]
            },
            "resolution": self.RESOLUTION
        }

    def import_from_dict(self, data: dict):
        """Import voxel map from dictionary."""
        if "voxels" in data:
            self.voxels = data["voxels"]
            self.voxel_timestamps = {k: None for k in self.voxels.keys()}

    def get_occupied_voxels(self, threshold: float = 0.5) -> list:
        """Get list of occupied voxel centers in world coordinates."""
        occupied = []
        for (vx, vy, vz), occupancy in self.voxels.items():
            if occupancy >= threshold:
                x, y, z = self._voxel_to_world(vx, vy, vz)
                occupied.append((x, y, z, occupancy))
        return occupied

    def get_free_voxels(self, threshold: float = 0.5) -> list:
        """Get list of free voxel centers in world coordinates."""
        free = []
        for (vx, vy, vz), occupancy in self.voxels.items():
            if occupancy < threshold:
                x, y, z = self._voxel_to_world(vx, vy, vz)
                free.append((x, y, z, occupancy))
        return free

    def get_voxel_data(self) -> dict:
        """Get all voxel data for visualization."""
        occupied = self.get_occupied_voxels()
        free = self.get_free_voxels()
        return {
            "occupied": occupied,
            "free": free,
            "stats": self.get_stats()
        }
