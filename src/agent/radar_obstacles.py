"""Planner obstacles from mmWave radar contacts (PERCEPTION=radar), replacing lidar + voxel map.

No zenoh dependency, so it is unit-tested.  The radar (drone/{id}/radar) reports unlabelled 3D
positions of nearby objects relative to the drone.  The planners were tuned on the planar lidar,
which saw every other drone as a circle of LIDAR_DRONE_RADIUS (3 m, inflated for separation) in the
horizontal plane, hit by up to 32 rays at the drone's own altitude, and placed one obstacle point
per hit.  This module builds exactly those points from the radar contacts: the same rays, the same
circles, the same altitude, so avoidance behaves as before, without ray-tracing every scan into a
voxel map (75-80% of an active drone's CPU).

Contacts on the ship (within ship_radius + margin of its centre) are dropped: the planners repel
from the ship on their own.
"""

import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


class RadarObstacles:
    def __init__(self, rays: int = 32, radius: float = 3.0, max_range: float = 50.0,
                 ship_radius: float = 16.0, ship_margin: float = 1.5, ttl_s: float = 0.5,
                 clock: Optional[Callable[[], float]] = None):
        self.rays, self.radius, self.max_range = rays, radius, max_range
        self.ship_clear = ship_radius + ship_margin
        self.ttl_s = ttl_s
        self.clock = clock
        self.points: List[Dict[str, float]] = []
        self.contacts = 0
        self._t = None

    def update(self, own: Vec3, contacts: Sequence[Vec3]) -> None:
        """A radar scan: contacts relative to us; own: our world position at the scan."""
        centres = []
        for rel in contacts:
            w = (own[0] + rel[0], own[1] + rel[1])
            if math.hypot(w[0], w[1]) <= self.ship_clear:
                continue                                       # the ship (handled by the planner)
            centres.append(w)
        self.contacts = len(centres)
        pts = []
        r2 = self.radius * self.radius
        for i in range(self.rays):
            a = 2.0 * math.pi * i / self.rays
            dx, dy = math.cos(a), math.sin(a)
            best = self.max_range
            for cx, cy in centres:
                # nearest positive t with |own + t d - c| = radius (2D), as the lidar did
                rx, ry = own[0] - cx, own[1] - cy
                b = dx * rx + dy * ry
                c = rx * rx + ry * ry - r2
                disc = b * b - c
                if disc < 0.0:
                    continue
                sq = math.sqrt(disc)
                t = -b - sq if -b - sq > 0.0 else -b + sq
                if 0.0 < t < best:
                    best = t
            if best < self.max_range:
                pts.append({"x": own[0] + best * dx, "y": own[1] + best * dy, "z": own[2]})
        self.points = pts
        self._t = self.clock() if self.clock else None

    def get_nearby_obstacles(self, x: float, y: float, z: float, radius: float) -> List[Dict[str, float]]:
        """Same interface as VoxelMap.get_nearby_obstacles."""
        if self.clock and self._t is not None and self.clock() - self._t > self.ttl_s:
            return []                                          # stale scan
        r2 = radius * radius
        return [p for p in self.points
                if (p["x"] - x) ** 2 + (p["y"] - y) ** 2 + (p["z"] - z) ** 2 <= r2]

    def get_stats(self) -> dict:
        return {"total_voxels": len(self.points), "contacts": self.contacts}
