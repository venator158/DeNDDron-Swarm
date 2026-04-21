import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

class PathPlanningStrategy(ABC):
    @abstractmethod
    def configure(self, config: Dict[str, Any]):
        pass

    @abstractmethod
    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        pass

    @abstractmethod
    def is_goal_reached(self) -> bool:
        pass

class APFStrategy(PathPlanningStrategy):
    def __init__(self):
        self.k_attractive = 2.0
        self.k_repulsive = 10.0
        self.influence_radius = 8.0
        self.a_decay = 0.5
        self.b_scale = 0.3

        self.alpha_stuck = 0.2
        self.min_movement = 0.001
        self.stuck_threshold = 10

        # Goal arrival
        self.goal_tolerance = 1.5       # metres — declare "arrived" within this radius
        self.braking_radius = 6.0       # metres — start smoothly braking inside this radius
        self._goal_reached = False

        # Global environment safety assumptions (matching simulator scene).
        self.ship_center = np.array([0.0, 0.0])
        self.ship_keepout_radius = 18.0
        self.ship_influence_radius = 30.0
        self.min_z = 1.0
        self.max_z = 50.0
        self.velocity_smoothing = 0.35

        self.pos_history = []
        self.stuck_count = 0
        self.current_k_att = self.k_attractive
        self.prev_v_total = np.zeros(3)

    def configure(self, config: Dict[str, Any]):
        self.k_attractive   = config.get("attractive_gain",   self.k_attractive)
        self.k_repulsive    = config.get("repulsive_gain",    self.k_repulsive)
        self.influence_radius = config.get("influence_radius", self.influence_radius)
        self.alpha_stuck    = config.get("stuck_growth_rate", self.alpha_stuck)
        self.goal_tolerance = config.get("goal_tolerance",    self.goal_tolerance)
        self.braking_radius = config.get("braking_radius",    self.braking_radius)
        self.ship_keepout_radius = config.get("ship_keepout_radius", self.ship_keepout_radius)
        self.ship_influence_radius = config.get("ship_influence_radius", self.ship_influence_radius)
        self.min_z = config.get("min_z", self.min_z)
        self.max_z = config.get("max_z", self.max_z)
        self.velocity_smoothing = config.get("velocity_smoothing", self.velocity_smoothing)
        self.current_k_att  = self.k_attractive
        self._goal_reached  = False   # reset on reconfigure
        self.prev_v_total = np.zeros(3)

    def is_goal_reached(self) -> bool:
        return self._goal_reached

    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        curr_pos = np.array([current_pose['x'], current_pose['y'], current_pose['z']])
        goal_pos = np.array([goal_pose['x'],    goal_pose['y'],    goal_pose['z']])

        to_goal      = goal_pos - curr_pos
        dist_to_goal = np.linalg.norm(to_goal)

        max_vel = 2.0

        if dist_to_goal <= self.goal_tolerance:
            self._goal_reached = True
            self.prev_v_total = np.zeros(3)
            return {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        self._goal_reached = False

        # ── 1. ATTRACTIVE VELOCITY ───────────────────────────────────────────
        v_goal = np.zeros(3)
        braking_scale = np.tanh(dist_to_goal / self.braking_radius)  # 0→1

        if dist_to_goal > 0.01:
            direction = to_goal / dist_to_goal
            # Make speed naturally collapse near goal to prevent overshoot.
            speed_xy = min(max_vel, max(0.15, dist_to_goal / self.braking_radius) * max_vel) * braking_scale
            # Move toward goal in XY plane
            v_goal[:2] = direction[:2] * speed_xy
            # Independent Z altitude control
            dz = goal_pos[2] - curr_pos[2]
            z_cmd = dz * 1.2 * braking_scale
            # Keep the drone above floor and below configured ceiling.
            if curr_pos[2] <= self.min_z + 0.2:
                z_cmd = max(0.0, z_cmd)
            if curr_pos[2] >= self.max_z - 0.2:
                z_cmd = min(0.0, z_cmd)
            v_goal[2] = np.clip(z_cmd, -max_vel, max_vel)

        # ── 2. REPULSIVE + TANGENTIAL ─────────────────────────────────────────
        v_rep = np.zeros(3)
        nearby_obstacles = voxel_map.get_nearby_obstacles(
            curr_pos[0], curr_pos[1], curr_pos[2], self.influence_radius)

        for obs in nearby_obstacles:
            obs_pos = np.array([obs['x'], obs['y'], obs['z']])
            to_obs  = curr_pos - obs_pos
            dist    = np.linalg.norm(to_obs)
            # Ignore ultra-near occupied voxels that are typically raytrace artifacts
            # around the drone itself.
            if dist < 1.2:
                continue
            if 0 < dist < self.influence_radius:
                # Bounded APF repulsion reduces spikes that cause zig-zag paths.
                strength = ((1.0 / max(dist, 0.2)) - (1.0 / self.influence_radius))
                strength = max(0.0, strength) / max(dist * dist, 0.04)
                strength = min(strength, 0.35)
                v_rep   += (to_obs / dist) * strength * self.k_repulsive

        # Dedicated repulsion from the central ship so paths cannot cut through it.
        ship_vec = curr_pos[:2] - self.ship_center
        ship_dist = np.linalg.norm(ship_vec)
        if ship_dist < self.ship_influence_radius:
            away = ship_vec / max(ship_dist, 1e-6)
            ship_strength = (self.ship_influence_radius - ship_dist) / max(self.ship_influence_radius - self.ship_keepout_radius, 1e-6)
            ship_strength = np.clip(ship_strength, 0.0, 2.0)
            v_rep[:2] += away * (self.k_repulsive * 2.0 * ship_strength)

        v_tangent = np.zeros(3)
        v_rep_xy_norm = np.linalg.norm(v_rep[:2])
        if v_rep_xy_norm > 0.15 and dist_to_goal > self.goal_tolerance * 2.0:
            perp = np.array([-v_rep[1], v_rep[0]])
            v_tangent[:2] = (perp / np.linalg.norm(perp)) * (v_rep_xy_norm * 0.12)
            if np.dot(v_tangent[:2], to_goal[:2]) < 0:
                v_tangent = -v_tangent

        # ── 3. BLEND + CAP ────────────────────────────────────────────────────
        v_total = v_goal + v_rep + v_tangent
        speed   = np.linalg.norm(v_total)
        if speed > max_vel:
            v_total = (v_total / speed) * max_vel

        # Smooth APF command to avoid twitching and erratic turns.
        alpha = float(np.clip(self.velocity_smoothing, 0.0, 1.0))
        v_total = (1.0 - alpha) * self.prev_v_total + alpha * v_total
        self.prev_v_total = v_total.copy()

        # Do not allow commanded XY velocity to keep moving inward when within keep-out.
        if ship_dist < self.ship_keepout_radius + 1.0:
            away = ship_vec / max(ship_dist, 1e-6)
            inward = np.dot(v_total[:2], -away)
            if inward > 0:
                v_total[:2] += away * inward

        return {
            "linear":  {"x": float(v_total[0]), "y": float(v_total[1]), "z": float(v_total[2])},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }