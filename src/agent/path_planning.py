import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any

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
        # Core APF parameters (mapped from C++ implementation).
        self.k_attractive = 1.8
        self.k_repulsive = 8.5
        self.influence_radius = 7.0
        self.apf_exponential_decay = 0.5
        self.apf_inverse_square_scale = 0.3
        self.apf_stuck_growth_rate = 0.2

        # Motion/output parameters.
        self.step_size = 0.2
        self.max_velocity = 4.0
        self.velocity_smoothing = 0.42

        # Goal/arrival parameters.
        self.goal_tolerance = 1.5
        self.braking_radius = 7.5
        self._goal_reached = False

        # Stuck detection and force ramping.
        self.min_movement = 0.01
        self.stuck_threshold = 5
        self.stuck_count = 0
        self.current_k_att = self.k_attractive

        # Environment and safety constraints.
        self.ship_center = np.array([0.0, 0.0], dtype=float)
        self.ship_keepout_radius = 18.0
        self.ship_influence_radius = 30.0
        self.min_z = 1.0
        self.max_z = 50.0

        # Internal state.
        self.prev_v_total = np.zeros(3, dtype=float)
        self.last_pos = None

    def configure(self, config: Dict[str, Any]):
        required_keys = [
            "attractive_gain",
            "repulsive_gain",
            "influence_radius",
            "step_size",
            "max_velocity",
            "goal_tolerance",
            "braking_radius",
            "ship_keepout_radius",
            "ship_influence_radius",
            "apf_exponential_decay",
            "apf_inverse_square_scale",
            "apf_stuck_growth_rate",
            "min_movement",
            "stuck_threshold",
            "min_z",
            "max_z",
            "velocity_smoothing",
        ]
        missing = [k for k in required_keys if k not in config]
        if missing:
            raise ValueError(f"APF config missing required keys: {missing}")

        self.k_attractive = float(config["attractive_gain"])
        self.k_repulsive = float(config["repulsive_gain"])
        self.influence_radius = float(config["influence_radius"])
        self.step_size = float(config["step_size"])
        self.max_velocity = float(config["max_velocity"])
        self.goal_tolerance = float(config["goal_tolerance"])
        self.braking_radius = float(config["braking_radius"])
        self.ship_keepout_radius = float(config["ship_keepout_radius"])
        self.ship_influence_radius = float(config["ship_influence_radius"])
        self.apf_exponential_decay = float(config["apf_exponential_decay"])
        self.apf_inverse_square_scale = float(config["apf_inverse_square_scale"])
        self.apf_stuck_growth_rate = float(config["apf_stuck_growth_rate"])
        self.min_movement = float(config["min_movement"])
        self.stuck_threshold = int(config["stuck_threshold"])
        self.min_z = float(config["min_z"])
        self.max_z = float(config["max_z"])
        self.velocity_smoothing = float(config["velocity_smoothing"])
        self.current_k_att  = self.k_attractive
        self._goal_reached  = False   # reset on reconfigure
        self.prev_v_total = np.zeros(3)
        self.stuck_count = 0
        self.last_pos = None

    def is_goal_reached(self) -> bool:
        return self._goal_reached

    def _repulsive_force(self, curr_pos: np.ndarray, voxel_map) -> np.ndarray:
        v_rep = np.zeros(3, dtype=float)
        nearby_obstacles = voxel_map.get_nearby_obstacles(
            curr_pos[0], curr_pos[1], curr_pos[2], self.influence_radius
        )

        a = max(0.0, self.apf_exponential_decay)
        b = max(1e-3, self.apf_inverse_square_scale)

        for obs in nearby_obstacles:
            obs_pos = np.array([obs["x"], obs["y"], obs["z"]], dtype=float)
            away_vec = curr_pos - obs_pos
            dist = float(np.linalg.norm(away_vec))

            if dist <= 1e-3 or dist > self.influence_radius:
                continue

            # z = (1 / (b * d^2)) * exp(-a * d), away from obstacle.
            force_mag = (1.0 / (b * dist * dist)) * np.exp(-a * dist)
            v_rep += (away_vec / dist) * (self.k_repulsive * force_mag)

        # Add central ship repulsion with same APF formula.
        ship_vec_2d = curr_pos[:2] - self.ship_center
        ship_dist = float(np.linalg.norm(ship_vec_2d))
        if 1e-6 < ship_dist < self.ship_influence_radius:
            force_mag = (1.0 / (b * ship_dist * ship_dist)) * np.exp(-a * ship_dist)
            v_rep[:2] += (ship_vec_2d / ship_dist) * (self.k_repulsive * force_mag)

        return v_rep

    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        curr_pos = np.array([current_pose["x"], current_pose["y"], current_pose["z"]], dtype=float)
        goal_pos = np.array([goal_pose["x"], goal_pose["y"], goal_pose["z"]], dtype=float)

        to_goal = goal_pos - curr_pos
        dist_to_goal = float(np.linalg.norm(to_goal))
        max_vel = max(0.1, self.max_velocity)

        if self.last_pos is None:
            self.last_pos = curr_pos.copy()

        self._goal_reached = dist_to_goal <= self.goal_tolerance
        if self._goal_reached:
            self.stuck_count = 0
            self.current_k_att = self.k_attractive
            self.prev_v_total = np.zeros(3, dtype=float)
            self.last_pos = curr_pos.copy()
            return {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }

        # 1) Attractive force: k * normalize(goal - current)
        attractive = np.zeros(3, dtype=float)
        if dist_to_goal > 1e-6:
            attractive = self.current_k_att * (to_goal / dist_to_goal)

        # 2) Repulsive force from obstacles and ship
        repulsive = self._repulsive_force(curr_pos, voxel_map)

        # 3) Combine force vectors
        total_force = attractive + repulsive
        force_norm = float(np.linalg.norm(total_force))

        desired_v = np.zeros(3, dtype=float)
        if force_norm > 1e-9:
            move_dir = total_force / force_norm
            # Convert C++-style step direction to continuous velocity command.
            base_speed = min(max_vel, max(0.05, self.step_size / 0.02))
            # Soft braking near goal to avoid overshoot/orbit around tolerance.
            brake_scale = np.clip((dist_to_goal - self.goal_tolerance) / max(self.braking_radius, 1e-3), 0.0, 1.0)
            desired_v = move_dir * (base_speed * brake_scale)

        # Keep within vertical limits.
        if curr_pos[2] <= self.min_z + 0.2:
            desired_v[2] = max(0.0, desired_v[2])
        if curr_pos[2] >= self.max_z - 0.2:
            desired_v[2] = min(0.0, desired_v[2])

        speed = float(np.linalg.norm(desired_v))
        if speed > max_vel:
            desired_v = (desired_v / speed) * max_vel

        # Output smoothing.
        alpha = float(np.clip(self.velocity_smoothing, 0.0, 1.0))
        v_total = (1.0 - alpha) * self.prev_v_total + alpha * desired_v
        self.prev_v_total = v_total.copy()

        # Stuck detection and force ramping per provided logic.
        movement = float(np.linalg.norm(curr_pos - self.last_pos))
        if movement < self.min_movement:
            self.stuck_count += 1
            if self.stuck_count >= self.stuck_threshold:
                growth = np.exp(self.apf_stuck_growth_rate * (self.stuck_count - self.stuck_threshold))
                self.current_k_att = min(self.k_attractive * growth, self.k_attractive * 10.0)
        else:
            self.stuck_count = 0
            self.current_k_att = self.k_attractive
        self.last_pos = curr_pos.copy()

        return {
            "linear": {"x": float(v_total[0]), "y": float(v_total[1]), "z": float(v_total[2])},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }