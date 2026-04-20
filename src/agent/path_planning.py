import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

class PathPlanningStrategy(ABC):
    """
    Base Strategy Interface for Path Planning Algorithms.
    Allows hot-swapping algorithms (e.g., APF, A*, ORCA).
    """

    @abstractmethod
    def configure(self, config: Dict[str, Any]):
        """Algorithm-specific configuration and hyperparameter tuning."""
        pass

    @abstractmethod
    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        """
        Compute the next immediate velocity vector (Reflexes).
        Returns a dictionary for cmd_vel: {'linear': {'x': 0.0, ...}, 'angular': {...}}
        """
        pass

class APFStrategy(PathPlanningStrategy):
    """
    Artificial Potential Field (APF) implementation using local VoxelMap data.
    """
    def __init__(self):
        # Default config mimicking pp_test C++ values
        self.k_attractive = 2.0     # Attractive gain
        self.k_repulsive = 15.0     # Repulsive gain (tuned for 50m ship, 3m drones)
        self.influence_radius = 8.0 # Distance (m) to start feeling repulsion
        self.a_decay = 0.5          # Exponential decay rate (slower = force persists further)
        self.b_scale = 0.3          # Inverse square mapping (softened)
        
        # Stuck condition (Local Minima) escape mechanisms from pp_test
        self.alpha_stuck = 0.2      # Stuck growth rate for k_attractive
        self.min_movement = 0.001   # Threshold to detect stuck condition (m/frame)
        self.stuck_threshold = 10   # Frames before considering stuck
        
        # State tracking
        self.last_pos = None
        self.stuck_count = 0
        self.current_k_att = self.k_attractive

    def configure(self, config: Dict[str, Any]):
        self.k_attractive = config.get("attractive_gain", self.k_attractive)
        self.k_repulsive = config.get("repulsive_gain", self.k_repulsive)
        self.influence_radius = config.get("influence_radius", self.influence_radius)
        self.alpha_stuck = config.get("stuck_growth_rate", self.alpha_stuck)
        self.current_k_att = self.k_attractive

    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        curr_pos = np.array([current_pose['x'], current_pose['y'], current_pose['z']])
        goal_pos = np.array([goal_pose['x'], goal_pose['y'], goal_pose['z']])

        # --- Check point: Local Minima Escape (Stuck Detection) ---
        if self.last_pos is not None:
            movement = np.linalg.norm(curr_pos - self.last_pos)
            if movement < self.min_movement:
                self.stuck_count += 1
                if self.stuck_count >= self.stuck_threshold:
                    # Exponentially increase attractive force to escape local minima
                    capped_stuck_count = min(self.stuck_count, 50)  # Prevent np.exp overflow
                    self.current_k_att = np.exp(self.alpha_stuck * capped_stuck_count)
            else:
                self.stuck_count = 0
                self.current_k_att = self.k_attractive  # Reset to default
        
        self.last_pos = curr_pos

        # 1. Attractive Force
        to_goal = goal_pos - curr_pos
        dist_to_goal = np.linalg.norm(to_goal)
        f_att = np.zeros(3)

        if dist_to_goal > 0.1:
            f_att = self.current_k_att * (to_goal / dist_to_goal)  # Normalized vector

        # 2. Repulsive Force (Iterate over local occupied voxels)
        f_rep = np.zeros(3)
        nearby_obstacles = voxel_map.get_nearby_obstacles(curr_pos[0], curr_pos[1], curr_pos[2], self.influence_radius)

        for obs in nearby_obstacles:
            obs_pos = np.array([obs['x'], obs['y'], obs['z']])
            to_obs = obs_pos - curr_pos
            dist = np.linalg.norm(to_obs)

            if 0 < dist < self.influence_radius:
                # Force magnitude formula from pp_test: (1/(b*d²)) * e^(-a*d)
                magnitude = (1.0 / (self.b_scale * (dist ** 2))) * np.exp(-self.a_decay * dist)
                f_rep -= self.k_repulsive * magnitude * (to_obs / dist) # Push away

        f_total = f_att + f_rep

        # Limit max velocity
        max_vel = 2.0  # m/s — matches kinematic limit
        if np.linalg.norm(f_total) > max_vel:
            f_total = (f_total / np.linalg.norm(f_total)) * max_vel

        return {
            "linear": {"x": float(f_total[0]), "y": float(f_total[1]), "z": float(f_total[2])},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }
