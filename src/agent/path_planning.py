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

        self.max_repulsive_force = 10.0
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
        self.max_repulsive_force = float(config.get("max_repulsive_force", 10.0))
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

            force_mag = (1.0 / (b * dist * dist)) * np.exp(-a * dist)
            v_rep += (away_vec / dist) * (self.k_repulsive * force_mag)

        # Add central ship repulsion with same APF formula.
        ship_vec_2d = curr_pos[:2] - self.ship_center
        ship_dist = float(np.linalg.norm(ship_vec_2d))
        if 1e-6 < ship_dist < self.ship_influence_radius:
            force_mag = (1.0 / (b * ship_dist * ship_dist)) * np.exp(-a * ship_dist)
            v_rep[:2] += (ship_vec_2d / ship_dist) * (self.k_repulsive * force_mag)

        # Numerical guards against NaN / Inf
        v_rep = np.nan_to_num(v_rep, nan=0.0, posinf=0.0, neginf=0.0)

        # Soft saturation: limit max repulsive force magnitude smoothly
        raw_mag = float(np.linalg.norm(v_rep))
        if raw_mag > 1e-9 and self.max_repulsive_force > 0:
            bounded_mag = self.max_repulsive_force * np.tanh(raw_mag / self.max_repulsive_force)
            v_rep = (v_rep / raw_mag) * bounded_mag

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

        import sys
        if np.random.rand() < 0.05:  # Print 5% of the time to avoid log spam
            print(f"APF Debug: curr={curr_pos}, goal={goal_pos}, to_goal={to_goal}, dist={dist_to_goal:.2f}", file=sys.stderr)
            print(f"APF Debug: attractive={attractive}, repulsive={repulsive}, total={total_force}", file=sys.stderr)

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


# ═══════════════════════════════════════════════════════════════════════════════
# ORCA Strategy — Optimal Reciprocal Collision Avoidance
# ═══════════════════════════════════════════════════════════════════════════════
#
# Translated from the C++ ORCAMAPFStrategy reference implementation.
# Key adaptation: C++ uses XZ-plane (Y=up); Gazebo uses XY-plane (Z=up).
# All 2D math (determinants, perpendiculars) operates on (x, y) components.
# ═══════════════════════════════════════════════════════════════════════════════

import logging
import sys

_orca_logger = logging.getLogger("ORCAStrategy")


class _ORCALine:
    """A half-plane constraint.  Velocities on the positive side of the
    directed line (point, direction) are permitted."""
    __slots__ = ("point", "direction")

    def __init__(self, point: np.ndarray, direction: np.ndarray):
        self.point = point          # 2-vector
        self.direction = direction  # 2-vector (unit)


class ORCAStrategy(PathPlanningStrategy):
    """
    Velocity-obstacle path planner using ORCA half-planes resolved via
    iterative linear programming.

    Each nearby obstacle voxel and the central ship produce one half-plane
    constraint.  The LP finds the velocity closest to the preferred
    (goal-seeking) velocity that satisfies every constraint simultaneously.

    Unlike APF, forces are never *summed* across voxels, so even a dense
    wall of detections cannot produce an unbounded repulsive blow-up.
    """

    def __init__(self):
        # ── Shared parameters (same semantics as APFStrategy) ─────────
        self.step_size = 0.2
        self.max_velocity = 4.0
        self.velocity_smoothing = 0.42
        self.goal_tolerance = 1.5
        self.braking_radius = 7.5
        self.min_z = 1.0
        self.max_z = 50.0

        # ── ORCA-specific parameters ──────────────────────────────────
        self.time_horizon_obst = 1.5      # seconds — how far ahead to avoid obstacles
        self.agent_radius = 2.0           # conservative bounding sphere (meters)
        self.influence_radius = 8.0       # only consider voxels within this range
        self.neighbor_dist = 5.0          # max distance for inter-agent ORCA (future)

        self.agent_vertical_radius = 0.5
        self.voxel_vertical_radius = 0.25
        self.orca_vertical_margin  = 1.0

        # ── Environment constraints ───────────────────────────────────
        self.ship_center = np.array([0.0, 0.0], dtype=float)
        self.ship_keepout_radius = 18.0

        # ── Internal state ────────────────────────────────────────────
        self._goal_reached = False
        self.last_velocity = np.zeros(3, dtype=float)
        self._step_count = 0

    # ------------------------------------------------------------------
    # configure()
    # ------------------------------------------------------------------
    def configure(self, config: Dict[str, Any]):
        """Accept the same config dict the agent passes to APF.
        Shared keys are consumed; APF-only keys are silently ignored."""
        self.step_size          = float(config.get("step_size", self.step_size))
        self.max_velocity       = float(config.get("max_velocity", self.max_velocity))
        self.velocity_smoothing = float(config.get("velocity_smoothing", self.velocity_smoothing))
        self.goal_tolerance     = float(config.get("goal_tolerance", self.goal_tolerance))
        self.braking_radius     = float(config.get("braking_radius", self.braking_radius))
        self.ship_keepout_radius = float(config.get("ship_keepout_radius", self.ship_keepout_radius))
        self.min_z              = float(config.get("min_z", self.min_z))
        self.max_z              = float(config.get("max_z", self.max_z))

        # ORCA-specific (optional overrides)
        self.time_horizon_obst = float(config.get("time_horizon_obst", self.time_horizon_obst))
        self.agent_radius      = float(config.get("agent_radius", self.agent_radius))
        self.influence_radius  = float(config.get("influence_radius", self.influence_radius))
        self.orca_vertical_margin = float(config.get("orca_vertical_margin", self.orca_vertical_margin))

        self._goal_reached = False
        self.last_velocity = np.zeros(3, dtype=float)
        self._step_count = 0

    def is_goal_reached(self) -> bool:
        return self._goal_reached

    # ==================================================================
    # 2-D  MATH  HELPERS  (XY plane)
    # ==================================================================

    @staticmethod
    def _det(v1: np.ndarray, v2: np.ndarray) -> float:
        """2-D cross product (determinant) in the XY plane."""
        return float(v1[0] * v2[1] - v1[1] * v2[0])

    def _is_valid_velocity(self, velocity: np.ndarray, line: _ORCALine) -> bool:
        """Is *velocity* on the permitted (left) side of the half-plane?"""
        diff = velocity - line.point
        return self._det(line.direction, diff) >= -1e-5

    @staticmethod
    def _project_on_line(velocity: np.ndarray, line: _ORCALine) -> np.ndarray:
        """Closest point on *line* to *velocity*."""
        diff = velocity - line.point
        t = float(np.dot(diff, line.direction))
        return line.point + line.direction * t

    # ==================================================================
    # ORCA  LINE  GENERATION
    # ==================================================================

    def _compute_obstacle_orca_lines(
        self, agent_pos_2d: np.ndarray, agent_vel_2d: np.ndarray,
        voxel_map, agent_z: float,
    ) -> list:
        """Convert each nearby voxel into one ORCA half-plane.

        Mirrors C++ ``computeObstacleORCALines`` but in the XY plane and
        using the voxel map's spatial query instead of bounding-box clamp.
        """
        lines: list[_ORCALine] = []
        nearby = voxel_map.get_nearby_obstacles(
            agent_pos_2d[0], agent_pos_2d[1], agent_z, self.influence_radius,
        )

        eff_radius = self.agent_radius + 0.5   # slight inflation for smoother sliding
        max_v_dist = self.agent_vertical_radius + self.voxel_vertical_radius + self.orca_vertical_margin

        for obs in nearby:
            obs_z = obs.get("z", None)
            if obs_z is not None and not np.isnan(obs_z):
                if abs(obs_z - agent_z) > max_v_dist:
                    continue  # Skip obstacle outside vertical collision envelope

            obs_2d = np.array([obs["x"], obs["y"]], dtype=float)
            rel_pos = obs_2d - agent_pos_2d
            dist_sq = float(np.dot(rel_pos, rel_pos))

            if dist_sq >= self.influence_radius * self.influence_radius:
                continue

            dist = np.sqrt(dist_sq)
            if dist < 1e-3:
                continue  # degenerate — on top of voxel

            direction = rel_pos / dist
            # Perpendicular in XY:  (-dy, dx)
            line_dir = np.array([-direction[1], direction[0]], dtype=float)
            # How far the velocity must be pushed *away* from the obstacle
            u = (eff_radius - dist) / max(self.time_horizon_obst, 1e-6)
            line_point = agent_vel_2d - direction * u

            lines.append(_ORCALine(point=line_point, direction=line_dir))

        return lines

    def _compute_ship_orca_line(
        self, agent_pos_2d: np.ndarray, agent_vel_2d: np.ndarray,
    ) -> _ORCALine | None:
        """Explicit half-plane for the central ship (hard-coded at origin,
        keepout radius from GazeboSimulator.cpp)."""
        ship_vec = self.ship_center - agent_pos_2d
        ship_dist = float(np.linalg.norm(ship_vec))

        # Only generate a constraint when the agent is close enough to care.
        if ship_dist < 1e-6 or ship_dist >= self.ship_keepout_radius + self.influence_radius:
            return None

        direction = ship_vec / ship_dist
        line_dir = np.array([-direction[1], direction[0]], dtype=float)
        u = (self.ship_keepout_radius + self.agent_radius - ship_dist) / max(self.time_horizon_obst, 1e-6)
        line_point = agent_vel_2d - direction * u

        return _ORCALine(point=line_point, direction=line_dir)

    # ==================================================================
    # LINEAR  PROGRAM  SOLVER
    # ==================================================================

    def _linear_program(
        self, lines: list, pref_velocity: np.ndarray, max_speed: float,
    ) -> np.ndarray:
        """Iteratively project *pref_velocity* onto ORCA half-planes.

        Mirrors the C++ ``linearProgram`` method.  If projection onto line *i*
        violates an earlier line *j < i*, we fall back to zero velocity rather
        than solving a full 2-D LP (matches the reference implementation's
        fallback behaviour).
        """
        result = pref_velocity.copy()

        # Clip to speed disc
        speed_sq = float(np.dot(result, result))
        if speed_sq > max_speed * max_speed:
            result = (result / np.sqrt(speed_sq)) * max_speed

        for i, line in enumerate(lines):
            if self._is_valid_velocity(result, line):
                continue  # already satisfies this half-plane

            # Project onto the violated line
            projected = self._project_on_line(result, line)

            # Re-clip to speed disc
            proj_speed_sq = float(np.dot(projected, projected))
            if proj_speed_sq > max_speed * max_speed:
                projected = (projected / np.sqrt(proj_speed_sq)) * max_speed

            # Verify the projection still satisfies all earlier lines
            satisfies_all = True
            for j in range(i):
                if not self._is_valid_velocity(projected, lines[j]):
                    satisfies_all = False
                    break

            if satisfies_all:
                result = projected
            else:
                # Infeasible — stop to be safe
                result = np.zeros(2, dtype=float)
                break

        return result

    # ==================================================================
    # compute_velocity()  —  main entry point
    # ==================================================================

    def compute_velocity(self, current_pose: dict, goal_pose: dict, voxel_map) -> dict:
        self._step_count += 1

        curr_pos = np.array([current_pose["x"], current_pose["y"], current_pose["z"]], dtype=float)
        goal_pos = np.array([goal_pose["x"], goal_pose["y"], goal_pose["z"]], dtype=float)

        to_goal = goal_pos - curr_pos
        dist_to_goal = float(np.linalg.norm(to_goal))

        # ── Goal arrival check ────────────────────────────────────────
        if dist_to_goal <= self.goal_tolerance:
            self._goal_reached = True
            self.last_velocity = np.zeros(3, dtype=float)
            return {
                "linear":  {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        self._goal_reached = False

        # ── 1. Preferred velocity (XY) ────────────────────────────────
        to_goal_2d = to_goal[:2]
        dist_2d = float(np.linalg.norm(to_goal_2d))

        pref_vel_2d = np.zeros(2, dtype=float)
        if dist_2d > 1e-6:
            base_speed = min(self.max_velocity, dist_2d / max(self.step_size, 1e-6))
            brake_scale = np.clip(
                (dist_2d - self.goal_tolerance) / max(self.braking_radius, 1e-3),
                0.0, 1.0,
            )
            pref_vel_2d = (to_goal_2d / dist_2d) * (base_speed * brake_scale)

        # ── 2. Build ORCA lines ───────────────────────────────────────
        agent_vel_2d = self.last_velocity[:2].copy()
        orca_lines = self._compute_obstacle_orca_lines(
            curr_pos[:2], agent_vel_2d, voxel_map, curr_pos[2],
        )
        ship_line = self._compute_ship_orca_line(curr_pos[:2], agent_vel_2d)
        if ship_line is not None:
            orca_lines.append(ship_line)

        # ── 3. Solve LP ───────────────────────────────────────────────
        opt_vel_2d = self._linear_program(orca_lines, pref_vel_2d, self.max_velocity)

        # ── Debug logging (every 100 ticks) ───────────────────────────
        if self._step_count % 100 == 0:
            ship_dist = float(np.linalg.norm(self.ship_center - curr_pos[:2]))
            print(
                f"ORCA Debug: pos=[{curr_pos[0]:.1f},{curr_pos[1]:.1f},{curr_pos[2]:.1f}] "
                f"pref={pref_vel_2d} opt={opt_vel_2d} "
                f"lines={len(orca_lines)} ship_d={ship_dist:.1f}",
                file=sys.stderr,
            )

        # ── 4. Z-axis altitude control ────────────────────────────────
        z_vel = 0.0
        if dist_to_goal > 1e-3:
            z_diff = to_goal[2]
            z_vel = float(np.clip(z_diff * 0.5, -self.max_velocity * 0.5, self.max_velocity * 0.5))

        # Hard altitude limits
        next_z = curr_pos[2] + z_vel * self.step_size
        if next_z < self.min_z and z_vel < 0:
            z_vel = max(0.0, (self.min_z - curr_pos[2]) / max(self.step_size, 1e-6))
        elif next_z > self.max_z and z_vel > 0:
            z_vel = min(0.0, (self.max_z - curr_pos[2]) / max(self.step_size, 1e-6))

        # ── 5. Compose 3-D velocity & smooth ──────────────────────────
        v_raw = np.array([opt_vel_2d[0], opt_vel_2d[1], z_vel], dtype=float)

        alpha = float(np.clip(self.velocity_smoothing, 0.0, 1.0))
        v_smooth = (1.0 - alpha) * self.last_velocity + alpha * v_raw
        self.last_velocity = v_smooth.copy()

        # Final speed cap
        speed = float(np.linalg.norm(self.last_velocity))
        if speed > self.max_velocity:
            self.last_velocity = (self.last_velocity / speed) * self.max_velocity

        return {
            "linear": {
                "x": float(self.last_velocity[0]),
                "y": float(self.last_velocity[1]),
                "z": float(self.last_velocity[2]),
            },
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
        }