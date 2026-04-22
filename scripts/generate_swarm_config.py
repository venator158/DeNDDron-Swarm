#!/usr/bin/env python3
"""Generate runtime swarm config and env file for Docker scaling."""

import argparse
import json
import math
import random
from pathlib import Path


GLOBAL_DEFAULTS = {
    "path_planning": {
        "algorithm": "orca",
        "attractive_gain": 1.8,
        "repulsive_gain": 8.5,
        "influence_radius": 7.0,
        "step_size": 0.2,
        "goal_tolerance": 1.5,
        "braking_radius": 7.5,
        "ship_keepout_radius": 18.0,
        "ship_influence_radius": 30.0,
        "apf_exponential_decay": 0.5,
        "apf_inverse_square_scale": 0.3,
        "apf_stuck_growth_rate": 0.2,
        "min_movement": 0.01,
        "stuck_threshold": 5,
        "velocity_smoothing": 0.42,
    },
    "kinematics": {
        "max_velocity": 4.0,
        "max_acceleration": 1.0,
        "max_z": 50.0,
        "min_z": 1.0,
        "max_service_radius": 100.0,
    },
    "goal_control": {
        "tolerance": 1.5,
        "stop_radius": 3.0,
        "tolerance_xy": 2.0,
        "tolerance_z": 1.5,
        "settle_ticks": 8,
    },
}


def _distance(a: tuple, b: tuple) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def build_agent_positions(
    agent_count: int,
    center_x: float,
    center_y: float,
    z: float,
    z_step: float,
    min_radius: float,
    max_radius: float,
    min_separation: float,
):
    agents = {}
    points = []
    quadrant_angles = [
        (0.0, 0.5 * math.pi),
        (0.5 * math.pi, math.pi),
        (math.pi, 1.5 * math.pi),
        (1.5 * math.pi, 2.0 * math.pi),
    ]
    altitude_layers = max(2, min(6, int(math.ceil(math.sqrt(agent_count)))))

    def _layered_z(index: int, phase_shift: int = 0) -> float:
        centered = ((index + phase_shift) % altitude_layers) - (altitude_layers - 1) / 2.0
        value = z + centered * z_step
        return max(2.0, min(45.0, value))

    for i in range(1, agent_count + 1):
        agent_id = f"drone_{i}"

        # Spread agents around the ship in quadrants, while keeping a bounded
        # radial distance and a minimum separation from other agents.
        x = center_x
        y = center_y
        for _ in range(500):
            radius = random.uniform(min_radius, max_radius)
            quadrant = quadrant_angles[(i - 1) % len(quadrant_angles)]
            theta = random.uniform(quadrant[0], quadrant[1])
            candidate = (
                center_x + radius * math.cos(theta),
                center_y + radius * math.sin(theta),
            )
            
            # Goal needs same constraints (not inside the ship, within max/min radius).
            # If we enforce candidate is good, let's just make the goal a valid reflection.
            # However, simple -x, -y could technically be too close to another drone's goal
            # or inside the ship (but if radius is 30-45, distance to origin is 30-45, so it won't be inside the ship).
            # The ship is presumed at origin (0,0). So reflection through origin preserves distance to ship.
            
            if all(_distance(candidate, prev) >= min_separation for prev in points):
                x, y = candidate
                break

        # Generation uses same constraints, generate completely new valid (x,y)
        goal_x, goal_y = center_x, center_y
        for _ in range(500):
            r2 = random.uniform(min_radius, max_radius)
            t2 = random.uniform(0, 2.0 * math.pi)
            candidate = (center_x + r2 * math.cos(t2), center_y + r2 * math.sin(t2))
            
            # To ensure it isn't too close to its own spawn point (so it actually flies)
            if _distance(candidate, (x, y)) > min_radius:
                goal_x, goal_y = candidate
                break

        spawn_z = _layered_z(i - 1, phase_shift=0)
        # Offset goal layer from spawn layer to encourage 3D trajectories.
        goal_z = _layered_z(i - 1, phase_shift=max(1, altitude_layers // 2))

        points.append((x, y))
        agents[agent_id] = {
            "spawn": {
                "x": x,
                "y": y,
                "z": spawn_z,
            },
            "goal": {
                "x": goal_x,
                "y": goal_y,
                "z": goal_z,
            },
            # Per-agent path_planning/kinematics/goal_control can still be added
            # as overrides, but defaults now live at runtime["defaults"].
        }
    return agents


def main():
    parser = argparse.ArgumentParser(description="Generate runtime config for DeNDDron swarm")
    parser.add_argument("--agents", type=int, default=3, help="Number of agents to spawn")
    parser.add_argument("--x", type=float, default=0.0, help="Spawn area center X")
    parser.add_argument("--y", type=float, default=0.0, help="Spawn area center Y")
    parser.add_argument("--z", type=float, default=20.0, help="Common spawn Z coordinate")
    parser.add_argument("--z-step", type=float, default=3.0, help="Vertical spacing between altitude layers")
    parser.add_argument("--min-radius", type=float, default=30.0, help="Minimum distance from center")
    parser.add_argument("--max-radius", type=float, default=45.0, help="Maximum distance from center")
    parser.add_argument("--min-separation", type=float, default=8.0, help="Minimum spacing between agents")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed for reproducible layouts")
    parser.add_argument("--algorithm", type=str, default="orca", choices=["orca", "apf"], help="Path planning algorithm (orca or apf)")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/swarm_runtime.json"),
        help="Output runtime config path",
    )
    parser.add_argument(
        "--env-output",
        type=Path,
        default=Path(".swarm.env"),
        help="Output env file path consumed by launcher",
    )
    args = parser.parse_args()

    if args.agents <= 0:
        raise ValueError("--agents must be > 0")
    if args.min_radius < 0:
        raise ValueError("--min-radius must be >= 0")
    if args.max_radius <= args.min_radius:
        raise ValueError("--max-radius must be > --min-radius")
    if args.min_separation < 0:
        raise ValueError("--min-separation must be >= 0")
    if args.z_step <= 0:
        raise ValueError("--z-step must be > 0")

    if args.seed is not None:
        random.seed(args.seed)

    runtime = {
        "agent_count": args.agents,
        "defaults": GLOBAL_DEFAULTS,
        "agents": build_agent_positions(
            args.agents,
            args.x,
            args.y,
            args.z,
            args.z_step,
            args.min_radius,
            args.max_radius,
            args.min_separation,
        ),
    }

    if args.algorithm:
        runtime["defaults"]["path_planning"]["algorithm"] = args.algorithm

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(runtime, indent=2) + "\n", encoding="utf-8")

    args.env_output.write_text(
        f"AGENT_COUNT={args.agents}\n"
        "SWARM_RUNTIME_CONFIG=/home/app/config/swarm_runtime.json\n",
        encoding="utf-8",
    )

    print(f"Wrote runtime config: {args.output}")
    print(f"Wrote docker env file: {args.env_output}")


if __name__ == "__main__":
    main()
