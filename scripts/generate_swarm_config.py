#!/usr/bin/env python3
"""Generate runtime swarm config and env file for Docker scaling."""

import argparse
import json
import math
import random
from pathlib import Path


def _distance(a: tuple, b: tuple) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def build_agent_positions(
    agent_count: int,
    center_x: float,
    center_y: float,
    z: float,
    min_radius: float,
    max_radius: float,
    min_separation: float,
):
    agents = {}
    points = []

    for i in range(1, agent_count + 1):
        agent_id = f"drone_{i}"

        # Rejection sample until we find a point in allowed range and far enough
        # from other agents to avoid immediate overlap.
        x = center_x
        y = center_y
        for _ in range(500):
            radius = random.uniform(min_radius, max_radius)
            theta = random.uniform(0.0, 2.0 * math.pi)
            candidate = (
                center_x + radius * math.cos(theta),
                center_y + radius * math.sin(theta),
            )
            if all(_distance(candidate, prev) >= min_separation for prev in points):
                x, y = candidate
                break

        points.append((x, y))
        agents[agent_id] = {
            "spawn": {
                "x": x,
                "y": y,
                "z": z,
            }
        }
    return agents


def main():
    parser = argparse.ArgumentParser(description="Generate runtime config for DeNDDron swarm")
    parser.add_argument("--agents", type=int, default=3, help="Number of agents to spawn")
    parser.add_argument("--x", type=float, default=-45.0, help="Spawn area center X")
    parser.add_argument("--y", type=float, default=0.0, help="Spawn area center Y")
    parser.add_argument("--z", type=float, default=20.0, help="Common spawn Z coordinate")
    parser.add_argument("--min-radius", type=float, default=12.0, help="Minimum distance from center")
    parser.add_argument("--max-radius", type=float, default=35.0, help="Maximum distance from center")
    parser.add_argument("--min-separation", type=float, default=8.0, help="Minimum spacing between agents")
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed for reproducible layouts")
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

    if args.seed is not None:
        random.seed(args.seed)

    runtime = {
        "agent_count": args.agents,
        "agents": build_agent_positions(
            args.agents,
            args.x,
            args.y,
            args.z,
            args.min_radius,
            args.max_radius,
            args.min_separation,
        ),
    }

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
