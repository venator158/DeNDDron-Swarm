#!/usr/bin/env python3
"""
VoxelMap Test & Visualization Script

Generates synthetic LiDAR data with an agent moving through a simulated environment
containing obstacles. Visualizes the resulting voxel map.

Usage:
    python3 test_voxel_map.py [--plot BACKEND] [--save FILEPATH]

Backends: 2d, 3d_mpl, 3d_plotly, open3d, all
"""

import numpy as np
import argparse
import sys
from pathlib import Path

# Add src/agent to path
sys.path.insert(0, str(Path(__file__).parent))

from voxel_map import VoxelMap
from voxel_map_visualizer import VoxelMapVisualizer


def generate_synthetic_lidar(agent_pos: tuple, obstacles: list, num_rays: int = 16,
                             max_distance: float = 50.0) -> list:
    """
    Generate synthetic LiDAR rays hitting obstacles.

    Args:
        agent_pos: (x, y, z, yaw) agent position and orientation
        obstacles: List of (center_x, center_y, center_z, size) for spheres
        num_rays: Number of rays
        max_distance: Max ray distance

    Returns:
        List of ray measurements
    """
    agent_x, agent_y, agent_z, yaw = agent_pos
    rays = []

    for i in range(num_rays):
        # Ray angle relative to robot orientation
        ray_angle = (2.0 * np.pi * i) / num_rays
        world_angle = yaw + ray_angle

        # Ray direction
        ray_dx = np.cos(world_angle)
        ray_dy = np.sin(world_angle)

        # Find intersection with obstacles
        min_distance = max_distance
        hit = False

        for obs_x, obs_y, obs_z, obs_size in obstacles:
            # Ray from agent in world frame
            ray_start = np.array([agent_x, agent_y, agent_z])
            ray_dir = np.array([ray_dx, ray_dy, 0.0])

            # Vector from ray start to obstacle center
            to_obs = np.array([obs_x - agent_x, obs_y - agent_y, obs_z - agent_z])

            # Project to find closest point on ray
            proj_len = np.dot(to_obs, ray_dir)

            if proj_len > 0 and proj_len < max_distance:
                # Closest point on ray
                closest = ray_start + proj_len * ray_dir
                # Distance to obstacle center
                dist_to_center = np.linalg.norm(np.array([obs_x, obs_y, obs_z]) - closest)

                if dist_to_center < obs_size:
                    dist_to_surface = proj_len - np.sqrt(obs_size ** 2 - dist_to_center ** 2)
                    if 0 < dist_to_surface < min_distance:
                        min_distance = dist_to_surface
                        hit = True

        if hit:
            rays.append({
                "angle": ray_angle,
                "distance": min_distance,
                "intensity": 0.8,
                "ray_id": i
            })
        else:
            # No hit, use max distance
            rays.append({
                "angle": ray_angle,
                "distance": max_distance,
                "intensity": 0.1,
                "ray_id": i
            })

    return rays


def simulate_agent_trajectory(voxel_map: VoxelMap, num_positions: int = 20):
    """
    Simulate agent moving through environment and collecting LiDAR data.

    Args:
        voxel_map: VoxelMap instance to populate
        num_positions: Number of agent positions to simulate
    """
    # Define obstacles (simple test case)
    obstacles = [
        (0.0, 0.0, 15.0, 15.0),   # Ship at origin, 15m radius
        (50.0, 30.0, 10.0, 8.0),   # Additional obstacle
        (-40.0, -20.0, 12.0, 6.0)  # Another obstacle
    ]

    # Agent trajectory: circular path
    print(f"[Simulator] Simulating {num_positions} agent positions...")

    for i in range(num_positions):
        t = i / (num_positions - 1)
        radius = 60.0
        angle = 2.0 * np.pi * t
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = 20.0
        yaw = angle

        agent_pos = (x, y, z, yaw)

        # Generate synthetic LiDAR
        rays = generate_synthetic_lidar(agent_pos, obstacles, num_rays=16, max_distance=50.0)

        # Process rays as agent would
        for ray in rays:
            try:
                angle_rel = ray.get("angle", 0.0)
                distance = ray.get("distance", 0.0)

                if distance <= 0 or distance > 100:
                    continue

                world_angle = yaw + angle_rel
                ray_x = x + distance * np.cos(world_angle)
                ray_y = y + distance * np.sin(world_angle)
                ray_z = z

                voxel_map.raytrace(x, y, z, ray_x, ray_y, ray_z)
            except Exception as e:
                print(f"[Simulator] Error processing ray: {e}")

        if (i + 1) % 5 == 0:
            stats = voxel_map.get_stats()
            print(f"  Position {i + 1}/{num_positions}: "
                  f"Voxels: {stats['total_voxels']}, "
                  f"Occupied: {stats['occupied']}, "
                  f"Memory: {stats['memory_mb']:.2f} MB")

    print("[Simulator] Done!")


def main():
    parser = argparse.ArgumentParser(description="VoxelMap Test & Visualization")
    parser.add_argument("--plot", default="2d",
                       choices=["2d", "3d_mpl", "3d_plotly", "open3d", "all"],
                       help="Visualization backend")
    parser.add_argument("--save", default=None,
                       help="Save visualization to file (e.g., /tmp/voxel_map.html)")
    parser.add_argument("--export-csv", default=None,
                       help="Export voxel map to CSV")
    parser.add_argument("--export-json", default=None,
                       help="Export voxel map to JSON")
    parser.add_argument("--positions", type=int, default=20,
                       help="Number of agent positions to simulate")

    args = parser.parse_args()

    print("=" * 70)
    print("VoxelMap Test & Visualization")
    print("=" * 70)

    # Create voxel map
    voxel_map = VoxelMap()

    # Simulate agent trajectory
    simulate_agent_trajectory(voxel_map, num_positions=args.positions)

    # Print statistics
    stats = voxel_map.get_stats()
    print(f"\n[VoxelMap Stats]")
    print(f"  Total voxels: {stats['total_voxels']}")
    print(f"  Occupied: {stats['occupied']}")
    print(f"  Free: {stats['free']}")
    print(f"  Memory: {stats['memory_mb']:.2f} MB")

    # Create visualizer
    visualizer = VoxelMapVisualizer(voxel_map)

    # Generate visualizations
    print(f"\n[Visualization]")

    if args.plot in ["2d", "all"]:
        print("  Generating 2D slice...")
        save_2d = args.save.replace(".html", "_2d.png") if args.save else None
        visualizer.plot_2d_slice(z_slice=20.0, save_path=save_2d)

    if args.plot in ["3d_mpl", "all"]:
        print("  Generating 3D matplotlib plot...")
        save_3d_mpl = args.save.replace(".html", "_3d_mpl.png") if args.save else None
        visualizer.plot_3d_matplotlib(save_path=save_3d_mpl)

    if args.plot in ["3d_plotly", "all"]:
        print("  Generating interactive 3D Plotly plot...")
        save_plotly = args.save if args.save and args.save.endswith(".html") else "/tmp/voxel_map.html"
        result = visualizer.plot_3d_plotly(save_path=save_plotly, max_voxels=10000)
        if result:
            print(f"  ✓ Saved to {save_plotly}")

    if args.plot == "open3d":
        print("  Opening Open3D viewer...")
        visualizer.plot_3d_open3d()

    # Export options
    if args.export_csv:
        print(f"\n[Export] Exporting to CSV: {args.export_csv}")
        visualizer.export_to_csv(args.export_csv)

    if args.export_json:
        print(f"[Export] Exporting to JSON: {args.export_json}")
        visualizer.export_to_json(args.export_json)

    print("\n" + "=" * 70)
    print("Test complete!")
    print("=" * 70)


if __name__ == "__main__":
    main()
