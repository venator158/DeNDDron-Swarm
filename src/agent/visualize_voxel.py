#!/usr/bin/env python3
"""
Live VoxelMap Visualizer

Displays the current voxel map from running agents.
No arguments needed - automatically finds and displays latest voxel map data.
"""

import json
import sys
from pathlib import Path
from datetime import datetime
import logging

sys.path.insert(0, str(Path(__file__).parent))

from voxel_map import VoxelMap
from voxel_map_visualizer import VoxelMapVisualizer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VoxelMapVisualizer")


def load_voxel_map(filepath: Path) -> VoxelMap:
    """Load voxel map from JSON file."""
    try:
        with open(filepath, 'r') as f:
            snapshot = json.load(f)

        voxel_data = snapshot.get('voxel_map', {})
        voxels_reconstructed = {}

        # Convert string keys back to tuples
        for key_str, value in voxel_data.get('voxels', {}).items():
            try:
                key = eval(key_str)
                voxels_reconstructed[key] = value
            except:
                pass

        vmap = VoxelMap()
        vmap.voxels = voxels_reconstructed
        vmap.voxel_timestamps = {k: None for k in voxels_reconstructed.keys()}

        return vmap
    except Exception as e:
        logger.error(f"Failed to load voxel map: {e}")
        return None


def find_latest_voxel_map() -> Path:
    """Find the latest saved voxel map file."""
    voxel_dir = Path("/tmp")
    files = sorted(voxel_dir.glob("voxel_agent_*.json"), 
                  key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def main():
    print("=" * 70)
    print("VoxelMap Live Visualizer")
    print("=" * 70)

    voxel_file = find_latest_voxel_map()

    if not voxel_file:
        print("\n❌ No voxel map data found!")
        print("\nMake sure agents are running and collecting LiDAR data.")
        print("Files searched: /tmp/voxel_agent_*.json")
        return 1

    print(f"\n📍 Found voxel map: {voxel_file.name}")
    print(f"   Modified: {datetime.fromtimestamp(voxel_file.stat().st_mtime)}")

    voxel_map = load_voxel_map(voxel_file)

    if not voxel_map or not voxel_map.voxels:
        print("\n❌ Voxel map is empty!")
        return 1

    stats = voxel_map.get_stats()
    print(f"\n📊 Voxel Map Statistics:")
    print(f"   Total voxels: {stats['total_voxels']}")
    print(f"   Occupied: {stats['occupied']}")
    print(f"   Free: {stats['free']}")
    print(f"   Memory: {stats['memory_mb']:.2f} MB")

    visualizer = VoxelMapVisualizer(voxel_map)

    print(f"\n🎨 Generating visualizations...")
    print("   • 2D slice at z=20m...")
    visualizer.plot_2d_slice(z_slice=20.0)

    print("   • 3D matplotlib plot...")
    visualizer.plot_3d_matplotlib()

    try:
        import plotly
        print("   • 3D interactive Plotly...")
        html_file = "/tmp/voxel_map_live.html"
        visualizer.plot_3d_plotly(save_path=html_file)
        print(f"\n✅ Saved interactive 3D: {html_file}")
    except ImportError:
        print("   (Plotly not installed - skipping interactive 3D)")

    print("\n" + "=" * 70)
    print("Visualization complete!")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
