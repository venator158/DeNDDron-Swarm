#!/usr/bin/env python3
"""
Live VoxelMap Recorder & Visualizer

Records voxel map snapshots during actual agent operation.
Can be run alongside the swarm to visualize real-time mapping.

Usage:
    # Start recording (runs in background, saves every 10 seconds)
    python3 record_voxel_map.py --agent drone_1 --interval 10

    # Later, visualize the recorded map
    python3 record_voxel_map.py --visualize --input /tmp/voxel_agent_1.json --plot 3d_mpl
"""

import json
import argparse
import sys
import time
from pathlib import Path
import threading
import logging

sys.path.insert(0, str(Path(__file__).parent))

from voxel_map import VoxelMap
from voxel_map_visualizer import VoxelMapVisualizer

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("VoxelMapRecorder")


class VoxelMapRecorder:
    """Record voxel map from live agent or file."""

    def __init__(self, agent_id: str, output_dir: str = "/tmp"):
        self.agent_id = agent_id
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.output_file = self.output_dir / f"voxel_agent_{agent_id}.json"
        self.voxel_map = VoxelMap()
        self.last_stats = None

    def record_snapshot(self, description: str = ""):
        """Take and save a snapshot of the current voxel map."""
        stats = self.voxel_map.get_stats()
        self.last_stats = stats

        snapshot = {
            "timestamp": time.time(),
            "agent_id": self.agent_id,
            "description": description,
            "voxel_map": self.voxel_map.export_to_dict(),
            "stats": stats
        }

        try:
            with open(self.output_file, 'w') as f:
                json.dump(snapshot, f)
            logger.info(f"[{self.agent_id}] Snapshot saved: {stats['total_voxels']} voxels, "
                       f"{stats['occupied']} occupied")
        except Exception as e:
            logger.error(f"Failed to save snapshot: {e}")

    def load_snapshot(self, filepath: str):
        """Load a voxel map from saved snapshot."""
        try:
            with open(filepath, 'r') as f:
                snapshot = json.load(f)

            # Reconstruct voxel map
            voxel_data = snapshot.get('voxel_map', {})
            # Convert string keys back to tuples
            voxels_reconstructed = {}
            for key_str, value in voxel_data.get('voxels', {}).items():
                # Parse "([x], [y], [z])" format back to tuple
                try:
                    key = eval(key_str)
                    voxels_reconstructed[key] = value
                except:
                    pass

            self.voxel_map.voxels = voxels_reconstructed
            self.voxel_map.voxel_timestamps = {k: None for k in voxels_reconstructed.keys()}
            self.last_stats = snapshot.get('stats')

            logger.info(f"Loaded snapshot with {len(voxels_reconstructed)} voxels")
            return True
        except Exception as e:
            logger.error(f"Failed to load snapshot: {e}")
            return False

    def continuous_record(self, interval: int = 10):
        """
        Placeholder for continuous recording from live agent.
        In real use, this would subscribe to agent's voxel map updates.
        """
        logger.info(f"[{self.agent_id}] Starting continuous recording (interval: {interval}s)")
        logger.info("Note: This is a placeholder. Use with live agent integration for real data.")

        try:
            count = 0
            while True:
                time.sleep(interval)
                count += 1
                self.record_snapshot(description=f"Auto-snapshot #{count}")
        except KeyboardInterrupt:
            logger.info(f"[{self.agent_id}] Recording stopped by user")


def main():
    parser = argparse.ArgumentParser(description="VoxelMap Recorder & Analyzer")
    parser.add_argument("--agent", default="drone_1",
                       help="Agent ID (for recording)")
    parser.add_argument("--output-dir", default="/tmp",
                       help="Output directory for snapshots")
    parser.add_argument("--interval", type=int, default=10,
                       help="Recording interval in seconds")
    parser.add_argument("--visualize", action="store_true",
                       help="Load and visualize existing snapshot")
    parser.add_argument("--input", default=None,
                       help="Input snapshot file (for visualization)")
    parser.add_argument("--plot", default="3d_mpl",
                       choices=["2d", "3d_mpl", "3d_plotly"],
                       help="Visualization backend")
    parser.add_argument("--export-csv", default=None,
                       help="Export to CSV")
    parser.add_argument("--export-json", default=None,
                       help="Export to JSON")

    args = parser.parse_args()

    recorder = VoxelMapRecorder(args.agent, args.output_dir)

    if args.visualize:
        # Load and visualize existing snapshot
        input_file = args.input or recorder.output_file
        print(f"\n[Visualizer] Loading snapshot from {input_file}...")

        if not recorder.load_snapshot(str(input_file)):
            print("Failed to load snapshot!")
            return

        print(f"[Visualizer] Loaded map: {recorder.last_stats}")

        visualizer = VoxelMapVisualizer(recorder.voxel_map)

        if args.plot == "2d":
            visualizer.plot_2d_slice(z_slice=20.0)
        elif args.plot == "3d_mpl":
            visualizer.plot_3d_matplotlib()
        elif args.plot == "3d_plotly":
            output_html = f"/tmp/voxel_{args.agent}.html"
            visualizer.plot_3d_plotly(save_path=output_html)
            print(f"✓ Saved interactive plot to {output_html}")

        if args.export_csv:
            visualizer.export_to_csv(args.export_csv)
        if args.export_json:
            visualizer.export_to_json(args.export_json)

    else:
        # Recording mode (placeholder)
        print(f"\n[Recorder] Starting recording for {args.agent}")
        print(f"[Recorder] Snapshots will be saved to: {recorder.output_file}")
        print(f"[Recorder] To visualize later: python3 record_voxel_map.py --visualize")
        try:
            recorder.continuous_record(interval=args.interval)
        except KeyboardInterrupt:
            print("\nRecording stopped.")


if __name__ == "__main__":
    main()
