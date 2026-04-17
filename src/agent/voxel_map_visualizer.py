"""
VoxelMapVisualizer: Multi-method visualization of 3D occupancy grids.

Supports:
1. Matplotlib 2D slices (always available)
2. Plotly 3D interactive (optional, requires: pip install plotly)
3. Open3D point cloud (optional, requires: pip install open3d)
"""

import numpy as np
import logging
from typing import Optional

logger = logging.getLogger("VoxelMapVisualizer")


class VoxelMapVisualizer:
    """Visualize 3D voxel maps with multiple backends."""

    def __init__(self, voxel_map):
        """
        Initialize visualizer.

        Args:
            voxel_map: VoxelMap instance to visualize
        """
        self.voxel_map = voxel_map

    # ==========================================
    # MATPLOTLIB 2D SLICE VISUALIZATION
    # ==========================================
    def plot_2d_slice(self, z_slice: Optional[float] = None, save_path: Optional[str] = None):
        """
        Visualize a 2D horizontal slice of the voxel map using matplotlib.

        Args:
            z_slice: Z-coordinate to slice at. If None, uses z = 20 (typical drone height).
            save_path: If provided, saves figure to this path instead of showing.
        """
        try:
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
        except ImportError:
            logger.error("Matplotlib required for 2D visualization. Install: pip install matplotlib")
            return

        if z_slice is None:
            z_slice = 20.0  # Typical drone height

        # Find voxels near the slice
        resolution = self.voxel_map.RESOLUTION
        z_tolerance = resolution * 1.5

        occupied_2d = []
        free_2d = []

        for (vx, vy, vz), occupancy in self.voxel_map.voxels.items():
            x, y, z = self.voxel_map._voxel_to_world(vx, vy, vz)

            # Include voxels within tolerance of slice
            if abs(z - z_slice) < z_tolerance:
                if occupancy >= 0.5:
                    occupied_2d.append((x, y))
                else:
                    free_2d.append((x, y))

        fig, ax = plt.subplots(figsize=(12, 10))

        # Plot free voxels (light)
        if free_2d:
            free_x, free_y = zip(*free_2d)
            ax.scatter(free_x, free_y, s=10, c='lightblue', alpha=0.3, label='Free')

        # Plot occupied voxels (dark)
        if occupied_2d:
            occ_x, occ_y = zip(*occupied_2d)
            ax.scatter(occ_x, occ_y, s=50, c='red', alpha=0.8, label='Occupied')
        
        if not free_2d and not occupied_2d:
            # No data to visualize
            ax.text(0.5, 0.5, 'No voxel data to visualize', 
                   horizontalalignment='center', verticalalignment='center',
                   transform=ax.transAxes, fontsize=14)

        # Map boundaries
        ax.axhline(y=self.voxel_map.MIN_Y, color='k', linestyle='--', alpha=0.5)
        ax.axhline(y=self.voxel_map.MAX_Y, color='k', linestyle='--', alpha=0.5)
        ax.axvline(x=self.voxel_map.MIN_X, color='k', linestyle='--', alpha=0.5)
        ax.axvline(x=self.voxel_map.MAX_X, color='k', linestyle='--', alpha=0.5)

        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title(f'Voxel Map 2D Slice at Z={z_slice}m (Resolution: {resolution}m)')
        
        # Only add legend if there are labeled artists
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend()
        
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')

        stats = self.voxel_map.get_stats()
        ax.text(0.02, 0.98, f"Occupied: {stats['occupied']}\nFree: {stats['free']}",
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            logger.info(f"Saved 2D slice to {save_path}")
        else:
            plt.show()

        plt.close(fig)

    def plot_3d_matplotlib(self, save_path: Optional[str] = None):
        """
        Visualize 3D voxel map using matplotlib (limited but always available).

        Args:
            save_path: If provided, saves figure to this path.
        """
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D
        except ImportError:
            logger.error("Matplotlib required for 3D visualization. Install: pip install matplotlib")
            return

        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Get voxel data (limit to 5000 voxels for performance)
        data = self.voxel_map.get_voxel_data()
        occupied = data['occupied'][:5000]
        free = data['free'][:5000]

        # Plot free voxels
        if free:
            free_x, free_y, free_z, _ = zip(*free)
            ax.scatter(free_x, free_y, free_z, s=5, c='lightblue', alpha=0.2, label='Free')

        # Plot occupied voxels
        if occupied:
            occ_x, occ_y, occ_z, _ = zip(*occupied)
            ax.scatter(occ_x, occ_y, occ_z, s=20, c='red', alpha=0.8, label='Occupied')
        
        if not free and not occupied:
            # No data - add a dummy point to show structure
            ax.scatter([0], [0], [0], s=1, c='gray', alpha=0.1)

        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_zlabel('Z (meters)')
        ax.set_title('Voxel Map 3D Visualization')
        
        # Only add legend if there are labeled artists
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend()

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            logger.info(f"Saved 3D plot to {save_path}")
        else:
            plt.show()

        plt.close(fig)

    # ==========================================
    # PLOTLY 3D INTERACTIVE VISUALIZATION
    # ==========================================
    def plot_3d_plotly(self, save_path: Optional[str] = None, max_voxels: int = 10000) -> Optional[str]:
        """
        Visualize 3D voxel map using Plotly (interactive, web-based).

        Args:
            save_path: If provided, saves HTML to this path. Otherwise returns HTML string.
            max_voxels: Max voxels to display (for performance).

        Returns:
            HTML string if save_path is None, otherwise None.

        Requires: pip install plotly
        """
        try:
            import plotly.graph_objects as go
            import plotly.express as px
        except ImportError:
            logger.error("Plotly required for interactive 3D. Install: pip install plotly")
            return None

        data = self.voxel_map.get_voxel_data()
        occupied = data['occupied'][:max_voxels]
        free = data['free'][:int(max_voxels * 0.2)]  # Show fewer free voxels for clarity

        fig = go.Figure()

        # Add occupied voxels
        if occupied:
            occ_x, occ_y, occ_z, occ_conf = zip(*occupied)
            fig.add_trace(go.Scatter3d(
                x=occ_x, y=occ_y, z=occ_z,
                mode='markers',
                marker=dict(
                    size=3,
                    color=occ_conf,
                    colorscale='Reds',
                    opacity=0.8,
                    showscale=True,
                    colorbar=dict(title="Occupancy")
                ),
                name='Occupied',
                text=[f"Occupancy: {c:.2f}" for c in occ_conf],
                hoverinfo='text'
            ))

        # Add free voxels (light)
        if free:
            free_x, free_y, free_z, _ = zip(*free)
            fig.add_trace(go.Scatter3d(
                x=free_x, y=free_y, z=free_z,
                mode='markers',
                marker=dict(size=2, color='lightblue', opacity=0.1),
                name='Free'
            ))
        
        # If no data, add a placeholder trace
        if not occupied and not free:
            fig.add_trace(go.Scatter3d(
                x=[0], y=[0], z=[0],
                mode='markers',
                marker=dict(size=1, color='gray', opacity=0.1),
                name='Empty'
            ))

        # Set layout
        stats = data['stats']
        fig.update_layout(
            title=f"Voxel Map 3D ({stats['total_voxels']} voxels, "
                  f"{stats['occupied']} occupied, {stats['free']} free)",
            scene=dict(
                xaxis_title='X (meters)',
                yaxis_title='Y (meters)',
                zaxis_title='Z (meters)',
                xaxis=dict(range=[self.voxel_map.MIN_X, self.voxel_map.MAX_X]),
                yaxis=dict(range=[self.voxel_map.MIN_Y, self.voxel_map.MAX_Y]),
                zaxis=dict(range=[self.voxel_map.MIN_Z, self.voxel_map.MAX_Z])
            ),
            width=1200,
            height=900,
            hovermode='closest'
        )

        if save_path:
            fig.write_html(save_path)
            logger.info(f"Saved interactive 3D plot to {save_path}")
            return save_path
        else:
            html = fig.to_html()
            return html

    # ==========================================
    # OPEN3D POINT CLOUD VISUALIZATION
    # ==========================================
    def plot_3d_open3d(self):
        """
        Visualize voxel map using Open3D point cloud viewer.

        Requires: pip install open3d
        """
        try:
            import open3d as o3d
        except ImportError:
            logger.error("Open3D required for point cloud visualization. Install: pip install open3d")
            return

        data = self.voxel_map.get_voxel_data()
        occupied = data['occupied']
        free = data['free'][:len(occupied) // 5] if occupied else []

        # Create point clouds
        points = []
        colors = []

        # Add occupied points (red)
        for x, y, z, conf in occupied:
            points.append([x, y, z])
            colors.append([1.0, 0.0, 0.0])  # Red

        # Add free points (light blue)
        for x, y, z, conf in free:
            points.append([x, y, z])
            colors.append([0.68, 0.85, 0.9])  # Light blue
        
        if not points:
            # No data - create a single point at origin
            points.append([0.0, 0.0, 0.0])
            colors.append([0.5, 0.5, 0.5])
            logger.warning("No voxel data to visualize - showing empty point cloud")

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

        logger.info("Displaying Open3D viewer (close window to continue)")
        o3d.visualization.draw_geometries([pcd])

    # ==========================================
    # EXPORT FOR EXTERNAL VISUALIZATION
    # ==========================================
    def export_to_csv(self, filepath: str):
        """
        Export voxel map to CSV for external analysis/visualization.

        Args:
            filepath: Path to save CSV file (e.g., 'voxel_map.csv')
        """
        import csv

        data = self.voxel_map.get_voxel_data()

        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z', 'occupancy', 'type'])

                for x, y, z, conf in data['occupied']:
                    writer.writerow([x, y, z, conf, 'occupied'])

                for x, y, z, conf in data['free']:
                    writer.writerow([x, y, z, conf, 'free'])

            logger.info(f"Exported voxel map to {filepath}")
        except Exception as e:
            logger.error(f"Failed to export CSV: {e}")

    def export_to_json(self, filepath: str):
        """
        Export voxel map to JSON for external analysis.

        Args:
            filepath: Path to save JSON file (e.g., 'voxel_map.json')
        """
        import json

        data = self.voxel_map.export_to_dict()
        # Convert tuple keys to strings for JSON serialization
        voxels_serializable = {str(k): v for k, v in data['voxels'].items()}

        try:
            with open(filepath, 'w') as f:
                json.dump({
                    'voxels': voxels_serializable,
                    'bounds': data['bounds'],
                    'resolution': data['resolution'],
                    'stats': self.voxel_map.get_stats()
                }, f, indent=2)

            logger.info(f"Exported voxel map to {filepath}")
        except Exception as e:
            logger.error(f"Failed to export JSON: {e}")
