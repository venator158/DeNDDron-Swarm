# VoxelMap - Complete Usage Guide

## Overview

The VoxelMap system provides real-time 3D obstacle detection and visualization for the DeNDDron swarm. It uses LiDAR data from each drone to build a probabilistic occupancy grid in world-absolute coordinates.

## Components

### 1. **VoxelMap Core** (`src/agent/voxel_map.py`)
- Sparse 3D occupancy grid with dictionary-based storage
- World coordinates: X[-1000, 1000]m, Y[-1000, 1000]m, Z[0, 100]m
- Resolution: 0.5 meters per voxel
- Probabilistic occupancy: 0.0 (free) to 1.0 (occupied)

### 2. **VoxelMapVisualizer** (`src/agent/voxel_map_visualizer.py`)
- Multiple visualization backends (2D, 3D Matplotlib, Plotly, Open3D)
- Export capabilities (CSV, JSON)

### 3. **Test & Demo** (`src/agent/test_voxel_map.py`)
- Generates synthetic LiDAR data with realistic agent trajectories
- Tests all visualization methods

### 4. **Recording Tool** (`src/agent/record_voxel_map.py`)
- Records snapshots of voxel maps during live operation
- Visualizes recorded data offline

### 5. **Agent Integration** (`src/agent/agent.py`)
- Receives LiDAR sensor data from Gazebo
- Transforms rays to world frame
- Updates voxel map via raytrace
- Exports snapshots for analysis

## Installation

```bash
cd src/agent
pip install -r requirements.txt
```

### Required Packages
- `numpy>=1.24.0` - Array operations
- `plotly>=5.0.0` - Interactive 3D visualization
- `matplotlib>=3.5.0` - 2D/3D static plots
- `pyyaml>=6.0` - Configuration parsing
- `eclipse-zenoh==1.0.4` - Distributed messaging

### Optional Packages
```bash
pip install open3d      # Professional point cloud viewer
```

## Quick Start

### 1. Generate Test Data and Visualize

```bash
cd src/agent

# 2D top-down view
python3 test_voxel_map.py --plot 2d --positions 20

# 3D matplotlib view
python3 test_voxel_map.py --plot 3d_mpl --positions 20

# Interactive 3D Plotly (recommended for exploration)
python3 test_voxel_map.py --plot 3d_plotly --positions 20
# Open index.html in the project root in your browser

# Generate all visualizations
python3 test_voxel_map.py --plot all --positions 20
```

### 2. Export for External Analysis

```bash
# CSV format (for Excel, pandas, etc.)
python3 test_voxel_map.py --positions 20 --export-csv /tmp/voxel_data.csv

# JSON format (for JavaScript, etc.)
python3 test_voxel_map.py --positions 20 --export-json /tmp/voxel_data.json
```

### 3. Live Agent Recording

```bash
# Start recording from a live agent (runs in background)
python3 record_voxel_map.py --agent drone_1 --interval 10 &

# Later, visualize the recorded map
python3 record_voxel_map.py --visualize --plot 3d_plotly --agent drone_1
```

## Core APIs

### VoxelMap Class

```python
from voxel_map import VoxelMap

vmap = VoxelMap()

# Mark locations as occupied/free
vmap.mark_occupied(10.0, 20.0, 15.0, confidence=1.0)
vmap.mark_free(10.5, 20.5, 15.0)

# Query occupancy
is_free = vmap.is_free(10.0, 20.0, 15.0)  # Returns True/False
occupancy = vmap.get_occupancy(10.0, 20.0, 15.0)  # Returns 0.0-1.0

# Ray tracing (LiDAR simulation)
vmap.raytrace(
    x_start=0.0, y_start=0.0, z_start=10.0,  # Agent position
    x_end=50.0, y_end=50.0, z_end=10.0        # Ray endpoint
)

# Get statistics
stats = vmap.get_stats()
print(f"Total voxels: {stats['total_voxels']}")
print(f"Occupied: {stats['occupied']}")
print(f"Free: {stats['free']}")
print(f"Memory: {stats['memory_mb']:.2f} MB")

# Export/Import
data = vmap.export_to_dict()
vmap.import_from_dict(data)

# Clear map
vmap.clear()
```

### VoxelMapVisualizer Class

```python
from voxel_map_visualizer import VoxelMapVisualizer

viz = VoxelMapVisualizer(vmap)

# 2D slice visualization
viz.plot_2d_slice(z_slice=20.0, save_path="/tmp/2d_slice.png")

# 3D matplotlib
viz.plot_3d_matplotlib(save_path="/tmp/3d_mpl.png")

# Interactive 3D Plotly
viz.plot_3d_plotly(save_path="/tmp/3d_plotly.html", max_voxels=10000)

# Point cloud (Open3D)
viz.plot_3d_open3d()

# Export
viz.export_to_csv("/tmp/voxel_data.csv")
viz.export_to_json("/tmp/voxel_data.json")
```

## Integration with Agent

The agent automatically processes LiDAR data and updates the voxel map:

```python
from agent import DenddronAgent

# Agent creates and maintains voxel map internally
agent = DenddronAgent(agent_id="drone_1")

# Voxel map is updated when sensor data arrives
# Exports happen automatically every 5 seconds to /tmp/voxel_agent_{agent_id}.json
```

## Data Flow

```
Gazebo Simulator
    ↓
  LiDAR Plugin
    ↓
Sensor Data (pose + rays)
    ↓
  Agent (_process_lidar)
    ↓
Transform to World Frame
    ↓
  Raytrace Update
    ↓
VoxelMap (World Coordinates)
    ↓
Visualization / Export
```

## Visualization Methods

### 1. 2D Slice (Fastest)
- **Use case**: Quick debugging, monitoring from terminal
- **Dependencies**: None (built-in)
- **Performance**: Instant even with huge maps
- **Output**: PNG or matplotlib window

### 2. 3D Matplotlib (Always Available)
- **Use case**: Basic 3D view, no web browser needed
- **Dependencies**: matplotlib (included)
- **Performance**: Good for <50k voxels
- **Output**: PNG or interactive window

### 3. Plotly Interactive 3D (Recommended)
- **Use case**: Explore map interactively in browser
- **Dependencies**: plotly (5.0+)
- **Performance**: Smooth for <100k voxels
- **Output**: Standalone HTML file
- **Features**: 
  - Rotate, zoom, pan
  - Hover for values
  - Color-coded occupancy
  - Share HTML with others

### 4. Open3D (Best Quality)
- **Use case**: Professional visualization, real-time viewer
- **Dependencies**: open3d
- **Performance**: Best for large datasets
- **Output**: Real-time 3D viewer window
- **Features**:
  - High-quality rendering
  - Point cloud tools
  - Coordinate axes shown

## Troubleshooting

### Issue: Plotly not available
```bash
pip install plotly
```

### Issue: Open3D viewer not showing
```bash
pip install open3d
export DISPLAY=:0  # If running remotely
```

### Issue: Voxel map not updating
- Check agent is receiving sensor data: `tail -f /tmp/voxel_agent_*.json`
- Verify JSON contains non-empty voxels
- Check agent logs for errors

### Issue: Export files too large
- Use `--export-csv` for sparse data
- Limit visualizations to specific voxel types (occupied-only)
- Consider 2D slices instead of full 3D

## Performance Notes

| Operation | Time (seconds) | Notes |
|-----------|---|---|
| Raytrace ray | 0.1-0.5ms | Per ray from LiDAR |
| Mark voxel | <0.1ms | Direct memory write |
| 2D visualization | <1s | Up to millions of voxels |
| 3D visualization | 1-10s | Depends on voxel count |
| CSV export | 1-5s | Millions of voxels |
| JSON export | 1-5s | Includes serialization |

## Example: Reading Voxel Commands from Agent Output

```bash
# Watch live voxel map exports
watch -n 1 'cat /tmp/voxel_agent_drone_1.json | python3 -m json.tool | head -20'

# Extract statistics
jq '.stats' /tmp/voxel_agent_drone_1.json

# Count occupied voxels
jq '[.voxels | to_entries[] | select(.value >= 0.5)] | length' /tmp/voxel_agent_drone_1.json
```

## Citation & References

- Probabilistic Occupancy Grids: Khatib & Abril (1999)
- Ray Casting: DDA/Bresenham algorithms
- Visualization: Plotly, Open3D, Matplotlib communities

## License

Part of the DeNDDron Swarm system.
