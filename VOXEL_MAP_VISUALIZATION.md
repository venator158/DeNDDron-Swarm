# VoxelMap Visualization Toolkit

## Overview

Complete toolkit for visualizing and analyzing 3D probabilistic occupancy grids in real-time and post-processing.

**Components:**
1. **VoxelMap** - Core sparse 3D occupancy grid (src/agent/voxel_map.py)
2. **VoxelMapVisualizer** - Multi-backend visualization (src/agent/voxel_map_visualizer.py)
3. **test_voxel_map.py** - Synthetic data generation and testing
4. **record_voxel_map.py** - Live recording and post-analysis

---

## Quick Start

### Generate Test Data & Visualize (2D Slice)

```bash
cd src/agent
python3 test_voxel_map.py --plot 2d --positions 20 --save /tmp/voxel_2d.png
```

Output:
- Generates 20 synthetic agent positions moving in a circle
- Creates obstacles at origin (ship) and 2 additional points
- Shows 2D top-down slice at z=20m
- Saves visualization to `/tmp/voxel_2d.png`

### Interactive 3D Visualization (Plotly)

```bash
# First install plotly
pip install plotly

# Generate visualization
python3 test_voxel_map.py --plot 3d_plotly --positions 15

# Open in browser
open index.html
```

Features:
- ✓ Rotate, zoom, pan in 3D
- ✓ Hover for occupancy values
- ✓ Color-coded occupancy from 0.0 (blue) to 1.0 (red)
- ✓ Statistics displayed in title

### Point Cloud Visualization (Open3D)

```bash
# Install open3d
pip install open3d

# Visualize
python3 test_voxel_map.py --plot open3d --positions 15
```

---

## Visualization Methods

### 1. 2D Slice (Matplotlib)

**Dependencies:** Built-in (Python 3.7+)

**Method:** `VoxelMapVisualizer.plot_2d_slice(z_slice=20.0, save_path=None)`

**Features:**
- Top-down view at specified Z height
- Occupied voxels: red circles
- Free voxels: light blue dots
- Map boundaries shown
- Statistics box overlay
- Zoom and pan support (Matplotlib GUI)

**Example:**
```python
from voxel_map_visualizer import VoxelMapVisualizer

visualizer = VoxelMapVisualizer(voxel_map)
visualizer.plot_2d_slice(z_slice=20.0, save_path="/tmp/slice.png")
```

### 2. 3D Matplotlib

**Dependencies:** Built-in (Python 3.7+)

**Method:** `VoxelMapVisualizer.plot_3d_matplotlib(save_path=None)`

**Features:**
- 3D scatter plot
- Basic 3D navigation (rotation, zoom)
- Occupied voxels: red
- Free voxels (subsampled): light blue
- Max 5000 voxels displayed for performance

**Limitations:**
- Limited interactivity
- Dense point clouds slow to render

**Example:**
```python
visualizer.plot_3d_matplotlib(save_path="/tmp/3d.png")
```

### 3. 3D Plotly (Interactive Web)

**Dependencies:** `pip install plotly`

**Method:** `VoxelMapVisualizer.plot_3d_plotly(save_path=None, max_voxels=10000)`

**Features:**
- ✓ Fully interactive (rotate, zoom, pan, hover)
- ✓ Color-coded occupancy (color scale 0.0-1.0)
- ✓ Occupancy values on hover
- ✓ Statistics in title
- ✓ HTML output (standalone, no server needed)
- ✓ Up to 10,000 voxels displayed

**Example:**
```python
visualizer.plot_3d_plotly(save_path="index.html", max_voxels=10000)
```

Then open `index.html` in the project root in any web browser.

### 4. Point Cloud (Open3D)

**Dependencies:** `pip install open3d`

**Method:** `VoxelMapVisualizer.plot_3d_open3d()`

**Features:**
- Real-time 3D viewer window
- Highest quality visualization
- Occupied voxels: red
- Free voxels (subsampled): light blue
- Full control: rotate, zoom, pan, screenshot

**Example:**
```python
visualizer.plot_3d_open3d()  # Opens interactive window
```

---

## Export Options

### Export to CSV

```python
visualizer.export_to_csv("/tmp/voxel_map.csv")
```

**Output format:**
```
x,y,z,occupancy,type
0.5,0.5,20.0,1.0,occupied
-0.5,0.5,20.0,0.0,free
...
```

**Use cases:**
- Import to Excel or visualization tools
- Analysis in pandas/numpy
- External rendering

### Export to JSON

```python
visualizer.export_to_json("/tmp/voxel_map.json")
```

**Output:**
```json
{
  "voxels": {
    "(0, 1, 40)": 1.0,
    "(1, 1, 40)": 0.0,
    ...
  },
  "bounds": {
    "x": [-1000.0, 1000.0],
    "y": [-1000.0, 1000.0],
    "z": [0.0, 100.0]
  },
  "resolution": 0.5,
  "stats": {
    "total_voxels": 12390,
    "occupied": 144,
    "free": 12246,
    "memory_mb": 0.38
  }
}
```

---

## Test Script Usage

### test_voxel_map.py

Comprehensive test with synthetic LiDAR and agent trajectory.

**Simulates:**
- Agent moving in circular path (60m radius, 20m altitude)
- Ship obstacle at origin (15m radius)
- 2 additional obstacles
- 16-ray LiDAR at 50m max range

**Usage:**

```bash
# Basic: 2D slice visualization (15 positions)
python3 test_voxel_map.py --plot 2d --positions 15

# 3D matplotlib
python3 test_voxel_map.py --plot 3d_mpl --positions 20

# Interactive Plotly (requires plotly)
python3 test_voxel_map.py --plot 3d_plotly --positions 30

# Generate all visualizations
python3 test_voxel_map.py --plot all --positions 25

# Export data
python3 test_voxel_map.py --export-csv /tmp/map.csv --export-json /tmp/map.json
```

**Parameters:**
- `--plot {2d, 3d_mpl, 3d_plotly, open3d, all}` - Visualization backend
- `--save PATH` - Save to file (HTML for plotly, PNG for others)
- `--export-csv PATH` - Export occupancy data to CSV
- `--export-json PATH` - Export full map to JSON
- `--positions N` - Number of agent positions to simulate (default: 20)

**Output Example:**
```
======================================================================
VoxelMap Test & Visualization
======================================================================
[Simulator] Simulating 20 agent positions...
  Position 5/20: Voxels: 6867, Occupied: 80, Memory: 0.21 MB
  Position 10/20: Voxels: 13580, Occupied: 160, Memory: 0.41 MB
  Position 20/20: Voxels: 24156, Occupied: 320, Memory: 0.74 MB
[Simulator] Done!

[VoxelMap Stats]
  Total voxels: 24156
  Occupied: 320
  Free: 23836
  Memory: 0.74 MB

[Visualization]
  Generating 2D slice...
  ✓ Saved 2D plot to /tmp/voxel_test_2d.png
  Generating 3D matplotlib plot...
  ✓ Saved 3D plot to /tmp/voxel_test_3d_mpl.png
  Generating interactive 3D Plotly plot...
  ✓ Saved to /tmp/voxel_test_3d_plotly.html

======================================================================
Test complete!
======================================================================
```

---

## Recording During Live Swarm

### record_voxel_map.py

Record and visualize voxel maps during actual swarm operation.

**Recording Mode:**
```bash
# Start recording agent drone_1's voxel map (autosave every 10 seconds)
python3 record_voxel_map.py --agent drone_1 --interval 10

# Snapshots saved to: /tmp/voxel_agent_drone_1.json
```

**Visualization Mode (Analyze Recorded Data):**
```bash
# Visualize last recorded snapshot
python3 record_voxel_map.py --visualize --agent drone_1 --plot 3d_mpl

# Or specify exact snapshot file
python3 record_voxel_map.py --visualize --input /tmp/voxel_agent_drone_1.json --plot 2d

# Export recorded data
python3 record_voxel_map.py --visualize --input /tmp/voxel_agent_drone_1.json \
  --export-csv /tmp/map_data.csv --export-json /tmp/map_data.json
```

**Features:**
- Automatic periodic snapshots during swarm operation
- JSON-based storage (voxel map state + metadata)
- Post-hoc visualization and analysis
- Export to CSV/JSON for external tools

---

## Integration with Live Agent

### In-Memory Access During Runtime

```python
# In agent.py or path planner:
from voxel_map_visualizer import VoxelMapVisualizer

visualizer = VoxelMapVisualizer(agent.voxel_map)

# Periodic visualization (e.g., every 10 seconds)
if step % 500 == 0:  # 50Hz * 10s = 500 steps
    visualizer.plot_2d_slice(z_slice=agent.current_pose['z'])
```

### Export Mid-Simulation

```python
# Periodically save state for analysis
if step % 1000 == 0:
    visualizer.export_to_json(f"/tmp/agent_{agent_id}_step_{step}.json")
```

---

## Performance Characteristics

### Memory Usage Per Voxel
- Base dictionary entry: ~32 bytes
- Example: 20,000 voxels = ~625 KB

### Rendering Performance
- 2D Slice: < 100ms
- 3D Matplotlib: 500ms - 2s (depends on density)
- 3D Plotly: 1-3s (HTML generation)
- Open3D: 500ms - 1s (viewer launch)

### Voxel Limits
- Safe for visualization: < 50,000 voxels
- Max recommended: 100,000 voxels (sparse grid)

---

## Troubleshooting

### "Matplotlib required for 2D visualization"
```bash
pip install matplotlib
```

### "Plotly required for interactive 3D"
```bash
pip install plotly
```

### "Open3D required for point cloud visualization"
```bash
pip install open3d
```

### Large memory usage?
- Check voxel density: `voxel_map.get_stats()`
- Limit rays processed per frame
- Reduce resolution (currently 0.5m)
- Use sparse export (CSV/JSON) instead of keeping in memory

### Visualization window won't open?
- Check display settings: `echo $DISPLAY`
- For remote: enable X11 forwarding or use Plotly (HTML-based)

---

## Files Created

- `src/agent/voxel_map_visualizer.py` - Main visualization library
- `src/agent/test_voxel_map.py` - Synthetic test and demo script
- `src/agent/record_voxel_map.py` - Live recording and analysis tool
- `VOXEL_MAP_VISUALIZATION.md` - This documentation

---

## Quick Reference

| Task | Command |
|------|---------|
| Quick 2D test | `python3 test_voxel_map.py --plot 2d` |
| Interactive 3D | `python3 test_voxel_map.py --plot 3d_plotly` |
| Point cloud viewer | `python3 test_voxel_map.py --plot open3d` |
| Export data | `python3 test_voxel_map.py --export-csv map.csv` |
| Record live | `python3 record_voxel_map.py --agent drone_1` |
| Visualize recording | `python3 record_voxel_map.py --visualize --plot 3d_mpl` |

---

## Next Steps

1. **Integrate into live swarm:**
   - Add visualizer instantiation to agent initialization
   - Periodic snapshots during operation
   - Export data for post-run analysis

2. **Real-time dashboard:**
   - Web-based viewer for multiple agents
   - Live updates via WebSocket
   - Swarm-wide occupancy overlay

3. **Analysis tools:**
   - Detection accuracy metrics
   - Computational efficiency profiling
   - Map coverage analysis

4. **Enhanced visualization:**
   - Agent position and trajectory overlay
   - LiDAR ray visualization
   - Consensus-based uncertainties (when implemented)
