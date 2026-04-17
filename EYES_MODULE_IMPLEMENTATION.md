# Eyes Module Implementation - Obstacle Perception

## Overview
The **Eyes** module (PILLAR 1) is now implemented for real-time obstacle perception and mapping. It processes LiDAR data from Gazebo and maintains a probabilistic 3D occupancy voxel map for dynamic path planning.

## Architecture

### 1. VoxelMap Class (`src/agent/voxel_map.py`)
A sparse, memory-efficient 3D occupancy grid system.

#### Design Characteristics
- **Coordinate System**: World-absolute (not agent-relative)
- **Bounds**:
  - X: [-1000, 1000] meters
  - Y: [-1000, 1000] meters
  - Z: [0, 100] meters
- **Resolution**: 0.5 meters per voxel
- **Storage**: Sparse dictionary keyed by (vx, vy, vz) tuples
  - Only occupied/free voxels stored
  - Unknown voxels default to 0.0 (free)

#### Probabilistic Occupancy Model
- **Range**: 0.0 (free) to 1.0 (occupied)
- **Update Rule**: `occupancy = max(current, new_confidence)`
  - Takes maximum of existing and new evidence
  - LiDAR detections: confidence = 1.0 (certain)
- **Query Threshold**: Default 0.5
  - `is_free()` returns True if occupancy < 0.5

#### Core Methods
```python
mark_occupied(x, y, z, confidence=1.0)  # Mark voxel as occupied
mark_free(x, y, z)                       # Mark voxel as free
is_free(x, y, z, threshold=0.5)         # Query if free
get_occupancy(x, y, z)                  # Get occupancy probability
raytrace(x_start, y_start, z_start,    # Cast ray: free space + occupied endpoint
         x_end, y_end, z_end)
get_stats()                              # Return map statistics
clear()                                  # Clear all voxels
```

#### Ray Casting Algorithm
- **Simple 3D Bresenham raytrace**
- **Intermediate voxels**: Marked as free (ray passes through)
- **Endpoint voxel**: Marked as occupied with confidence 1.0
- **Steps**: Interpolation with `steps = max(|Δx|, |Δy|, |Δz|) + 1`

### 2. Agent Integration (`src/agent/agent.py`)

#### Sensor Data Pipeline
```
Gazebo Simulator
    ↓ (drone/{agent_id}/sensors topic)
    ├─ pose: {x, y, z, yaw, ...}
    └─ lidar: [{angle, distance, intensity, ray_id}, ...]
    ↓
Agent._on_sensor_data()
    ↓ (parse JSON)
  current_pose = {...}
    ↓
Agent._process_lidar()
    ├─ For each ray:
    │  ├─ Transform to world frame (robot yaw + ray angle)
    │  ├─ Ray endpoint: (x + distance*cos(angle), y + distance*sin(angle), z)
    │  └─ Raytrace to update voxel map
    ↓
VoxelMap.raytrace()
    ├─ Mark intermediate voxels as free
    └─ Mark endpoint as occupied
    ↓
[Voxel Map Ready for Path Planner Queries]
```

#### Coordinate Transformation
- **Robot frame** → **World frame**
  - Ray angle in robot: horizontal bearing relative to drone
  - World angle: `yaw + angle` (robot yaw + relative bearing)
  - Ray endpoint in world: `(x + d*cos(world_angle), y + d*sin(world_angle), z)`
  - Assumes horizontal LiDAR at drone center height (z coordinate unchanged)

#### Error Handling
- Invalid rays (distance ≤ 0 or > 100m) are skipped
- Voxels outside map bounds are rejected
- Try-catch blocks ensure exceptions don't crash sensor processing
- Light logging at DEBUG level: `logger.debug()` for map stats

### 3. Data Flow

```
Gazebo (50 Hz)
  ↓
Publish sensor data (drone/{agent_id}/sensors)
  ↓
Agent receives sample
  ↓
_on_sensor_data():
  - Decode JSON
  - Extract pose and lidar array
  - Store current_pose
  - Call _process_lidar()
  ↓
_process_lidar():
  - Transform rays to world coords
  - For each ray: raytrace()
  ↓
VoxelMap.raytrace():
  - Mark free space along ray
  - Mark endpoint as occupied
  ↓
VoxelMap persists and accumulates
  ↓
Path Planner queries: is_free(x, y, z)?
```

## Configuration

### Map Parameters (in VoxelMap)
```python
MIN_X, MAX_X = -1000.0, 1000.0
MIN_Y, MAX_Y = -1000.0, 1000.0
MIN_Z, MAX_Z = 0.0, 100.0
RESOLUTION = 0.5  # meters per voxel
```

### LiDAR Parameters (from Gazebo simulation)
```cpp
// In GazeboSimulator.cpp - simulate_lidar()
- Rays: 16
- Angle distribution: Full 2π radians (360°)
- Distance: Constant 50.0 meters (placeholder)
- Update rate: 50 Hz (published via `publish_sensor_data()`)
```

## Performance Characteristics

### Memory Usage
- Sparse storage only stores occupied/free voxels
- Per voxel: ~32 bytes (dictionary overhead)
- Typical scenario (~500-1000 voxels): 15-30 MB
- Map statistics available via `get_stats()`

### Computation
- Per ray: O(num_voxels_along_ray) = O(200) at most
- Per LiDAR frame (16 rays): O(3200) voxel updates
- At 50 Hz: ~160,000 voxel operations/sec (acceptable)

### Query Latency
- `is_free()` lookup: O(1) dictionary access
- Path planner can query in real-time

## Testing & Validation

### Light Logging
Integrated at DEBUG level:
```python
logger.debug(f"[{self.agent_id}] Voxel map: {self.voxel_map.get_stats()}")
```

Outputs:
- `total_voxels`: Count of stored voxels
- `occupied`: Count of occupied voxels (occupancy ≥ 0.5)
- `free`: Count of free voxels (occupancy < 0.5)
- `memory_mb`: Rough memory estimate

### Enable Debug Logging
To see Eyes module activity:
```python
logging.basicConfig(level=logging.DEBUG)  # Instead of INFO
```

## Integration with Path Planner

The path planner can now query the voxel map:

```python
# In path planner (future implementation):
if agent.voxel_map.is_free(x, y, z):
    # Safe to move to this location
else:
    # Obstacle detected, avoid this location
```

## Future Enhancements

1. **Occupancy Decay** (aging): Reduce confidence of old observations
2. **Consensus-based Probabilities**: Incorporate shared agent locations from consensus mechanism
3. **Advanced Ray Casting**: Angle-dependent confidence based on LiDAR uncertainty
4. **Downsampling**: Efficient spatial queries with octree or k-d tree
5. **Visualization**: Export voxel map for debugging/validation

## Files Modified

- **Created**: `src/agent/voxel_map.py` (VoxelMap class)
- **Modified**: `src/agent/agent.py` (Eyes integration, LiDAR processing)

## Next Steps

1. Implement **Path Planner** to query voxel map
2. Implement **Reflexes** (APF) using voxel map for repulsive forces
3. Implement **Consensus mechanism** for location sharing
4. Run end-to-end swarm simulation with obstacle avoidance
