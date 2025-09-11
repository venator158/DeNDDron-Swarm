# Path Planning Module Integration Guide

This document explains how to implement your path planning algorithms in the provided framework.

## Quick Start

1. **The agents are spawned and stationary by default**
2. **Press 'P' to trigger path planning** (calls your module)
3. **Press 'S' to start agent movement** (follows the planned path)
4. **Press 'T' to stop agent movement**

## Files You Need to Modify

### `path_planning_module.cpp`
This is where you implement your algorithms. Key functions:

```cpp
std::vector<glm::vec3> planPath(const glm::vec3& start, 
                               const glm::vec3& goal, 
                               const std::vector<Obstacle*>& obstacles)
```
- **Input**: Start position, goal position, list of obstacles
- **Output**: Vector of 3D waypoints (empty if no path found)
- **Implement**: Your pathfinding algorithm (A*, RRT, etc.)

## Algorithm Implementation Examples

### A* Algorithm
```cpp
// Pseudo-code for A* implementation
std::vector<glm::vec3> planPath(const glm::vec3& start, const glm::vec3& goal, const std::vector<Obstacle*>& obstacles) {
    // 1. Create 3D grid or graph
    // 2. Initialize open and closed sets
    // 3. Add start node to open set
    // 4. While open set is not empty:
    //    - Get node with lowest f-score
    //    - If it's the goal, reconstruct path
    //    - Move to closed set
    //    - Check all neighbors
    //    - Calculate g, h, and f scores
    // 5. Return path or empty vector
}
```

### RRT Algorithm
```cpp
// Pseudo-code for RRT implementation
std::vector<glm::vec3> planPath(const glm::vec3& start, const glm::vec3& goal, const std::vector<Obstacle*>& obstacles) {
    // 1. Initialize tree with start node
    // 2. For max_iterations:
    //    - Sample random point in 3D space
    //    - Find nearest node in tree
    //    - Extend towards random point
    //    - Check for collision with obstacles
    //    - Add new node if collision-free
    //    - Check if close to goal
    // 3. Reconstruct path from goal to start
}
```

## 3D Collision Detection

You'll need to implement collision checking between paths and 3D obstacles:

```cpp
bool isCollisionFree(const glm::vec3& point, const std::vector<Obstacle*>& obstacles) {
    for (const auto& obstacle : obstacles) {
        glm::vec3 obstaclePos = obstacle->getPosition();
        glm::vec3 obstacleScale = obstacle->getScale();
        
        // Check if point is inside the obstacle bounding box
        if (point.x >= obstaclePos.x - obstacleScale.x/2 && 
            point.x <= obstaclePos.x + obstacleScale.x/2 &&
            point.y >= obstaclePos.y - obstacleScale.y/2 && 
            point.y <= obstaclePos.y + obstacleScale.y/2 &&
            point.z >= obstaclePos.z - obstacleScale.z/2 && 
            point.z <= obstaclePos.z + obstacleScale.z/2) {
            return false; // Collision detected
        }
    }
    return true; // No collision
}
```

## Configuration

You can adjust path planning parameters in the `Config` structure:

```cpp
PathPlanning::Config config;
config.agentRadius = 0.5f;      // Agent size for collision checking
config.stepSize = 1.0f;         // Path resolution
config.maxIterations = 1000;    // Algorithm termination
config.goalTolerance = 0.5f;    // How close to goal is "reached"
PathPlanning::setConfig(config);
```

## Testing Your Implementation

1. **Build**: `./build.sh`
2. **Run**: `cd build && ./path_planning`
3. **Test Controls**:
   - **R** - Generate new random scenario
   - **P** - Trigger your path planning
   - **S** - Start movement
   - **Mouse** - Orbit camera to inspect the path

