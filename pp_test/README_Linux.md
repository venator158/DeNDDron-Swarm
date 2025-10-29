# 3D Path Planning Simulation Suite - Linux Port

This is a comprehensive 3D OpenGL-based path planning simulation suite, featuring both single-agent and multi-agent scenarios. The system demonstrates various path planning algorithms using the Strategy Pattern.

## Features

### Single-Agent Simulation (`path_planning`)
- 3D visualization using OpenGL and FreeGLUT
- Multiple algorithms: APF, A*, RRT
- Single agent navigation with obstacle avoidance
- Real-time path visualization
- Interactive camera controls

### Multi-Agent Simulation (`multi_agent_planning`)
- **Multi-Agent Path Finding (MAPF)** with simultaneous planning
- **APF MAPF Strategy** - True multi-agent coordination with agent-to-agent repulsion
- **Regular APF Strategy** - Sequential planning for comparison
- Multiple colored agents with individual goals
- Real-time algorithm switching
- Visual comparison of coordination approaches

### Test Suite (`path_planning_test`)
- Comprehensive algorithm testing
- Performance comparison between strategies
- Multi-agent coordination validation

## Quick Start

1. **Install dependencies:**
   ```bash
   # Ubuntu/Debian
   sudo apt update
   sudo apt install build-essential cmake libgl1-mesa-dev libglu1-mesa-dev libglew-dev freeglut3-dev libglm-dev
   ```

2. **Build the project:**
   ```bash
   ./build.sh
   ```

3. **Run the simulations:**
   ```bash
   cd build
   ./path_planning          # Single-agent simulation
   ./multi_agent_planning   # Multi-agent simulation (recommended!)
   ./path_planning_test     # Test suite
   ```

## Multi-Agent Simulation Controls

- **P** - Plan paths for all agents
- **S** - Start agent movement
- **T** - Stop agent movement  
- **A** - Switch between APF MAPF and Regular APF algorithms
- **R** - Reset scenario with new random setup
- **ESC** - Exit the simulation
- **+/-** - Zoom in/out
- **Mouse drag** - Rotate camera

## Single-Agent Simulation Controls

- **ESC** - Exit the simulation
- **R** - Reset the simulation (generates new random obstacles and agent/goal positions)
- **C** - Reset camera to default position
- **+/=** - Zoom in
- **-/_** - Zoom out
- **P** - Trigger path planning (calls your path planning module)
- **S** - Start agent movement (agent follows planned path)
- **T** - Stop agent movement (agent becomes stationary)
- **Left Mouse + Drag** - Rotate camera around the scene

## Visual Indicators
- **Green Sphere** - Agent (bright green when moving, normal green when stationary)
- **Red Sphere** - Goal position
- **Red Cubes** - Obstacles
- **Yellow Line** - Planned path (when path planning is active)
- **Gray Line** - Direct line to goal (when no path is planned)

## Architecture

The code is structured with a clean separation between simulation and path planning:

**Simulation Core:**
- `Agent` - The navigating entity with movement capabilities
- `Obstacle` - Static obstacles in the 3D environment  
- `World` - Manages all entities and simulation logic
- `Camera` - 3D camera system with mouse controls

**Path Planning Interface:**
- `path_planning_interface.h` - Defines the API for your path planning module
- `path_planning_module.cpp` - Stub implementation you can fill in

## Implementing Your Path Planning Module

1. **Edit `path_planning_module.cpp`** - Implement your algorithms in the provided functions
2. **Key Functions to Implement:**
   - `planPath()` - Single agent path planning
   - `planMultiplePaths()` - Multi-agent coordination  
   - `updateDynamicPlanning()` - Real-time replanning
   - `isPathValid()` - Path validation

3. **Algorithm Examples You Can Implement:**
   - A* algorithm for optimal pathfinding
   - RRT (Rapidly-exploring Random Tree) for complex 3D spaces
   - PRM (Probabilistic Roadmap) for preprocessing
   - Custom swarm coordination algorithms

## Next Steps

- Implement collision detection and avoidance
- Add path planning algorithms (A*, RRT, etc.)
- Add multiple agents for swarm behavior
- Implement more sophisticated obstacle configurations

## Build Options

```bash
./build.sh --help    # Show all available options
./build.sh --debug   # Build in debug mode
./build.sh --clang   # Use Clang compiler
./build.sh --clean   # Clean build before building
```

## Troubleshooting

If you encounter missing dependency errors, check `DEPENDENCIES.md` for detailed installation instructions for your Linux distribution.
