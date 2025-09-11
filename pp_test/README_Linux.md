# 3D Agent Navigation Simulation - Linux Port

This is a 3D OpenGL-based agent navigation simulation, ported to Linux. The simulation demonstrates direct agent movement in 3D space towards goals while avoiding obstacles.

## Features

- 3D visualization using OpenGL and FreeGLUT
- Single agent (green sphere) direct navigation to goal
- Random 3D obstacle generation (red cubes)
- Real-time goal visualization (red sphere)
- Interactive camera controls
- 3D environment with grid boundariesing Simulation - Linux Port

This is a 3D OpenGL-based path planning simulation for a single agent, ported to Linux. The simulation demonstrates the Strategy Pattern for path planning algorithms.

## Features

- 3D visualization using OpenGL and FreeGLUT
- Single agent (green sphere) navigation
- Random obstacle generation (red cubes)
- Real-time path visualization (yellow lines)
- Goal visualization (red sphere)
- Interactive controls

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

3. **Run the simulation:**
   ```bash
   cd build
   ./path_planning
   ```

## Controls

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
