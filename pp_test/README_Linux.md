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
- **ORCA Strategy** - Optimal Reciprocal Collision Avoidance
- **Regular APF Strategy** - Sequential planning for comparison
- **YAML Configuration** - Define agents, obstacles, and world bounds in `config.yaml`
- Multiple colored agents with individual goals
- Real-time algorithm switching
- Visual comparison of coordination approaches
- Support for any number of agents
- Random obstacle generation or explicit positioning

### Test Suite (`path_planning_test`)
- Comprehensive algorithm testing
- Performance comparison between strategies
- Multi-agent coordination validation

## Quick Start

1. **Install dependencies:**
   ```bash
   # Ubuntu/Debian
   sudo apt update
   sudo apt install build-essential cmake libgl1-mesa-dev libglu1-mesa-dev libglew-dev freeglut3-dev libglm-dev libyaml-cpp-dev
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
- **A** - Switch between APF MAPF, ORCA, and Regular APF algorithms
- **F** - Toggle between Real-Time Forces and Waypoint Following
- **R** - Reset scenario and reload configuration from config.yaml
- **C** - Reset camera to default view (centered on world)
- **+/- or Mouse Wheel** - Zoom in/out
- **Left Mouse Drag** - Rotate camera around the scene
- **ESC** - Exit the simulation

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
- **A** - Cycle through the algorithms.

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

## Configuration (Multi-Agent Simulation)

The multi-agent simulation uses a YAML configuration file (`config.yaml`) to define:
- World boundaries
- Obstacles (with optional random generation)
- Agent start/goal positions and properties

### Configuration File Format

Create or edit `config.yaml` in the `pp_test` directory:

```yaml
world:
  bounds:
    min: [-20.0, -20.0, -5.0]  # World minimum corner [x, y, z]
    max: [20.0, 20.0, 5.0]     # World maximum corner [x, y, z]

obstacles:
  count: 5                       # Total number of obstacles
  default_size: [2.0, 2.0, 2.0]  # Default size [x, y, z]
  positions:                     # Optional: specific obstacle placements
    - center: [5.0, 5.0, 0.0]
      size: [2.0, 2.0, 2.0]
    - center: [8.0, 3.0, 0.0]
      size: [2.0, 2.0, 2.0]
  # If count > positions.length, remaining obstacles are randomly generated

agents:
  default_radius: 0.5            # Default agent radius
  default_speed: 2.0             # Default agent speed
  
  agent_list:
    - id: 0
      start: [0.0, 0.0, 0.0]     # Starting position [x, y, z]
      goal: [10.0, 10.0, 0.0]    # Goal position [x, y, z]
      color: [0.0, 1.0, 0.0]     # RGB color (0.0-1.0)
      radius: 0.5                # Optional: override default
      speed: 2.0                 # Optional: override default
      
    - id: 1
      start: [10.0, 0.0, 0.0]
      goal: [0.0, 10.0, 0.0]
      color: [0.0, 0.5, 1.0]
    
    # Add as many agents as needed!
```
# Running the python script for generation of agents/obstacles
## if 10 agents with existing obstacles
```bash
python3 generate_random_agents.py -n 10
```
## if 20 agents with 15 obstacles 
```bash
python3 generate_random_agents.py -n 20 -o 15
```
so and so forth..

### Notes:
- You can define **any number of agents** in the configuration
- Obstacles can be explicitly positioned or randomly generated
- Press **'R'** to reload the configuration file during runtime
- The simulation will fail to start if `config.yaml` is missing or invalid


