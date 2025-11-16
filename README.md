# DeNDDron-Swarm
Codebase for DeNDDron Swarm

## Projects

### 3D Navigation Test (`pp_test/`)
A 3D OpenGL-based agent navigation simulation, now ported to Linux with YAML-based configuration.

**Features:**
- Single-agent and multi-agent path planning
- Multiple algorithms: APF, APF-MAPF, ORCA, A*, RRT
- YAML configuration for flexible scenario design
- Real-time visualization and algorithm switching
- Comprehensive MAPF metrics (makespan, sum of costs, collisions, efficiency)

**Quick Start (Linux):**
```bash
# Install dependencies (including yaml-cpp)
sudo apt install build-essential cmake libgl1-mesa-dev libglu1-mesa-dev \
                 libglew-dev freeglut3-dev libglm-dev libyaml-cpp-dev

# Build and run
cd pp_test
./build.sh
cd build && ./multi_agent_planning  # Multi-agent (uses config.yaml)
# or
./path_planning  # Single agent
```

See `pp_test/README_Linux.md` for detailed instructions.  
See `pp_test/CONFIG_REFERENCE.md` for YAML configuration guide.

**Metrics:**
- Automatically calculated when all agents reach goals
- Essential: Makespan, Sum of Costs, Planning Time, Success Rate
- Quality: Path Efficiency, Smoothness, Collisions, Near Misses
- Results saved to `metrics/` folder with timestamps and algorithm names
- Aggregate results in `metrics/all_metrics.csv` for comparison


## Architecture
The project follows modular design principles with the Strategy Pattern for path planning algorithms, making it extensible for future swarm robotics implementations.
