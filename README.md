# DeNDDron-Swarm
Codebase for DeNDDron Swarm

## Projects

### 3D Navigation Test (`pp_test/`)
A 3D OpenGL-based agent navigation simulation, now ported to Linux.


**Quick Start (Linux):**
```bash
cd pp_test
./build.sh
cd build && ./path_planning
```

See `pp_test/README_Linux.md` for detailed instructions.


## Architecture
The project follows modular design principles with the Strategy Pattern for path planning algorithms, making it extensible for future swarm robotics implementations.
