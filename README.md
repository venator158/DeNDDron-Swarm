# DeNDDron-Swarm
Codebase for DeNDDron Swarm

## Projects

### 3D Navigation Test (`pp_test/`)
A 3D OpenGL-based agent navigation simulation, now ported to Linux.

**Features:**
- Direct 3D agent movement towards goals
- 3D visualization with OpenGL and FreeGLUT
- Real-time agent movement in 3D space
- Interactive obstacle environment in 3D
- Mouse-controlled camera system (orbit, zoom)
- Keyboard controls for simulation reset and camera management

**Quick Start (Linux):**
```bash
cd pp_test
./build.sh
cd build && ./path_planning
```

See `pp_test/README_Linux.md` for detailed instructions.

## Platform Support
- ✅ Linux (primary)
- ⚠️ Windows (legacy support in `pp_test/`)

## Architecture
The project follows modular design principles with the Strategy Pattern for path planning algorithms, making it extensible for future swarm robotics implementations.
