# Dependencies Installation Guide

## Required Libraries:
1. OpenGL - Usually comes with your graphics drivers
2. GLEW - GL Extension Wrangler Library
3. GLM - OpenGL Mathematics
4. FreeGLUT - For glutSolidCube, glutSolidSphere functions
5. yaml-cpp - YAML configuration file parser

---

## Linux Installation

### Ubuntu/Debian Systems

```bash
# Update package list
sudo apt update

# Install development tools
sudo apt install build-essential cmake

# Install OpenGL and graphics libraries
sudo apt install libgl1-mesa-dev libglu1-mesa-dev

# Install GLEW
sudo apt install libglew-dev

# Install FreeGLUT
sudo apt install freeglut3-dev

# Install GLM (OpenGL Mathematics)
sudo apt install libglm-dev

# Install yaml-cpp (for configuration file support)
sudo apt install libyaml-cpp-dev

# Optional: Install Clang if you want to use it instead of GCC
sudo apt install clang
```

### CentOS/RHEL/Fedora Systems

```bash
# For CentOS/RHEL (with EPEL repository enabled)
sudo yum install gcc-c++ cmake
sudo yum install mesa-libGL-devel mesa-libGLU-devel
sudo yum install glew-devel
sudo yum install freeglut-devel
sudo yum install glm-devel
sudo yum install yaml-cpp-devel

# For Fedora
sudo dnf install gcc-c++ cmake
sudo dnf install mesa-libGL-devel mesa-libGLU-devel
sudo dnf install glew-devel
sudo dnf install freeglut-devel
sudo dnf install glm-devel
sudo dnf install yaml-cpp-devel
```

### Arch Linux

```bash
sudo pacman -S base-devel cmake
sudo pacman -S mesa glu
sudo pacman -S glew
sudo pacman -S freeglut
sudo pacman -S glm
sudo pacman -S yaml-cpp
```

### Building the Project

```bash
# Make the build script executable
chmod +x build.sh

# Build in Release mode (default)
./build.sh

# Build in Debug mode
./build.sh --debug

# Build with Clang instead of GCC
./build.sh --clang

# Clean build and rebuild
./build.sh --clean

# Verbose output
./build.sh --verbose

# Show help
./build.sh --help
```

### Running the Simulation

```bash
cd build
./path_planning
```

### Controls:
- **ESC** - Exit the simulation
- **R** - Reset the simulation
- **C** - Reset camera to default position
- **+/=** - Zoom in
- **-/_** - Zoom out
- **Left Mouse + Drag** - Rotate camera around the scene

---

## Windows Installation (Legacy)

## Installation using VCPKG (Recommended)

VCPKG is a package manager for C++ libraries that makes installing dependencies straightforward.

### 1. Install VCPKG

```powershell
# Clone VCPKG repository
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg

# Run the bootstrap script
.\bootstrap-vcpkg.bat

# Integrate with Visual Studio (if you're using it)
.\vcpkg integrate install
```

### 2. Install Required Packages

```powershell
# Install packages (32-bit)
.\vcpkg install glew:x86-windows
.\vcpkg install glfw3:x86-windows
.\vcpkg install glm:x86-windows
.\vcpkg install freeglut:x86-windows

# Or for 64-bit
.\vcpkg install glew:x64-windows
.\vcpkg install glfw3:x64-windows
.\vcpkg install glm:x64-windows
.\vcpkg install freeglut:x64-windows
```

### 3. Using with CMake

When using CMake with VCPKG, use the toolchain file:

```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake
```

## Alternative: Use Pre-built Binaries

You can also download pre-built binaries manually:

1. GLEW: http://glew.sourceforge.net/
2. GLFW: https://www.glfw.org/download.html
3. GLM: https://github.com/g-truc/glm/releases
4. FreeGLUT: https://www.transmissionzero.co.uk/software/freeglut-devel/

Then configure your CMake file to find these libraries in your specified locations.

## Building with Clang on Windows

To use Clang with this CMake configuration:

```powershell
# Install Clang (if not already installed)
# You can get it from LLVM releases: https://releases.llvm.org/

# Then build with CMake using Clang
cmake -B build -S . -DUSE_CLANG=ON
cmake --build build
```

## Troubleshooting

If you encounter "library not found" errors:
1. Check that all libraries are properly installed
2. Update the CMakeLists.txt to point to the correct directories
3. For manual installs, set environment variables:
   - Add DLL directories to PATH
   - Set CMAKE_PREFIX_PATH to include your library installation paths
