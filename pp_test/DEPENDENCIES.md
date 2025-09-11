# Dependencies Installation Guide for Windows

## Required Libraries:
1. OpenGL - Usually comes with your graphics drivers
2. GLEW - GL Extension Wrangler Library
3. GLFW - Graphics Library Framework
4. GLM - OpenGL Mathematics
5. FreeGLUT - For glutSolidCube, glutSolidSphere functions

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
