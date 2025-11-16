#!/bin/bash

# Bash script to build the project on Linux

# Default values
USE_CLANG=false
BUILD_TYPE="Release"
CLEAN=false
VERBOSE=false

# Function to show usage
show_usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -c, --clang      Use Clang compiler instead of GCC"
    echo "  -d, --debug      Build in Debug mode (default: Release)"
    echo "  -r, --release    Build in Release mode"
    echo "  --clean          Clean build directory before building"
    echo "  -v, --verbose    Verbose build output"
    echo "  -h, --help       Show this help message"
    exit 1
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clang)
            USE_CLANG=true
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_usage
            ;;
        *)
            echo "Unknown option $1"
            show_usage
            ;;
    esac
done

echo "Building Path Planning Simulation for Linux"
echo "Build type: $BUILD_TYPE"

# Check if dependencies are installed
echo "Checking dependencies..."
missing_deps=()

# Check for required packages
if ! dpkg -l | grep -q libglew-dev && ! rpm -q glew-devel &>/dev/null; then
    missing_deps+=("GLEW development libraries")
fi

if ! dpkg -l | grep -q freeglut3-dev && ! rpm -q freeglut-devel &>/dev/null; then
    missing_deps+=("FreeGLUT development libraries")
fi

if ! dpkg -l | grep -q libglm-dev && ! rpm -q glm-devel &>/dev/null; then
    missing_deps+=("GLM mathematics library")
fi

if ! dpkg -l | grep -q libgl1-mesa-dev && ! rpm -q mesa-libGL-devel &>/dev/null; then
    missing_deps+=("OpenGL development libraries")
fi

if [ ${#missing_deps[@]} -ne 0 ]; then
    echo "Missing dependencies:"
    for dep in "${missing_deps[@]}"; do
        echo "  - $dep"
    done
    echo
    echo "Please install dependencies first. See DEPENDENCIES.md for instructions."
    exit 1
fi

echo "All dependencies found!"

# Create build directory
if [ ! -d "build" ]; then
    mkdir build
fi

# Clean build if requested
if [ "$CLEAN" = true ] && [ -d "build" ]; then
    echo "Cleaning build directory..."
    rm -rf build/*
fi

# Build CMake command
CMAKE_ARGS="-B build -S . -DCMAKE_BUILD_TYPE=$BUILD_TYPE"

# Add Clang option if requested
if [ "$USE_CLANG" = true ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DUSE_CLANG=ON"
    echo "Using Clang compiler"
fi

# Run CMake configure
echo "Configuring project..."
if [ "$VERBOSE" = true ]; then
    echo "CMake command: cmake $CMAKE_ARGS"
fi

cmake $CMAKE_ARGS

if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# Build the project
echo "Building project in $BUILD_TYPE mode..."
if [ "$VERBOSE" = true ]; then
    cmake --build build --config $BUILD_TYPE -- -j$(nproc)
else
    cmake --build build --config $BUILD_TYPE -- -j$(nproc) > /dev/null
fi

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Executables:"
    echo "  build/path_planning (Single-agent simulation)"
    echo "  build/multi_agent_planning (Multi-agent simulation)"
    echo "  build/path_planning_test (Test suite)"
    echo
    echo "To run the simulations:"
    echo "  cd build && ./path_planning          # Single-agent APF/A*/RRT"
    echo "  cd build && ./multi_agent_planning   # Multi-agent APF MAPF vs APF and ORCA"
    echo "  cd build && ./path_planning_test     # Test algorithms"
else
    echo "Build failed!"
    exit 1
fi
