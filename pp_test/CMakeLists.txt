cmake_minimum_required(VERSION 3.10)
project(PathPlanning)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(USE_CLANG "Build with clang" OFF)

# Check if we want to use Clang
if(USE_CLANG)
    set(CMAKE_C_COMPILER "clang")
    set(CMAKE_CXX_COMPILER "clang++")
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

# Include directories
include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
)

# For GLM, typically you need to specify where to find it
# You may need to adjust this path based on your system
find_package(glm QUIET)
if(NOT glm_FOUND)
    # If GLM is not found through find_package, set include dir manually
    # Uncomment and update this if needed:
    # set(GLM_INCLUDE_DIR "path/to/glm")
    # include_directories(${GLM_INCLUDE_DIR})
endif()

# Source files
set(SOURCES
    single_agent_pp.cpp
    # Add other source files here as needed
)

# Define the executable
add_executable(path_planning ${SOURCES})

# Link libraries
target_link_libraries(path_planning
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    glfw
    # On some systems, you might need to link explicitly with X11, dl, pthread
    # Uncomment as needed:
    # ${X11_LIBRARIES}
    # dl
    # pthread
)

# Add freeGLUT if your code uses glutSolidCube, glutSolidSphere, etc.
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})
target_link_libraries(path_planning ${GLUT_LIBRARIES})

# Compiler-specific options
if(MSVC)
    # MSVC-specific options
    target_compile_options(path_planning PRIVATE /W4)
else()
    # GCC/Clang options
    target_compile_options(path_planning PRIVATE -Wall -Wextra -pedantic)
endif()
