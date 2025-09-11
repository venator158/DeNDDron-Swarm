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
find_package(GLUT REQUIRED)

# Find GLM
find_package(glm QUIET)
if(NOT glm_FOUND)
    # Try to find GLM in common locations
    find_path(GLM_INCLUDE_DIR glm/glm.hpp
        PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
    )
    if(GLM_INCLUDE_DIR)
        message(STATUS "Found GLM at: ${GLM_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "GLM not found. Please install libglm-dev (Ubuntu/Debian) or glm-devel (CentOS/RHEL)")
    endif()
endif()

# Include directories
include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
)

# Source files
set(SOURCES
    single_agent_pp.cpp
    path_planning_module.cpp
    # Add other source files here as needed
)

# Define the executable
add_executable(path_planning ${SOURCES})

# Link libraries
target_link_libraries(path_planning
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLUT_LIBRARIES}
    # Linux-specific libraries
    ${CMAKE_DL_LIBS}
    pthread
)

# Compiler-specific options
if(MSVC)
    # MSVC-specific options
    target_compile_options(path_planning PRIVATE /W4)
else()
    # GCC/Clang options
    target_compile_options(path_planning PRIVATE -Wall -Wextra -pedantic)
endif()
