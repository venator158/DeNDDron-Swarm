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

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

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
    ${YAML_CPP_INCLUDE_DIR}
)

# Source files for single agent simulation
set(SINGLE_AGENT_SOURCES
    single_agent_pp.cpp
    path_planning_module.cpp
    strategies/rrt_strategy.cpp
    strategies/astar_strategy.cpp
    strategies/apf_strategy.cpp
    strategies/apf_mapf_strategy.cpp
    strategies/orca_mapf_strategy.cpp
)

# Source files for multi agent simulation
set(MULTI_AGENT_SOURCES
    multi_agent_pp.cpp
    path_planning_module.cpp
    config_loader.cpp
    mapf_metrics.cpp
    strategies/rrt_strategy.cpp
    strategies/astar_strategy.cpp
    strategies/apf_strategy.cpp
    strategies/apf_mapf_strategy.cpp
    strategies/orca_mapf_strategy.cpp
)

# Create a library for the path planning system
add_library(path_planning_lib 
    path_planning_module.cpp
    strategies/rrt_strategy.cpp
    strategies/astar_strategy.cpp
    strategies/apf_strategy.cpp
    strategies/apf_mapf_strategy.cpp
    strategies/orca_mapf_strategy.cpp
)

# Define the single agent executable
add_executable(path_planning ${SINGLE_AGENT_SOURCES})

# Define the multi agent executable
add_executable(multi_agent_planning ${MULTI_AGENT_SOURCES})

# Create the test executable
add_executable(path_planning_test test_main.cpp)

# Link libraries for single agent executable
target_link_libraries(path_planning
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLUT_LIBRARIES}
    # Linux-specific libraries
    ${CMAKE_DL_LIBS}
    pthread
)

# Link libraries for multi agent executable
target_link_libraries(multi_agent_planning
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLUT_LIBRARIES}
    yaml-cpp
    # Linux-specific libraries
    ${CMAKE_DL_LIBS}
    pthread
)

# Link libraries for test executable (no OpenGL needed)
target_link_libraries(path_planning_test path_planning_lib)

# Compiler-specific options
if(MSVC)
    # MSVC-specific options
    target_compile_options(path_planning PRIVATE /W4)
else()
    # GCC/Clang options
    target_compile_options(path_planning PRIVATE -Wall -Wextra -pedantic)
endif()
