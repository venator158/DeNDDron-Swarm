cmake_minimum_required(VERSION 3.20)
project(DeNDDron-Swarm CXX)

# ============================================================================
# Root CMakeLists.txt for DeNDDron Swarm Simulation
# ============================================================================
# This project builds:
# - Gazebo simulator bridge (sim/)
# - Metrics collection node (src/metrics/)
# - Agent binaries (src/ - when Agent.cpp is completed)
# ============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Find Dependencies
# ============================================================================

if(NOT TARGET zenohcpp)
    find_package(gazebo)
endif()

# ============================================================================
# Add Subdirectories
# ============================================================================

# Gazebo simulator bridge
add_subdirectory(sim)

# Metrics collection node
add_subdirectory(src/metrics)

# ============================================================================
# Agent Executable
# ============================================================================

# Collect agent source files
file(GLOB_RECURSE AGENT_SOURCES
    src/core/*.cpp
    src/handlers/*.cpp
    src/comms/*.cpp
    src/hal/*.cpp
    src/utils/*.cpp
)

# Agent executable
add_executable(denddron_agent
    src/main.cpp
    ${AGENT_SOURCES}
)

target_include_directories(denddron_agent PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/extern/json/include
)

target_link_libraries(denddron_agent PRIVATE
    zenohcpp
    pthread
)

# ============================================================================
# Summary
# ============================================================================

message(STATUS "DeNDDron Swarm build configured successfully")
message(STATUS "  - Gazebo simulator: sim/gazebo_simulator")
message(STATUS "  - Metrics node: src/metrics/metrics_node")
message(STATUS "  - Agent: denddron_agent")
