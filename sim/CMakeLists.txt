find_package(nlohmann_json REQUIRED)
cmake_minimum_required(VERSION 3.10)
project(denddron_gazebo_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
# Gazebo - the physics simulator client library
find_package(gazebo REQUIRED)

# nlohmann_json - for JSON parsing/generation
find_package(nlohmann_json 3.2.0 REQUIRED)

# Zenoh-C - for distributed messaging
# Include directories
include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add linker directories
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

# ============================================================================
# Gazebo Simulator Bridge Library
# ============================================================================
# This library provides the core simulator bridge functionality:
# - Connection to Gazebo gzserver
# - Zenoh subscriber/publisher setup
# - Drone spawning and control
# - Sensor simulation and data publishing
add_library(gazebo_simulator_lib STATIC
    GazeboSimulator.cpp
    GazeboSimulator.hpp
)

target_include_directories(gazebo_simulator_lib PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(gazebo_simulator_lib PUBLIC
    ${GAZEBO_LIBRARIES}
    nlohmann_json::nlohmann_json
    zenohcpp
)

# ============================================================================
# Main Simulator Executable
# ============================================================================
# Entry point for the Gazebo simulator container.
# This executable:
# 1. Starts the simulator bridge
# 2. Listens for agent join events
# 3. Manages drone lifecycle
# 4. Publishes sensor data and metrics
add_executable(gazebo_simulator
    simulator_main.cpp
)

target_link_libraries(gazebo_simulator
    gazebo_simulator_lib
)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS gazebo_simulator
    RUNTIME DESTINATION bin
)

install(TARGETS gazebo_simulator_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    GazeboSimulator.hpp
    DESTINATION include/denddron
)

install(FILES
    ocean.world
    DESTINATION share/denddron/worlds
)
