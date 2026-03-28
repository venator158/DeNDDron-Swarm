find_package(nlohmann_json REQUIRED)
# ============================================================================
# Metrics Node Build Configuration
# ============================================================================
# Builds the metrics collection node that observes the swarm simulation
# 
# This node subscribes to:
# - swarm/metrics       (simulator telemetry)
# - swarm/agents/join   (spawn events)
# - swarm/agents/despawn (destruction events)
# 
# And aggregates into:
# - Per-agent metrics with position history
# - Swarm-wide statistics with survival analysis
# - Event logs with timestamps and reasons
# ============================================================================

cmake_minimum_required(VERSION 3.20)

# ============================================================================
# Find Dependencies
# ============================================================================


# ============================================================================
# Metrics Library
# ============================================================================
# Shared library containing the metrics collection logic
# This can be linked by multiple executables if needed

add_library(metrics_lib STATIC
    MetricsNode.cpp
    MetricsNode.hpp
)

target_include_directories(metrics_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(metrics_lib 
    PUBLIC
        zenohcpp
        nlohmann_json::nlohmann_json
)

target_compile_features(metrics_lib PUBLIC cxx_std_17)

# ============================================================================
# Metrics Node Executable
# ============================================================================
# Main entry point for the metrics collection service
# 
# Usage in docker-compose:
#   metrics:
#     image: denddron/metrics:latest
#     depends_on:
#       - zenoh_router
#       - gazebo_simulator
#     environment:
#       ZENOH_ROUTER: zenoh_router:7447
#

add_executable(metrics_node
    metrics_main.cpp
)

target_link_libraries(metrics_node
    PRIVATE
        metrics_lib
)

target_compile_features(metrics_node PRIVATE cxx_std_17)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS metrics_node
    RUNTIME DESTINATION bin
)

install(TARGETS metrics_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES MetricsNode.hpp
    DESTINATION include/denddron/metrics
)
