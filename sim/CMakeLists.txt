cmake_minimum_required(VERSION 3.20)
project(denddron_gazebo_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gazebo REQUIRED)

# Nlohmann JSON - header only in extern/json/include if not found
find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    PATHS ${CMAKE_SOURCE_DIR}/extern/json/include
)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${NLOHMANN_JSON_INCLUDE_DIR}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_simulator_lib STATIC
    GazeboSimulator.cpp
    GazeboSimulator.hpp
)

target_link_libraries(gazebo_simulator_lib PUBLIC
    ${GAZEBO_LIBRARIES}
    zenohcxx::zenohc
)

add_executable(gazebo_simulator
    simulator_main.cpp
)

target_link_libraries(gazebo_simulator
    gazebo_simulator_lib
)
