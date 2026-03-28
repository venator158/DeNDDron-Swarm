cmake_minimum_required(VERSION 3.20)
project(DeNDDron-Swarm CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(gazebo)

add_subdirectory(sim)

