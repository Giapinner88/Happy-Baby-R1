cmake_minimum_required(VERSION 3.16)
project(r1_low_level CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Unitree SDK2
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_bridge)

# RPATH for Jetson
set(CMAKE_INSTALL_RPATH "/opt/unitree_robotics/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# C++ Bridge
add_executable(r1_dds_bridge src/cpp_bridge/r1_dds_bridge.cpp)
target_link_libraries(r1_dds_bridge unitree_sdk2 pthread)
