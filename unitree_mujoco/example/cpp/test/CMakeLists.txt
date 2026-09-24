cmake_minimum_required(VERSION 3.16)
project(stand_g1)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_g1 test_g1.cpp)
target_link_libraries(stand_g1  unitree_sdk2)
set(CMAKE_CXX_STANDARD 17)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE Release)
