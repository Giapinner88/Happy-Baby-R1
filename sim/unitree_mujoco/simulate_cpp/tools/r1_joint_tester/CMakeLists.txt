cmake_minimum_required(VERSION 3.10)
project(r1_joint_tuner)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(UNITREE_SDK2_DIR $ENV{HOME}/unitree_sdk2)

include_directories(
    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include
    ${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx   # CycloneDDS C++ headers
)

# Link directories to find ddscxx and ddsc libs
link_directories(
    ${UNITREE_SDK2_DIR}/thirdparty/lib
    /usr/local/lib
)

add_executable(r1_dds_bridge r1_dds_bridge.cpp)

target_link_libraries(r1_dds_bridge
    ${UNITREE_SDK2_DIR}/lib/aarch64/libunitree_sdk2.a
    ddscxx
    ddsc
    pthread
)
