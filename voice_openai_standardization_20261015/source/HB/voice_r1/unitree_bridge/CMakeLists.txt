cmake_minimum_required(VERSION 3.10)
project(voice_r1_unitree_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(UNITREE_SDK2_ROOT "" CACHE PATH "Path to a Unitree SDK2 source tree with R1 headers")

if(UNITREE_SDK2_ROOT)
  if(NOT EXISTS "${UNITREE_SDK2_ROOT}/include/unitree/robot/r1/audio/audio_client.hpp")
    message(FATAL_ERROR "UNITREE_SDK2_ROOT does not contain R1 audio headers: ${UNITREE_SDK2_ROOT}")
  endif()

  set(UNITREE_SDK2_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
  set(UNITREE_SDK2_LIB "${UNITREE_SDK2_ROOT}/lib/${UNITREE_SDK2_ARCH}/libunitree_sdk2.a")
  set(UNITREE_DDSC_LIB "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddsc.so")
  set(UNITREE_DDSCXX_LIB "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddscxx.so")

  if(NOT EXISTS "${UNITREE_SDK2_LIB}")
    message(FATAL_ERROR "Unitree SDK2 library not found: ${UNITREE_SDK2_LIB}")
  endif()

  add_library(unitree_sdk2 STATIC IMPORTED)
  set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION "${UNITREE_SDK2_LIB}")
  target_include_directories(unitree_sdk2 INTERFACE
    "${UNITREE_SDK2_ROOT}/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include"
    "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")

  set(UNITREE_EXTRA_LIBS "${UNITREE_DDSC_LIB}" "${UNITREE_DDSCXX_LIB}" pthread)
  set(CMAKE_BUILD_RPATH "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}")
else()
  find_package(unitree_sdk2 REQUIRED)
  set(UNITREE_EXTRA_LIBS "")
endif()

add_executable(r1_bridge r1_bridge.cpp)
target_link_libraries(r1_bridge unitree_sdk2 ${UNITREE_EXTRA_LIBS})
