cmake_minimum_required(VERSION 3.16)
project(hb_high_level CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # cho clangd/IDE

# ─── Unitree SDK2 ─────────────────────────────────────────
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)


# ─── ZLIB (cnpy cần) ──────────────────────────────────────
find_package(ZLIB REQUIRED)

# ─── ONNX Runtime (tự chọn theo kiến trúc CPU) ────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime_aarch64")
else()
    set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/onnxruntime")
endif()
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# ─── cnpy (bundled) ───────────────────────────────────────
set(CNPY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cnpy")
include_directories(${CNPY_DIR})

# ─── Eigen3 ───────────────────────────────────────────────
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
else()
    include_directories(/usr/include/eigen3)
endif()

# ─── X11 ──────────────────────────────────────────────────
find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Header Unitree AudioClient (a2/audio) nằm ở /usr/local/include trên robot —
# thêm tường minh phòng khi gói unitree_sdk2 (/opt) không kèm phần audio.
include_directories(/usr/local/include)

# ─── RPATH (để tìm .so lúc chạy trên robot) ───────────────
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib;/opt/unitree_robotics/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# ─── Build ────────────────────────────────────────────────
add_executable(run_r1
    src/main.cpp
    src/app/Application.cpp
    src/config/Tuning.cpp
    src/motion/MotionData.cpp
    src/policy/OnnxPolicy.cpp
    src/input/KeyboardX11.cpp
    ${CNPY_DIR}/cnpy.cpp
)

target_link_libraries(run_r1
    unitree_sdk2
    onnxruntime
    ZLIB::ZLIB
    ${X11_LIBRARIES}
    pthread
)
