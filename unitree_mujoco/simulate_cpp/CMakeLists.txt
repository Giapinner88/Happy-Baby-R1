cmake_minimum_required(VERSION 3.16)
project(simulate_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Add Unitree SDK2 cmake configuration prefix
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(yaml-cpp REQUIRED)

# ONNX Runtime Configuration (relative path to unitree_sdk2 folder)
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_sdk2/thirdparty/onnxruntime")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# CNPY and Eigen3 Configuration. Prefer the original checkout, but allow the
# equivalent active mjlab checkouts when that directory is not present.
set(CNPY_DIR "")
foreach(CNPY_CANDIDATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_rl_mjlab/deploy/thirdparty/cnpy"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_rl_mjlab_meta/deploy/thirdparty/cnpy"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_rl_mjlab_dev/deploy/thirdparty/cnpy"
        "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_r1_mujoco_cpp/third_party/cnpy")
    if(EXISTS "${CNPY_CANDIDATE}/cnpy.cpp" AND EXISTS "${CNPY_CANDIDATE}/cnpy.h")
        set(CNPY_DIR "${CNPY_CANDIDATE}")
        break()
    endif()
endforeach()
if(NOT CNPY_DIR)
    message(FATAL_ERROR "Cannot find cnpy.cpp/cnpy.h in any supported checkout")
endif()
include_directories(${CNPY_DIR})
include_directories(/usr/include/eigen3)
set(
    MUJOCO_DIR
    "$ENV{HOME}/.mujoco/mujoco-3.3.6"
    CACHE PATH
    "MuJoCo installation prefix")
if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "MuJoCo headers not found below MUJOCO_DIR=${MUJOCO_DIR}")
endif()
include_directories(${MUJOCO_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
link_directories(${MUJOCO_DIR}/lib)

# Set rpath so the executable finds libonnxruntime.so, Unitree SDK libraries, and MuJoCo at runtime
set(CMAKE_INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib;/opt/unitree_robotics/lib;${MUJOCO_DIR}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Source files
add_library(
    r1_onnx STATIC
    src/onnx/OnnxModel.cpp
    src/onnx/PolicyContract.cpp)
target_link_libraries(r1_onnx PUBLIC onnxruntime)

add_library(r1_runtime STATIC src/runtime/Tuning.cpp)

# Gait v3 core is dependency-free so its FK/FSM contract can be exercised by
# unit tests without bringing up DDS, X11, or the MuJoCo application.
add_library(
    gait_v3_core STATIC
    src/controllers/locomotion/LegFK.cpp
    src/controllers/locomotion/GaitModeSchedulerV3.cpp)

# Generic history bank + packers for H4/H5 and A-RMA. Kept as its own target so
# contract tests link it without the DDS/mujoco/X11 stack.
add_library(
    flat_plus_history STATIC
    src/controllers/locomotion/BaseObservationHistory.cpp)

add_library(
    arma_core STATIC
    src/controllers/arma/ArmaRuntime.cpp)
target_link_libraries(arma_core PUBLIC r1_onnx yaml-cpp flat_plus_history)

add_library(
    slope_meta_core STATIC
    src/controllers/slope_meta/SlopeMetaRuntime.cpp)
target_link_libraries(slope_meta_core PUBLIC r1_onnx yaml-cpp)

add_library(
    rma_meta_core STATIC
    src/controllers/rma_meta/RmaMetaRuntime.cpp)
target_link_libraries(rma_meta_core PUBLIC r1_onnx yaml-cpp)

set(SOURCES
    src/app/main.cpp
    src/app/PolicyApplication.cpp
    src/controllers/locomotion/FlatController.cpp
    # FlatPlusController inherits the shared implementation below; its legacy
    # translation unit is intentionally not compiled.
    src/controllers/locomotion/FlatHistoryController.cpp
    src/controllers/locomotion/FlatPlusGaitController.cpp
    src/controllers/arma/ArmaController.cpp
    src/controllers/locomotion/RoughController.cpp
    src/controllers/slope_meta/SlopeMetaController.cpp
    src/controllers/rma_meta/RmaMetaController.cpp
    src/controllers/motion/DanceController.cpp
    src/controllers/motion/SitController.cpp
    src/motion/GestureLoader.cpp
    src/motion/MotionData.cpp
    ${CNPY_DIR}/cnpy.cpp
)

# Executable Target
add_executable(run_policy ${SOURCES})

# Locate and Link X11 for Graphical Window Capture
find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIR})
target_link_libraries(
    run_policy
    unitree_sdk2 r1_runtime gait_v3_core slope_meta_core rma_meta_core flat_plus_history arma_core mujoco ZLIB::ZLIB ${X11_LIBRARIES})

add_executable(
    slope_meta_preflight
    tools/slope_meta_preflight.cpp)
target_link_libraries(slope_meta_preflight slope_meta_core)

add_executable(
    rma_meta_preflight
    tools/rma_meta_preflight.cpp)
target_link_libraries(rma_meta_preflight rma_meta_core)

add_executable(
    arma_preflight
    tools/arma_preflight.cpp)
target_link_libraries(arma_preflight arma_core)

# Executable Target for R1 Policy
add_executable(run_r1_policy tools/run_r1_policy.cpp)
target_link_libraries(run_r1_policy unitree_sdk2 onnxruntime mujoco ZLIB::ZLIB ${X11_LIBRARIES})

include(CTest)
if(BUILD_TESTING)
    add_executable(
        arma_runtime_test
        tests/contracts/arma_runtime_test.cpp)
    target_link_libraries(arma_runtime_test arma_core)
    add_test(
        NAME arma_runtime_test
        COMMAND arma_runtime_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/arma/flat_o83_k50_z8_v1)

    find_program(ARMA_SHA256SUM_EXECUTABLE sha256sum)
    if(ARMA_SHA256SUM_EXECUTABLE)
        add_test(
            NAME arma_artifact_hash_test
            COMMAND ${ARMA_SHA256SUM_EXECUTABLE} --check SHA256SUMS)
        set_tests_properties(
            arma_artifact_hash_test
            PROPERTIES WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/arma/flat_o83_k50_z8_v1)
    endif()

    add_executable(r1_joint_mapping_test tests/unit/r1_joint_mapping_test.cpp)
    add_test(NAME r1_joint_mapping_test COMMAND r1_joint_mapping_test)

    add_executable(
        expert_action_adapter_test
        tests/unit/expert_action_adapter_test.cpp)
    add_test(NAME expert_action_adapter_test COMMAND expert_action_adapter_test)

    add_executable(
        slope_meta_selector_test
        tests/unit/slope_meta_selector_test.cpp)
    target_link_libraries(slope_meta_selector_test yaml-cpp)
    add_test(NAME slope_meta_selector_test COMMAND slope_meta_selector_test)

    add_executable(
        slope_meta_command_governor_test
        tests/unit/slope_meta_command_governor_test.cpp)
    add_test(
        NAME slope_meta_command_governor_test
        COMMAND slope_meta_command_governor_test)

    add_executable(
        slope_meta_runtime_test
        tests/contracts/slope_meta_runtime_test.cpp)
    target_link_libraries(slope_meta_runtime_test slope_meta_core)
    add_test(
        NAME slope_meta_runtime_test
        COMMAND slope_meta_runtime_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/slope_meta_v6)

    add_executable(
        meta_policy_runtime_test
        tests/contracts/meta_policy_runtime_test.cpp)
    target_link_libraries(meta_policy_runtime_test slope_meta_core)
    add_test(
        NAME meta_policy_runtime_test
        COMMAND meta_policy_runtime_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/meta_policy_v1)

    add_executable(
        rma_meta_runtime_test
        tests/contracts/rma_meta_runtime_test.cpp)
    target_link_libraries(rma_meta_runtime_test rma_meta_core)
    add_test(
        NAME rma_meta_runtime_test
        COMMAND rma_meta_runtime_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/rma_meta_v1)

    find_program(SHA256SUM_EXECUTABLE sha256sum)
    if(SHA256SUM_EXECUTABLE)
        add_test(
            NAME slope_meta_artifact_hash_test
            COMMAND ${SHA256SUM_EXECUTABLE} --check SHA256SUMS)
        set_tests_properties(
            slope_meta_artifact_hash_test
            PROPERTIES WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/slope_meta_v6)
        add_test(
            NAME meta_policy_artifact_hash_test
            COMMAND ${SHA256SUM_EXECUTABLE} --check SHA256SUMS)
        set_tests_properties(
            meta_policy_artifact_hash_test
            PROPERTIES WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/meta_policy_v1)
        add_test(
            NAME rma_meta_artifact_hash_test
            COMMAND ${SHA256SUM_EXECUTABLE} --check SHA256SUMS)
        set_tests_properties(
            rma_meta_artifact_hash_test
            PROPERTIES WORKING_DIRECTORY
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/meta/rma_meta_v1)
    endif()

    add_executable(rough_policy_contract_test tests/contracts/rough_policy_contract_test.cpp)
    target_link_libraries(rough_policy_contract_test onnxruntime)
    add_test(
        NAME rough_policy_contract_test_selected
        COMMAND rough_policy_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/rough/policy_r1_rough.onnx)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/rough/policy_r1_270.onnx)
        add_test(
            NAME rough_policy_contract_test_legacy_270
            COMMAND rough_policy_contract_test
                    ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/rough/policy_r1_270.onnx)
    endif()

    add_executable(flat_policy_contract_test tests/contracts/flat_policy_contract_test.cpp)
    target_link_libraries(flat_policy_contract_test onnxruntime)
    add_test(
        NAME flat_policy_contract_test_policy_goc
        COMMAND flat_policy_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/flat/policy_goc.onnx)

    add_executable(
        single_policy_smoke_test
        tests/contracts/single_policy_smoke_test.cpp)
    target_link_libraries(single_policy_smoke_test r1_onnx)
    file(
        GLOB_RECURSE SINGLE_POLICY_ONNX
        CONFIGURE_DEPENDS
        RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        policy/locomotion/*.onnx)
    foreach(POLICY_PATH IN LISTS SINGLE_POLICY_ONNX)
        # Meta and ARMA components are composite packages, not standalone
        # [1,N] -> [1,24] policies. Their dedicated runtime tests above own
        # the contract; feeding adapter.onnx to this smoke test is a false
        # failure and can hide the actual single-policy result.
        if(NOT POLICY_PATH MATCHES "^policy/locomotion/(meta|arma)/")
            string(REGEX REPLACE "[^A-Za-z0-9_]" "_" POLICY_TEST_NAME "${POLICY_PATH}")
            add_test(
                NAME single_policy_${POLICY_TEST_NAME}
                COMMAND single_policy_smoke_test
                        ${CMAKE_CURRENT_SOURCE_DIR}/${POLICY_PATH})
            # This archived 20k gait checkpoint is intentionally incomplete
            # (it predates the exported metadata contract). Keep it available
            # for forensic comparison, but make its fail-closed rejection an
            # explicit expected result instead of a red full-suite preflight.
            if(POLICY_PATH MATCHES "policy_flat_plus_gait_h4_20k\\.onnx$")
                set_tests_properties(
                    single_policy_${POLICY_TEST_NAME}
                    PROPERTIES WILL_FAIL TRUE)
            endif()
        endif()
    endforeach()

    add_executable(scene_resolver_test tests/scenes/scene_resolver_test.cpp)
    add_test(NAME scene_resolver_test COMMAND scene_resolver_test)

    add_executable(flat_plus_history_test tests/unit/flat_plus_history_test.cpp)
    target_link_libraries(flat_plus_history_test flat_plus_history)
    add_test(NAME flat_plus_history_test COMMAND flat_plus_history_test)

    add_executable(gait_mode_scheduler_test tests/unit/gait_mode_scheduler_test.cpp)
    add_test(NAME gait_mode_scheduler_test COMMAND gait_mode_scheduler_test)

    add_executable(gait_golden_trace_test tests/unit/gait_golden_trace_test.cpp)
    add_test(
        NAME gait_golden_trace_test
        COMMAND gait_golden_trace_test
                ${CMAKE_CURRENT_SOURCE_DIR}/policy/locomotion/flat_plus/gait_golden_trace.txt)

    add_executable(gait_v3_scheduler_test tests/unit/gait_v3_scheduler_test.cpp)
    target_link_libraries(gait_v3_scheduler_test gait_v3_core)
    add_test(NAME gait_v3_scheduler_test COMMAND gait_v3_scheduler_test)

    add_executable(flat_plus_golden_trace_test tests/unit/flat_plus_golden_trace_test.cpp)
    target_link_libraries(flat_plus_golden_trace_test flat_plus_history)
    add_test(
        NAME flat_plus_golden_trace_test
        COMMAND flat_plus_golden_trace_test
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/unit/flat_plus_golden_trace.txt)

    add_executable(tuning_profile_test tests/unit/tuning_profile_test.cpp)
    target_link_libraries(tuning_profile_test r1_runtime)
    add_test(
        NAME tuning_profile_test
        COMMAND tuning_profile_test ${CMAKE_CURRENT_SOURCE_DIR}/config/tuning.yaml)

    add_executable(mimic_transition_test tests/unit/mimic_transition_test.cpp)
    add_test(NAME mimic_transition_test COMMAND mimic_transition_test)

    add_executable(slope_scene_contract_test tests/scenes/slope_scene_contract_test.cpp)
    target_link_libraries(slope_scene_contract_test mujoco)
    add_test(
        NAME slope_scene_contract_5_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_5.xml 5 direct)
    add_test(
        NAME slope_scene_contract_10_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_10.xml 10 direct)
    add_test(
        NAME slope_scene_contract_15_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_15.xml 15 direct)
    add_test(
        NAME slope_scene_contract_20_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_20.xml 20 direct)
    add_test(
        NAME slope_scene_contract_25_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_25.xml 25 direct)
    add_test(
        NAME slope_scene_contract_30_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_30.xml 30 direct)
    add_test(
        NAME slope_scene_contract_35_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_35.xml 35 direct)
    add_test(
        NAME slope_scene_contract_40_direct
        COMMAND slope_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_40.xml 40 direct)
    foreach(PLATFORM_ANGLE 5 10 15 20 25 30)
        add_test(
            NAME slope_scene_contract_${PLATFORM_ANGLE}_platform
            COMMAND slope_scene_contract_test
                    ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_slope_${PLATFORM_ANGLE}_platform.xml
                    ${PLATFORM_ANGLE} platform)
    endforeach()

    find_program(PYTHON3_EXECUTABLE python3)
    if(PYTHON3_EXECUTABLE)
        add_test(
            NAME meta_policy_platform_scene_generation_test
            COMMAND ${PYTHON3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_meta_policy_platform_scenes.py
                    --check)
    endif()

    add_executable(mixed_terrain_scene_contract_test
                   tests/scenes/mixed_terrain_scene_contract_test.cpp)
    target_link_libraries(mixed_terrain_scene_contract_test mujoco)
    add_test(
        NAME mixed_terrain_scene_contract
        COMMAND mixed_terrain_scene_contract_test
                ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_robots/r1/scene_mixed_terrain.xml)
endif()
