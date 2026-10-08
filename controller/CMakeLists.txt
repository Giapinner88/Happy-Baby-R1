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
find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime libonnxruntime.so.1.22.0 libonnxruntime.so.1.15.1
    PATHS ${ONNXRUNTIME_DIR}/lib
    NO_DEFAULT_PATH
)
if(NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "Không tìm thấy ONNX Runtime trong ${ONNXRUNTIME_DIR}/lib")
endif()

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
    src/logging/MimicTelemetryLogger.cpp
    src/motion/MotionData.cpp
    src/motion/JointTrajectory.cpp
    src/policy/OnnxPolicy.cpp
    src/input/KeyboardX11.cpp
    ${CNPY_DIR}/cnpy.cpp
)

target_link_libraries(run_r1
    unitree_sdk2
    ${ONNXRUNTIME_LIB}
    ZLIB::ZLIB
    ${X11_LIBRARIES}
    pthread
)

include(CTest)
if(BUILD_TESTING)
    add_executable(teleop_h4_policy_gate_test tests/teleop_h4_policy_gate_test.cpp)
    add_test(NAME teleop_h4_policy_gate_test COMMAND teleop_h4_policy_gate_test)
    add_executable(application_safety_test
        tests/application_safety_test.cpp
        src/app/Application.cpp src/config/Tuning.cpp
        src/logging/MimicTelemetryLogger.cpp
        src/motion/MotionData.cpp src/motion/JointTrajectory.cpp
        src/policy/OnnxPolicy.cpp src/input/KeyboardX11.cpp ${CNPY_DIR}/cnpy.cpp)
    target_compile_definitions(application_safety_test PRIVATE
        HB_TEST_PACKET_SINK HB_PROJECT_PATH="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(application_safety_test unitree_sdk2 ${ONNXRUNTIME_LIB}
        ZLIB::ZLIB ${X11_LIBRARIES} pthread)
    add_test(NAME application_safety_test COMMAND application_safety_test)

    add_executable(input_sample_progress_test tests/input_sample_progress_test.cpp)
    target_link_libraries(input_sample_progress_test unitree_sdk2 pthread)
    add_test(NAME input_sample_progress_test COMMAND input_sample_progress_test)

    add_executable(motion_validation_test tests/motion_validation_test.cpp
        src/motion/JointTrajectory.cpp src/motion/MotionData.cpp ${CNPY_DIR}/cnpy.cpp)
    target_link_libraries(motion_validation_test ZLIB::ZLIB)
    add_test(NAME motion_validation_test COMMAND motion_validation_test)

    add_executable(mimic_transition_test tests/unit/mimic_transition_test.cpp)
    add_test(NAME mimic_transition_test COMMAND mimic_transition_test)

    add_executable(tensor_contract_test tests/tensor_contract_test.cpp)
    target_link_libraries(tensor_contract_test ${ONNXRUNTIME_LIB})
    add_test(NAME tensor_contract_test COMMAND tensor_contract_test)

    add_executable(onnx_file_contract_test tests/onnx_file_contract_test.cpp
        src/policy/OnnxPolicy.cpp)
    target_link_libraries(onnx_file_contract_test ${ONNXRUNTIME_LIB})
    set(HB_TEST_PYTHON "" CACHE FILEPATH "Optional host Python with onnx/numpy for generated model tests")
    if(HB_TEST_PYTHON)
        add_test(NAME onnx_file_contract_test COMMAND "${HB_TEST_PYTHON}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/onnx_file_contract_test.py"
            $<TARGET_FILE:onnx_file_contract_test>)
    endif()
    add_executable(remote_health_test tests/remote_health_test.cpp)
    target_link_libraries(remote_health_test unitree_sdk2 pthread)
    add_test(NAME remote_health_test COMMAND remote_health_test)

    add_executable(lowcmd_upper_body_zero_torque_test
        tests/lowcmd_upper_body_zero_torque_test.cpp)
    target_link_libraries(lowcmd_upper_body_zero_torque_test unitree_sdk2 pthread)
    add_test(NAME lowcmd_upper_body_zero_torque_test
             COMMAND lowcmd_upper_body_zero_torque_test)

    add_executable(lowcmd_safety_gate_test tests/lowcmd_safety_gate_test.cpp)
    target_link_libraries(lowcmd_safety_gate_test unitree_sdk2 pthread)
    add_test(NAME lowcmd_safety_gate_test COMMAND lowcmd_safety_gate_test)

    add_executable(state_estimator_test
        tests/state_estimator_test.cpp
        src/config/Tuning.cpp
    )
    target_link_libraries(state_estimator_test unitree_sdk2 pthread)
    add_test(NAME state_estimator_test COMMAND state_estimator_test)

    add_executable(teleop_receiver_test tests/teleop_receiver_test.cpp)
    target_link_libraries(teleop_receiver_test pthread)
    add_test(NAME teleop_receiver_test COMMAND teleop_receiver_test)

    add_executable(fall_detector_test
        tests/fall_detector_test.cpp
        src/config/Tuning.cpp
    )
    target_compile_definitions(fall_detector_test PRIVATE
        HB_TUNING_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tuning.yaml")
    add_test(NAME fall_detector_test COMMAND fall_detector_test)

    add_executable(arm_gesture_player_test tests/arm_gesture_player_test.cpp)
    add_test(NAME arm_gesture_player_test COMMAND arm_gesture_player_test)

    add_executable(voice_gesture_owner_test tests/voice_gesture_owner_test.cpp)
    add_test(NAME voice_gesture_owner_test COMMAND voice_gesture_owner_test)

    add_executable(gesture_handover_guard_test tests/gesture_handover_guard_test.cpp)
    add_test(NAME gesture_handover_guard_test COMMAND gesture_handover_guard_test)

    add_executable(joint_trajectory_head_test
        tests/joint_trajectory_head_test.cpp
        src/motion/JointTrajectory.cpp
        ${CNPY_DIR}/cnpy.cpp
    )
    target_link_libraries(joint_trajectory_head_test ZLIB::ZLIB)
    add_test(NAME joint_trajectory_head_test COMMAND joint_trajectory_head_test)

    add_executable(ankle_limits_test
        tests/ankle_limits_test.cpp
        src/motion/JointTrajectory.cpp
        ${CNPY_DIR}/cnpy.cpp
    )
    target_link_libraries(ankle_limits_test ZLIB::ZLIB)
    add_test(NAME ankle_limits_test COMMAND ankle_limits_test
             ${CMAKE_CURRENT_SOURCE_DIR}/motions/getup.npz
             ${CMAKE_CURRENT_SOURCE_DIR}/motions/liedown.npz)

    add_executable(flat_policy_profile_test tests/flat_policy_profile_test.cpp)
    add_test(NAME flat_policy_profile_test COMMAND flat_policy_profile_test)

    add_executable(flat_plus_history_test tests/flat_plus_history_test.cpp)
    add_test(NAME flat_plus_history_test COMMAND flat_plus_history_test)

    add_executable(gait_mode_scheduler_test tests/gait_mode_scheduler_test.cpp)
    add_test(NAME gait_mode_scheduler_test COMMAND gait_mode_scheduler_test)

    add_executable(gait_golden_trace_test tests/gait_golden_trace_test.cpp)
    add_test(
        NAME gait_golden_trace_test
        COMMAND gait_golden_trace_test
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/gait_golden_trace.txt)

    add_test(
        NAME policy_artifact_manifest_test
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/policy_artifact_manifest_test.sh
                       ${CMAKE_CURRENT_SOURCE_DIR})

    add_test(
        NAME flat_plus_profiles_preflight_test
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/flat_plus_profiles_preflight_test.sh
                       ${CMAKE_CURRENT_SOURCE_DIR} $<TARGET_FILE:run_r1>)

    add_test(
        NAME flat_plus_v5_preflight_test
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/flat_plus_profiles_preflight_test.sh
                       ${CMAKE_CURRENT_SOURCE_DIR} $<TARGET_FILE:run_r1>
                       gait_0917_v5_a1 gait_0917_v5_a2a)

    add_test(
        NAME flat_plus_armfree_settle_config_test
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/tests/flat_plus_profiles_preflight_test.sh
                       ${CMAKE_CURRENT_SOURCE_DIR} $<TARGET_FILE:run_r1>
                       gait_0917_armfree_settle_override)

    add_executable(flat_plus_base83_golden_test tests/flat_plus_base83_golden_test.cpp)
    target_link_libraries(flat_plus_base83_golden_test unitree_sdk2 pthread)
    add_test(
        NAME flat_plus_base83_golden_test
        COMMAND flat_plus_base83_golden_test
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/flat_plus_golden_trace.txt)

    add_executable(flat_plus_golden_trace_test tests/flat_plus_golden_trace_test.cpp)
    add_test(
        NAME flat_plus_golden_trace_test
        COMMAND flat_plus_golden_trace_test
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/data/flat_plus_golden_trace.txt)

    add_executable(onnx_input_contract_test
        tests/onnx_input_contract_test.cpp
        src/policy/OnnxPolicy.cpp
    )
    target_link_libraries(onnx_input_contract_test ${ONNXRUNTIME_LIB})
    target_compile_definitions(onnx_input_contract_test PRIVATE
        HB_DOREMON_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/policies/dance/doremon_v8/policy.onnx")
    add_test(NAME onnx_input_contract_test COMMAND onnx_input_contract_test)

    add_executable(tuning_include_test tests/tuning_include_test.cpp src/config/Tuning.cpp)
    target_compile_definitions(tuning_include_test PRIVATE
        HB_TUNING_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tuning.yaml")
    add_test(NAME tuning_include_test COMMAND tuning_include_test)

    add_executable(gait_scheduler_test tests/gait_scheduler_test.cpp src/config/Tuning.cpp)
    target_compile_definitions(gait_scheduler_test PRIVATE
        HB_TUNING_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tuning.yaml")
    add_test(NAME gait_scheduler_test COMMAND gait_scheduler_test)

    add_test(NAME config_reader_test
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/config_reader_test.sh"
                     "${CMAKE_CURRENT_SOURCE_DIR}/../integration/scripts/config_reader.sh"
                     "${CMAKE_CURRENT_SOURCE_DIR}/config/tuning.yaml")

    add_test(NAME preflight_test COMMAND run_r1 --preflight)
    set_tests_properties(preflight_test PROPERTIES
        ENVIRONMENT "HB_PROJECT_DIR=${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# ─── record_motion: ghi quỹ đạo khớp từ built-in (chỉ nghe, không gửi gì) ───
# Dùng để tạo motions/getup.npz + motions/liedown.npz cho chức năng đứng lên/nằm
# xuống (L2+X). Xem docs/operation.md ở gốc repo.
add_executable(record_motion tools/record_motion.cpp ${CNPY_DIR}/cnpy.cpp)
target_link_libraries(record_motion unitree_sdk2 ZLIB::ZLIB pthread)
