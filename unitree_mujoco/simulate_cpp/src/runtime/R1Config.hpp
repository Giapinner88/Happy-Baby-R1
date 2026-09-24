#ifndef R1_CONFIG_HPP
#define R1_CONFIG_HPP

#include <array>
#include "../../../common/R1JointMap.hpp"

namespace R1Config {
    inline constexpr int NUM_JOINTS = R1JointMap::kPolicyJointCount;

    // R1 policy/XML order: waist_roll=12, waist_yaw=13.
    inline constexpr auto POLICY_TO_R1_SDK_JOINT = R1JointMap::kPolicyToLogical;

    // Ánh xạ SDK joint index -> IDL motor index (26 phần tử: 24 khớp + 2 đầu).
    inline constexpr auto joint_idx_in_idl = R1JointMap::kLogicalToIdl;

    inline constexpr int WAIST_ROLL_POLICY_INDEX = R1JointMap::kWaistRollPolicyIndex;
    inline constexpr int WAIST_YAW_POLICY_INDEX = R1JointMap::kWaistYawPolicyIndex;
    inline constexpr int HEAD_PITCH_LOGICAL_INDEX = R1JointMap::kHeadPitchLogicalIndex;
    inline constexpr int HEAD_YAW_LOGICAL_INDEX = R1JointMap::kHeadYawLogicalIndex;

    constexpr int PolicyToIdl(int policy_index) {
        return R1JointMap::PolicyToIdl(policy_index);
    }

    // Tư thế đứng mặc định (rad).
    const std::array<float, NUM_JOINTS> DEFAULT_JOINT_POS = {
        -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Left leg
        -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Right leg
        0.0f, 0.0f,                               // Waist
        0.35f, 0.18f, 0.0f, 0.87f, 0.0f,         // Left arm
        0.35f, -0.18f, 0.0f, 0.87f, 0.0f         // Right arm
    };

    // action_raw * scale + default_q = target_q.
    const std::array<float, NUM_JOINTS> ACTION_SCALE = {
        0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f, // Left leg
        0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f, // Right leg
        0.125f, 0.22f,                                   // Waist
        0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f, // Left arm
        0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f  // Right arm
    };

    // PD gains khớp với lúc train (kKpTrain/kKdTrain trong RobotSpec.hpp).
    // Cập nhật 2026-07-09: KD hip/gối/eo 2->3, tay KP 20->40, KD 1->2.
    const std::array<float, NUM_JOINTS> KP_ARRAY = {
        100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Left leg
        100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Right leg
        100.0f, 100.0f,                                   // Waist
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f,               // Left arm
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f                // Right arm
    };

    const std::array<float, NUM_JOINTS> KD_ARRAY = {
        3.0f, 3.0f, 3.0f, 3.0f, 2.0f, 2.0f,             // Left leg: hip x3, knee=3 | ankle=2
        3.0f, 3.0f, 3.0f, 3.0f, 2.0f, 2.0f,             // Right leg
        3.0f, 3.0f,                                       // Waist = 3
        2.0f, 2.0f, 2.0f, 2.0f, 2.0f,                   // Left arm = 2
        2.0f, 2.0f, 2.0f, 2.0f, 2.0f                    // Right arm = 2
    };

    // HB/high_level_2 return gains: keep the standing body supported while
    // locomotion hands the robot to a dance controller.
    const std::array<float, NUM_JOINTS> DANCE_RETURN_KP_ARRAY = {
        200.0f, 200.0f, 200.0f, 200.0f, 120.0f, 120.0f,
        200.0f, 200.0f, 200.0f, 200.0f, 120.0f, 120.0f,
        200.0f, 200.0f,
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f,
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f
    };

    const std::array<float, NUM_JOINTS> DANCE_RETURN_KD_ARRAY = {
        3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
        3.0f, 3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
        3.0f, 3.0f,
        3.0f, 3.0f, 3.0f, 3.0f, 3.0f,
        3.0f, 3.0f, 3.0f, 3.0f, 3.0f
    };
}

#endif
