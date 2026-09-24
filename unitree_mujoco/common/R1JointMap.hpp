#pragma once

#include <array>
#include <cstddef>

// Canonical Unitree R1 joint mapping shared by the MuJoCo bridge and policy
// controllers.  R1 is not ordered like G1 at the waist:
//   R1 policy/XML/logical: waist_roll=12, waist_yaw=13
//   R1 LowCmd/LowState IDL: waist_roll=12, waist_yaw=13
namespace R1JointMap {

inline constexpr int kPolicyJointCount = 24;
inline constexpr int kLogicalJointCount = 26;

inline constexpr int kWaistRollPolicyIndex = 12;
inline constexpr int kWaistYawPolicyIndex = 13;
inline constexpr int kHeadPitchLogicalIndex = 24;
inline constexpr int kHeadYawLogicalIndex = 25;

// ONNX policy/XML order -> R1 logical joint order.
inline constexpr std::array<int, kPolicyJointCount> kPolicyToLogical = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    14, 15, 16, 17, 18,
    19, 20, 21, 22, 23,
};

// R1 logical joint order -> LowCmd/LowState IDL motor slot.
inline constexpr std::array<int, kLogicalJointCount> kLogicalToIdl = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13,
    15, 16, 17, 18, 19,
    22, 23, 24, 25, 26,
    29, 30,
};

constexpr std::array<int, kPolicyJointCount> MakePolicyToIdl() {
    std::array<int, kPolicyJointCount> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
        result[i] = kLogicalToIdl[kPolicyToLogical[i]];
    }
    return result;
}

// MuJoCo R1 actuator/sensor order is the same as the ONNX policy order.
inline constexpr auto kPolicyToIdl = MakePolicyToIdl();
inline constexpr auto kSimToIdl = kPolicyToIdl;

constexpr int PolicyToIdl(int policy_index) {
    return kPolicyToIdl[static_cast<std::size_t>(policy_index)];
}

static_assert(kPolicyToIdl[kWaistRollPolicyIndex] == 12,
              "R1 waist_roll must use IDL motor 12");
static_assert(kPolicyToIdl[kWaistYawPolicyIndex] == 13,
              "R1 waist_yaw must use IDL motor 13");
static_assert(kLogicalToIdl[kHeadPitchLogicalIndex] == 29,
              "R1 head_pitch must use IDL motor 29");
static_assert(kLogicalToIdl[kHeadYawLogicalIndex] == 30,
              "R1 head_yaw must use IDL motor 30");

}  // namespace R1JointMap
