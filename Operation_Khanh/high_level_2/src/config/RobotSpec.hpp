#pragma once

#include <array>

namespace spec {

// Kích thước dữ liệu
constexpr int kNumJoints    = 24;  // Số khớp policy
constexpr int kNumMotorsIdl = 35;  // Số motor LowCmd/LowState

constexpr int kFlatObsSize  = 83;
constexpr int kMimicObsSize = 129;

// Thứ tự khớp policy (0-5: L_leg, 6-11: R_leg, 12-13: Waist, 14-18: L_arm, 19-23: R_arm)
constexpr std::array<int, kNumJoints> kPolicyToSdk = {
    0,  1,  2,  3,  4,  5,   // Chân trái
    6,  7,  8,  9,  10, 11,  // Chân phải
    12, 13,                  // Waist
    14, 15, 16, 17, 18,      // Tay trái
    19, 20, 21, 22, 23       // Tay phải
};

// Map SDK joint -> index motor IDL
constexpr std::array<int, 26> kSdkToIdl = {
    0,  1,  2,  3,  4,  5,   // Chân trái
    6,  7,  8,  9,  10, 11,  // Chân phải
    12, 13,                  // Waist
    15, 16, 17, 18, 19,      // Tay trái
    22, 23, 24, 25, 26,      // Tay phải
    29, 30                   // Đầu (yaw, pitch)
};

// Map index policy -> motor IDL
constexpr int MotorIdl(int policy_idx) {
    return kSdkToIdl[static_cast<size_t>(kPolicyToSdk[static_cast<size_t>(policy_idx)])];
}

constexpr int kHeadYawIdl   = 29;
constexpr int kHeadPitchIdl = 30;

constexpr int kWaistRollPolicyIdx = 12;
constexpr int kWaistYawPolicyIdx  = 13;

// Tư thế mặc định
constexpr std::array<float, kNumJoints> kDefaultJointPos = {
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Chân trái
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Chân phải
    0.0f,  0.0f,                            // Waist
    0.35f, 0.18f, 0.0f, 0.87f, 0.0f,        // Tay trái
    0.35f, -0.18f, 0.0f, 0.87f, 0.0f        // Tay phải
};

// Action scale
constexpr std::array<float, kNumJoints> kActionScale = {
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // Chân trái
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // Chân phải
    0.125f, 0.22f,                                      // Waist
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f,   // Tay trái
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f    // Tay phải
};

// PD gains lúc train policy
constexpr std::array<float, kNumJoints> kKpTrain = {
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Chân trái
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Chân phải
    100.0f, 100.0f,                                 // Waist
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f,              // Tay trái
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f               // Tay phải
};

// Kd mặc định lúc train: hip/knee/waist = 3.0, ankle/tay = 2.0
constexpr std::array<float, kNumJoints> kKdTrain = {
    3.0f, 3.0f, 3.0f, 3.0f, 2.0f, 2.0f,   // Chân trái: hip x3, knee | ankle x2
    3.0f, 3.0f, 3.0f, 3.0f, 2.0f, 2.0f,   // Chân phải
    3.0f, 3.0f,                           // Waist
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f,         // Tay trái
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f          // Tay phải
};

// Chu kỳ gait (gait period) cố định
constexpr float kGaitPeriodS  = 0.8f;
constexpr float kCmdGateNorm  = 0.1f;

// Tần số điều khiển (500Hz loop, 50Hz policy)
constexpr float kLoopDt        = 0.002f;
constexpr int   kPolicyDecimation = 10;
constexpr float kPolicyDt      = kLoopDt * kPolicyDecimation;

// Index torso trong dance clip
constexpr int kMotionTorsoIdx  = 14;

} // namespace spec
