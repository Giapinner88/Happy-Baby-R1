#pragma once

#include <array>

namespace spec {

// Kích thước mảng dữ liệu
constexpr int kNumJoints    = 24;  // Số khớp policy điều khiển
constexpr int kNumMotorsIdl = 35;  // Số motor trong LowCmd/LowState
constexpr int kArmBeginPolicyIdx = 14;
constexpr int kNumArmJoints = 10;

constexpr int kFlatObsSize  = 83;
constexpr int kMimicObsSize = 129;

// Thứ tự khớp của policy (khớp với ONNX metadata):
// 0-5: Chân trái
// 6-11: Chân phải
// 12-13: Waist (roll, yaw)
// 14-18: Tay trái
// 19-23: Tay phải
constexpr std::array<int, kNumJoints> kPolicyToSdk = {
    0,  1,  2,  3,  4,  5,   // Chân trái
    6,  7,  8,  9,  10, 11,  // Chân phải
    12, 13,                  // Waist
    14, 15, 16, 17, 18,      // Tay trái
    19, 20, 21, 22, 23       // Tay phải
};

// Vị trí SDK joint -> motor index trong mảng IDL (LowCmd/LowState)
constexpr std::array<int, 26> kSdkToIdl = {
    0,  1,  2,  3,  4,  5,   // Chân trái
    6,  7,  8,  9,  10, 11,  // Chân phải
    12, 13,                  // Waist
    15, 16, 17, 18, 19,      // Tay trái
    22, 23, 24, 25, 26,      // Tay phải
    29, 30                   // Đầu (yaw, pitch)
};

// Hàm map từ index policy sang index motor IDL
constexpr int MotorIdl(int policy_idx) {
    return kSdkToIdl[static_cast<size_t>(kPolicyToSdk[static_cast<size_t>(policy_idx)])];
}

// R1-A5 IDL order is pitch then yaw (verified against the pinned Unitree
// arm/head interface used by hardware/teleop).
constexpr int kHeadPitchIdl = 29;
constexpr int kHeadYawIdl   = 30;

constexpr int kWaistRollPolicyIdx = 12;
constexpr int kWaistYawPolicyIdx  = 13;

// Tư thế mặc định (default joint position)
constexpr std::array<float, kNumJoints> kDefaultJointPos = {
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Chân trái
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // Chân phải
    0.0f,  0.0f,                            // Waist
    0.35f, 0.18f, 0.0f, 0.87f, 0.0f,        // Tay trái
    0.35f, -0.18f, 0.0f, 0.87f, 0.0f        // Tay phải
};

// Tỉ lệ tỷ lệ hành động (action scale)
constexpr std::array<float, kNumJoints> kActionScale = {
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // Chân trái
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // Chân phải
    0.125f, 0.22f,                                      // Waist
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f,   // Tay trái
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f    // Tay phải
};

// PD gains cố định lúc huấn luyện policy
constexpr std::array<float, kNumJoints> kKpTrain = {
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Chân trái
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // Chân phải
    100.0f, 100.0f,                                 // Waist
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f,              // Tay trái
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f               // Tay phải
};

// Hip/knee/waist dùng damping 3.0 (r1_constants.py đổi 2.0 -> 3.0 ngày 2026-07-09),
// ankle/tay giữ 2.0. Policy mimic KHÔNG mang metadata nên rơi thẳng vào mảng này —
// sai ở đây là chân bị under-damped so với lúc train.
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

// Tần số điều khiển (500Hz DDS loop, 50Hz policy inference)
constexpr float kLoopDt        = 0.002f;
constexpr int   kPolicyDecimation = 10;
constexpr float kPolicyDt      = kLoopDt * kPolicyDecimation;

// Index của torso trong dance clip (dance*.npz)
constexpr int kMotionTorsoIdx  = 14;

} // namespace spec
