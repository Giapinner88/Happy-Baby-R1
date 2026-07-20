#pragma once
/**
 * RobotSpec.hpp — Các hằng số GẮN CHẶT VỚI LÚC TRAIN (train-locked).
 * ⚠️ KHÔNG TUNE các giá trị trong file này.
 */

#include <array>

namespace spec {

// Kích thước
constexpr int kNumJoints    = 24;  // số khớp policy điều khiển
constexpr int kNumMotorsIdl = 35;  // kích thước mảng motor trong LowCmd_/LowState_

constexpr int kFlatObsSize  = 83;  // gyro3 + grav3 + cmd3 + phase2 + q24 + dq24 + act24
constexpr int kMimicObsSize = 129; // jp24 + jv24 + ori6 + gyro3 + q24 + dq24 + act24

// Thứ tự khớp của POLICY:
//   0-5   : chân trái  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
//   6-11  : chân phải  (như trên)
//   12    : waist_roll
//   13    : waist_yaw
//   14-18 : tay trái   (shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll)
//   19-23 : tay phải   (như trên)


constexpr std::array<int, kNumJoints> kPolicyToSdk = {
    0,  1,  2,  3,  4,  5,   // chân trái
    6,  7,  8,  9,  10, 11,  // chân phải
    12, 13,                  // waist (roll, yaw)
    14, 15, 16, 17, 18,      // tay trái
    19, 20, 21, 22, 23       // tay phải
};

// SDK joint index -> vị trí motor trong mảng IDL (LowCmd_/LowState_).
constexpr std::array<int, 26> kSdkToIdl = {
    0,  1,  2,  3,  4,  5,   // chân trái
    6,  7,  8,  9,  10, 11,  // chân phải
    12, 13,                  // waist
    15, 16, 17, 18, 19,      // tay trái
    22, 23, 24, 25, 26,      // tay phải
    29, 30                   // đầu (yaw, pitch)
};

// policy index -> motor index trong IDL (dùng hàm này ở mọi nơi)
constexpr int MotorIdl(int policy_idx) {
    return kSdkToIdl[static_cast<size_t>(kPolicyToSdk[static_cast<size_t>(policy_idx)])];
}

// Khớp đầu (không do policy điều khiển — giữ 0 rad với gain nhẹ)
constexpr int kHeadYawIdl   = 29;
constexpr int kHeadPitchIdl = 30;

// Vị trí waist trong vector q thứ tự policy (dùng cho mimic torso)
constexpr int kWaistRollPolicyIdx = 12;
constexpr int kWaistYawPolicyIdx  = 13;

// Tư thế mặc định & action scale (từ ONNX metadata)
// target_q = default + action * scale
constexpr std::array<float, kNumJoints> kDefaultJointPos = {
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // chân trái
    -0.1f, 0.0f, 0.0f, 0.3f, -0.2f, 0.0f,   // chân phải
    0.0f,  0.0f,                            // waist
    0.35f, 0.18f, 0.0f, 0.87f, 0.0f,        // tay trái
    0.35f, -0.18f, 0.0f, 0.87f, 0.0f        // tay phải
};

constexpr std::array<float, kNumJoints> kActionScale = {
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // chân trái
    0.22f, 0.22f, 0.22f, 0.3475f, 0.3125f, 0.3125f,     // chân phải
    0.125f, 0.22f,                                      // waist
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f,   // tay trái
    0.15625f, 0.15625f, 0.15625f, 0.15625f, 0.15625f    // tay phải
};

// PD gains khi POLICY chạy — PHẢI đúng giá trị train.
constexpr std::array<float, kNumJoints> kKpTrain = {
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // chân trái
    100.0f, 100.0f, 100.0f, 100.0f, 40.0f, 40.0f,   // chân phải
    100.0f, 100.0f,                                 // waist
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f,              // tay trái
    40.0f, 40.0f, 40.0f, 40.0f, 40.0f               // tay phải
};

constexpr std::array<float, kNumJoints> kKdTrain = {
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 2.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f,
    2.0f, 2.0f, 2.0f, 2.0f, 2.0f
};

// Gait (train-locked)
constexpr float kGaitPeriodS  = 0.8f;
constexpr float kCmdGateNorm  = 0.1f;

// Tần số vòng lặp (khớp train: policy 50Hz, PD bám target 500Hz)
constexpr float kLoopDt        = 0.002f; // 500Hz DDS
constexpr int   kPolicyDecimation = 10;  // 500/10 = 50Hz inference
constexpr float kPolicyDt      = kLoopDt * kPolicyDecimation; // 0.02s

// Motion NPZ (mimic) — layout của dance*.npz.
constexpr int kMotionTorsoIdx  = 14;  // torso; IMU vật lý ở pelvis

} // namespace spec
