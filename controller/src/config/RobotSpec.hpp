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
    29, 30                   // Đầu theo SDK: pitch, yaw
};

// Hàm map từ index policy sang index motor IDL
constexpr int MotorIdl(int policy_idx) {
    return kSdkToIdl[static_cast<size_t>(kPolicyToSdk[static_cast<size_t>(policy_idx)])];
}

// Unitree R1 SDK: head pitch = IDL 29, head yaw = IDL 30.
// Giao thức UTL1 vẫn mang dữ liệu theo thứ tự ngữ nghĩa (yaw, pitch); sender
// chịu trách nhiệm đặt từng giá trị vào đúng IDL vật lý này.
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

// ─── Cổng an toàn cuối trước rt/lowcmd (LowCmdSender) ─────────────────────
// Tầm khớp chép từ MJCF unitree_mujoco/unitree_robots/r1/unitree_r1/xmls/r1.xml
// (cùng model dùng lúc train). ĐÂY KHÔNG PHẢI tầm cơ khí đã đo trên robot:
// encoder thật từng đọc ankle_roll 0.369 rad (motions/getup.npz, ghi lại từ
// captures_20260725/capture_016_L2_X.npz) trong khi MJCF chỉ cho ±0.2618.
constexpr std::array<float, kNumJoints> kJointRangeMin = {
    -2.93215f, -1.0472f,  -2.7402f, -0.174533f, -0.87266f, -0.2618f,    // Chân trái
    -2.93215f, -1.74533f, -2.7402f, -0.17453f,  -0.87266f, -0.261799f,  // Chân phải
    -0.5236f,  -2.618f,                                                 // Waist
    -3.1416f,  -0.22689f, -1.9199f, -0.97564f,  -1.9199f,               // Tay trái
    -3.1416f,  -2.47849f, -1.9199f, -0.97564f,  -1.9199f                // Tay phải
};
constexpr std::array<float, kNumJoints> kJointRangeMax = {
    2.54818f, 1.74533f, 2.7402f, 2.42601f, 0.57596f, 0.2618f,     // Chân trái
    2.54818f, 1.0472f,  2.7402f, 2.42601f, 0.57596f, 0.261799f,   // Chân phải
    0.5236f,  2.618f,                                             // Waist
    2.0944f,  2.4784f,  1.9199f, 2.1852f,  1.9199f,               // Tay trái
    2.0944f,  0.2268f,  1.9199f, 2.1852f,  1.9199f                // Tay phải
};

// Cổng CHỈ để chặn rác (NaN/Inf/1e30/lệch layout bộ nhớ), KHÔNG để ép khớp vào
// tầm cơ khí. Nới rộng mỗi phía kGuardSlack nên không một quỹ đạo/tư thế nào
// đang chạy bị cắt (biên rộng nhất hiện tại: ankle_roll 0.368 rad trong
// getup.npz) — cổng chạm được nghĩa là lệnh đã sai vài lần tầm khớp, lúc đó
// giá trị bị clamp cũng không đổi vị trí thật vì khớp đã tì vào chặn cơ khí.
// Siết bảng này về tầm thật là việc RIÊNG, phải đo dòng/nhiệt trên robot trước.
constexpr float kGuardSlack = 0.5f;

constexpr std::array<float, kNumJoints> OffsetAll(
        const std::array<float, kNumJoints>& base, float delta) {
    std::array<float, kNumJoints> out{};
    for (int i = 0; i < kNumJoints; ++i) out[i] = base[i] + delta;
    return out;
}
constexpr std::array<float, kNumJoints> kJointGuardMin =
    OffsetAll(kJointRangeMin, -kGuardSlack);
constexpr std::array<float, kNumJoints> kJointGuardMax =
    OffsetAll(kJointRangeMax, kGuardSlack);

// Provisional command envelope, rounded INWARD from 66,774 capture frames:
// pitch [-0.144674, +0.648844], yaw [-0.658669, +0.441480].
// Observed encoder travel is NOT mechanical-limit or load/thermal acceptance.
// Use asymmetric limits at the final sender for ALL head producers; no slack.
constexpr float kHeadPitchMin = -0.14f;
constexpr float kHeadPitchMax =  0.64f;
constexpr float kHeadYawMin   = -0.65f;
constexpr float kHeadYawMax   =  0.44f;

// ─── Trần cổ chân cho tư thế/phát lại (KHÔNG phải cổng an toàn ở trên) ────
// Ba chỗ trong Application từng mang ba con số rời không nguồn: sit ±0.44,
// ApplyAnkleFlat ±0.44, và cặp pitch -0.873/+0.576. Gom về đây kèm căn cứ.
//
// PITCH = đúng tầm MJCF. Cố ý giữ nguyên: trần này CHẶT hơn dữ liệu quan sát
// (getup.npz chạm 0.588) nên chỉ cắt 0.012 rad của một đoạn chuyển tiếp — sai
// về phía an toàn.
//
// ROLL thì ngược lại. MJCF ghi ±0.2618 nhưng con số đó KHÔNG mô tả phần cứng:
// 74.442 frame encoder ghi trên robot thật (motions/captures*) cho thấy cổ chân
// đi tới +0.3686/-0.3527 (trái) và +0.3429/-0.3390 (phải). Đỉnh của các đoạn đó
// TRƠN — tăng dần rồi quay đầu, không có plateau kiểu tì vào chặn cơ khí — và
// hai chân lệch zero ngược dấu ~0.05 rad, đúng dạng sai lệch calib từng chân.
// Bảng giới hạn trong GUI cũ KHÔNG phải nguồn độc lập: đối chiếu cả 8 khớp thì
// nó là ~90% của chính bảng MJCF.
//
// Nên trần roll = GIÁ TRỊ LỚN NHẤT ĐÃ CHỨNG MINH TRÊN PHẦN CỨNG (0.3686), làm
// tròn lên 0.37. Đây KHÔNG phải giới hạn cơ khí thật (chưa ai đo trên robot);
// nó chỉ bảo đảm không lệnh ra ngoài vùng robot đã thực sự đi được. Trần cũ
// 0.44 nằm ngoài mọi quan sát, và tư thế NGỒI giữ vô hạn ở đúng 0.44 với
// Kp=200 — đó mới là chỗ có thể tì motor, không phải getup.npz.
constexpr float kAnklePitchMin  = -0.87266f;
constexpr float kAnklePitchMax  =  0.57596f;
constexpr float kAnkleRollLimit =  0.37f;

// Trần gain/rate của cổng — chỉ chặn lỗi cấu hình thô, không phải giá trị vận
// hành. Cao nhất đang dùng: kp 220 (getup_kp_leg), kd 10 (head_pitch_kd),
// rate 30 rad/s (policy_rate_limit).
constexpr float kMaxCmdKp   = 500.0f;
constexpr float kMaxCmdKd   = 50.0f;
constexpr float kMaxCmdRate = 200.0f;

// Chu kỳ gait (gait period) cố định — khớp period của phase obs lúc train (0.6 s)
constexpr float kGaitPeriodS  = 0.6f;
constexpr float kCmdGateNorm  = 0.1f;

// Tần số điều khiển (500Hz DDS loop, 50Hz policy inference)
constexpr float kLoopDt        = 0.002f;
constexpr int   kPolicyDecimation = 10;
constexpr float kPolicyDt      = kLoopDt * kPolicyDecimation;

// Index của torso trong dance clip (dance*.npz)
constexpr int kMotionTorsoIdx  = 14;

} // namespace spec
