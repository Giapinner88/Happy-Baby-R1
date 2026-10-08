#pragma once

// ============================================================================
// Bộ dựng frame quan sát 83-D chuẩn (`r1_common_pd_base83_v1`).
//
// Tách riêng khỏi LocomotionController để mọi contract dùng lại đúng MỘT công
// thức gravity / thứ tự khớp / offset last_action. Viết lại lần thứ hai cho
// policy history là cách chắc chắn nhất để hai đường lệch nhau âm thầm.
//
// Hàm này KHÔNG áp overlay tay: overlay là bước sau, và chỉ contract nào khai
// SupportsArmOverlay() mới được phép chạy nó.
//
// Twin: unitree_mujoco/simulate_cpp/src/controllers/locomotion/FlatController
//       ::BuildBaseObservation83()
// Nguồn train: src/tasks/velocity/config/r1/history_contract.py
// ============================================================================

#include <algorithm>
#include <array>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "ControlContext.hpp"

namespace r1::obs83 {

// Thứ tự term đúng như observation manager nối actor group lúc train:
//   base_ang_vel(3) projected_gravity(3) command(3) phase(2)
//   joint_pos(24) joint_vel(24) actions(24)
inline constexpr int kDim = spec::kFlatObsSize;  // 83

inline constexpr int kAngVelOffset  = 0;
inline constexpr int kGravityOffset = 3;
inline constexpr int kCommandOffset = 6;
inline constexpr int kPhaseOffset   = 9;
inline constexpr int kJointPosOffset = 11;
inline constexpr int kJointVelOffset = 35;
inline constexpr int kActionsOffset  = 59;

// Dựng frame 83-D vào `out` (phải có đúng kDim phần tử).
// `last_action` là raw action của policy step TRƯỚC, đúng semantics lúc train.
inline void Build(const ControlContext& ctx,
                  const std::array<float, spec::kNumJoints>& default_q,
                  const std::array<float, spec::kNumJoints>& last_action,
                  std::vector<float>& out) {
    const RobotState& s = ctx.state;

    out[kAngVelOffset + 0] = s.gyro.x();
    out[kAngVelOffset + 1] = s.gyro.y();
    out[kAngVelOffset + 2] = s.gyro.z();

    out[kGravityOffset + 0] = s.projected_gravity.x();
    out[kGravityOffset + 1] = s.projected_gravity.y();
    out[kGravityOffset + 2] = s.projected_gravity.z();

    out[kCommandOffset + 0] = ctx.cmd_vx;
    out[kCommandOffset + 1] = ctx.cmd_vy;
    out[kCommandOffset + 2] = ctx.cmd_yaw;

    out[kPhaseOffset + 0] = ctx.gait_phase[0];
    out[kPhaseOffset + 1] = ctx.gait_phase[1];

    for (int i = 0; i < spec::kNumJoints; ++i) {
        out[kJointPosOffset + i] = s.q[i] - default_q[i];
        out[kJointVelOffset + i] = s.dq[i];
        out[kActionsOffset + i]  = last_action[i];
    }
}

// Gesture history phải mask frame 83-D trước khi pack term-major. Áp vào
// vector 332-D sau khi pack sẽ chạm nhầm term/timestep. last_action được giữ
// nguyên để policy vẫn thấy action thô của chính nó ở policy step trước.
inline void MaskArmState(std::vector<float>& frame, float arm_mask_keep) {
    const float keep = std::clamp(arm_mask_keep, 0.0f, 1.0f);
    for (int i = 14; i < spec::kNumJoints; ++i) {
        frame[kJointPosOffset + i] *= keep;
        frame[kJointVelOffset + i] *= keep;
    }
}

}  // namespace r1::obs83
