#pragma once

// Contract `r1_wb_legs_v1_blind` (nhánh loco, thí nghiệm U1, Isaac Lab):
// policy chỉ điều khiển 12 khớp chân; eo giữ default_q, tay thuộc nguồn ngoài
// (teleop H4 / default) — thiết kế "D8 = A" trong loco docs/loco/whole_body_contract.md.
//
// Observation 71-D, không scale, normalizer đã gói trong ONNX:
//   base_ang_vel(3) projected_gravity(3) command(3) phase(2)
//   joint_pos(24) joint_vel(24) actions(12)
// = 59 phần tử đầu của frame 83-D chuẩn + 12 raw action của policy step trước.
// phase = sin/cos(2π t / period), bằng 0 khi ‖(vx,vy,yaw)‖ < 0.1 (giống
// GaitScheduler::PhaseObs). Nguồn train: loco/r1/isaac/env_cfg.py build() +
// loco/r1/isaac/mdp.py phase().
//
// "blind": actor KHÔNG thấy tham chiếu tay; lúc train tay bị kéo theo tham chiếu
// ngẫu nhiên và q/dq tay thật nằm trong obs. Vì vậy overlay tay KHÔNG mask obs
// (ctx.arm_override_active bị bỏ qua) — khác contract H4 history.

#include <string>
#include <vector>

#include "BaseObservation83.hpp"
#include "PolicyController.hpp"

class WholeBodyLegsController : public PolicyController {
public:
    static constexpr int kActionDim = 12;
    static constexpr int kObsDim = r1::obs83::kActionsOffset + kActionDim;  // 71
    static constexpr const char* kContract = "r1_wb_legs_v1_blind";

    int ObsSize() const override { return kObsDim; }
    int ActionSize() const override { return kActionDim; }
    std::string Name() const override { return "WholeBodyLegs"; }

    // Tay nhận target teleop trực tiếp ở bước gửi lệnh; obs giữ q/dq tay thật.
    bool SupportsH4TeleopOverlay() const override { return true; }
    // Policy chỉ xuất 12 action chân. Gesture có thể ghi đè target tay trực tiếp;
    // q/dq tay thật vẫn nằm trong observation 71-D nên không mask chúng.
    bool SupportsDirectArmGestureOverlay() const override { return true; }

    // Chu kỳ phase lúc train (metadata gait_period_s); Application so với tuning.
    float metadata_gait_period_s() const { return gait_period_s_; }

protected:
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        frame_.assign(r1::obs83::kDim, 0.0f);
        r1::obs83::Build(ctx, default_q_, last_action_, frame_);
        std::copy(frame_.begin(), frame_.begin() + r1::obs83::kActionsOffset, obs.begin());
        for (int i = 0; i < kActionDim; ++i)
            obs[r1::obs83::kActionsOffset + i] = last_action_[i];
    }

    void ValidateExtraMetadata(const std::string& model_path) override {
        RequireMetadata(model_path, "policy_contract", kContract);
        RequireMetadata(model_path, "observation_layout",
                        "base_ang_vel,projected_gravity,command,phase,"
                        "joint_pos[24],joint_vel[24],actions[12]");
        RequireMetadata(model_path, "joint_names",
                        "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,"
                        "left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
                        "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,"
                        "right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint,"
                        "waist_roll_joint,waist_yaw_joint,"
                        "left_shoulder_pitch_joint,left_shoulder_roll_joint,"
                        "left_shoulder_yaw_joint,left_elbow_joint,left_wrist_roll_joint,"
                        "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
                        "right_shoulder_yaw_joint,right_elbow_joint,right_wrist_roll_joint");
        RequireMetadata(model_path, "action_joint_names",
                        "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,"
                        "left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
                        "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,"
                        "right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint");
        std::string period;
        if (!policy_.ReadMetadataString("gait_period_s", period))
            throw std::runtime_error("ONNX thieu metadata gait_period_s: " + model_path);
        gait_period_s_ = std::stof(period);
    }

private:
    std::vector<float> frame_;
    float gait_period_s_ = 0.0f;
};
