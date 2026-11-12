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
//
// Số action là tham số dựng: 12 (chân, các contract trên) hoặc 14 (chân + waist_roll,
// waist_yaw — contract U7 `r1_wb_legs_waist_gait_h4_v1_blind`). Action i điều khiển
// khớp policy i, nên eo (12, 13) nhận default + scale * action như chân; tay vẫn
// thuộc nguồn ngoài. Frame = 59 + số action (71 hoặc 73).
// 22 = 12 chân + 10 residual tay (U10, chỉ dùng qua WholeBodyLegsGaitController): frame
// thêm arm_q_ref[10], arm_dq_ref[10] sau actions -> 101.

#include <stdexcept>
#include <string>
#include <vector>

#include "BaseObservation83.hpp"
#include "PolicyController.hpp"

class WholeBodyLegsController : public PolicyController {
public:
    static constexpr int kLegActionDim = 12;
    static constexpr int kLegsWaistActionDim = 14;
    static constexpr int kLegsArmResActionDim = 22;
    static constexpr int kArmRefDim = 2 * spec::kNumArmJoints;  // arm_q_ref, arm_dq_ref
    static constexpr const char* kContract = "r1_wb_legs_v1_blind";

    explicit WholeBodyLegsController(int action_dim = kLegActionDim) : action_dim_(action_dim) {
        if (action_dim != kLegActionDim && action_dim != kLegsWaistActionDim &&
            action_dim != kLegsArmResActionDim)
            throw std::invalid_argument("WholeBodyLegsController: action_dim phai la 12, 14 hoac 22");
    }

    // Một frame obs: 59 phần tử đầu của frame 83-D + last action của policy
    // (+ tham chiếu tay với contract arm-residual).
    int FrameDim() const {
        return r1::obs83::kActionsOffset + action_dim_ + (arm_residual() ? kArmRefDim : 0);
    }
    int ObsSize() const override { return FrameDim(); }
    int ActionSize() const override { return action_dim_; }
    bool waist_in_policy() const { return action_dim_ == kLegsWaistActionDim; }
    bool arm_residual() const { return action_dim_ == kLegsArmResActionDim; }
    std::string Name() const override { return "WholeBodyLegs"; }

    // Tay nhận target teleop trực tiếp ở bước gửi lệnh; obs giữ q/dq tay thật.
    bool SupportsH4TeleopOverlay() const override { return true; }
    // Policy chỉ xuất action chân (+ eo). Gesture có thể ghi đè target tay trực tiếp;
    // q/dq tay thật vẫn nằm trong observation nên không mask chúng.
    bool SupportsDirectArmGestureOverlay() const override { return true; }

    // Chu kỳ phase lúc train (metadata gait_period_s); Application so với tuning.
    float metadata_gait_period_s() const { return gait_period_s_; }

protected:
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        BuildFrame(ctx, obs.begin());
    }

    void ValidateExtraMetadata(const std::string& model_path) override {
        ValidateWholeBodyMetadata(model_path, kContract, FrameLayout());
    }

    static constexpr const char* kArmJointNames =
        "left_shoulder_pitch_joint,left_shoulder_roll_joint,left_shoulder_yaw_joint,"
        "left_elbow_joint,left_wrist_roll_joint,right_shoulder_pitch_joint,"
        "right_shoulder_roll_joint,right_shoulder_yaw_joint,right_elbow_joint,"
        "right_wrist_roll_joint";

    std::string FrameLayout() const {
        return "base_ang_vel,projected_gravity,command,phase,joint_pos[24],joint_vel[24],actions[" +
               std::to_string(action_dim_) + "]" +
               (arm_residual() ? ",arm_q_ref[10],arm_dq_ref[10]" : "");
    }

    // 59 + action_dim_ phần tử đầu của một frame (dùng chung với các contract history
    // gait H4; contract arm-residual tự ghi tham chiếu tay phía sau).
    template <typename It>
    void BuildFrame(const ControlContext& ctx, It out) {
        frame_.assign(r1::obs83::kDim, 0.0f);
        r1::obs83::Build(ctx, default_q_, last_action_, frame_);
        std::copy(frame_.begin(), frame_.begin() + r1::obs83::kActionsOffset, out);
        for (int i = 0; i < action_dim_; ++i)
            out[r1::obs83::kActionsOffset + i] = last_action_[i];
    }

    void ValidateWholeBodyMetadata(const std::string& model_path, const char* contract,
                                   const std::string& layout) {
        RequireMetadata(model_path, "policy_contract", contract);
        RequireMetadata(model_path, "observation_layout", layout);
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
                        std::string("left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,"
                                    "left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
                                    "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,"
                                    "right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint") +
                            (waist_in_policy() ? ",waist_roll_joint,waist_yaw_joint" : "") +
                            (arm_residual() ? std::string(",") + kArmJointNames : ""));
        std::string period;
        if (!policy_.ReadMetadataString("gait_period_s", period))
            throw std::runtime_error("ONNX thieu metadata gait_period_s: " + model_path);
        gait_period_s_ = std::stof(period);
    }

private:
    int action_dim_;
    std::vector<float> frame_;
    float gait_period_s_ = 0.0f;
};
