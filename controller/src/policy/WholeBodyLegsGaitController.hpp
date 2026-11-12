#pragma once

// Contract `r1_wb_legs_gait_h4_v1_blind` (nhánh loco, thí nghiệm U4, Isaac Lab):
// cùng frame 71-D và 12 action chân như `r1_wb_legs_v1_blind`, thêm
//   - history 4 bước, term-major, oldest -> newest, bước đầu lặp lại
//     (Isaac Lab ObservationGroup history_length=4, flatten_history_dim);
//   - one-hot STAND/WALK/W2S của FSM, nối MỘT lần sau history (group `gait`,
//     không nhiễu, không stack) -> 4 x 71 + 3 = 287.
// FSM: loco/r1/gait_fsm.py (port từ task armfree), cập nhật mỗi policy step
// TRƯỚC khi dựng obs (GaitCommand.compute chạy trước observation manager);
// ổn định = gyro lọc + RMS vận tốc 12 khớp chân; vào STAND cần nghiêng <= 5°;
// push exit khỏi STAND. Mọi hằng số FSM được đối chiếu với metadata ONNX.
//
// Contract `r1_wb_legs_waist_gait_h4_v1_blind` (U7): giống hệt, nhưng policy xuất
// 14 action (12 chân + waist_roll, waist_yaw) nên frame 73-D, obs 4 x 73 + 3 = 295;
// FSM, history, one-hot không đổi (ổn định vẫn xét 12 khớp chân).
//
// Contract `r1_wb_legs_armres_gait_h4_v1_ref` (U10): 22 action = 12 chân + 10 residual
// tay. Frame 101 = 59 + actions[22] + arm_q_ref[10] + arm_dq_ref[10]; obs = 4 x 101 +
// one-hot 3 + arm_authority g 1 = 408. Tham chiếu tay: ctx.arm_desired (teleop/gesture,
// default khi không ai cầm) qua ArmReferenceFilter ở 50 Hz (trần train 4 rad/s, 14
// rad/s^2), cập nhật TRƯỚC obs như ArmReferenceCommand. Target tay =
// clamp(q_ref + g * arm_residual_rad * clip(a, -1, 1), soft limit 0.9 của URDF) với
// cùng q_ref trong obs của bước đó (loco/r1/isaac/mdp.py WholeBodyAction). Eo giữ default
// (kp/kd 200/4 trong metadata). Nguồn: loco experiments/r1_loco/u10/experiment.md.
// Golden: tests/wb_legs_gait_golden_test.cpp (scripts/make_wb_legs_gait_golden.py).

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../gait/GaitModeScheduler.hpp"
#include "ArmReferenceFilter.hpp"
#include "WholeBodyLegsController.hpp"

class WholeBodyLegsGaitController final : public WholeBodyLegsController {
public:
    static constexpr int kHistorySteps = 4;
    static constexpr int kGaitDim = 3;
    static constexpr const char* kGaitContract = "r1_wb_legs_gait_h4_v1_blind";
    static constexpr const char* kWaistGaitContract = "r1_wb_legs_waist_gait_h4_v1_blind";
    static constexpr const char* kArmResGaitContract = "r1_wb_legs_armres_gait_h4_v1_ref";
    static constexpr float kArmSoftLimitFactor = 0.9f;  // Isaac soft_joint_pos_limit_factor (U10 env.yaml)

    explicit WholeBodyLegsGaitController(int action_dim = kLegActionDim)
        : WholeBodyLegsController(action_dim) {}

    // 287 (12 action), 295 (14 action) hoặc 408 (22 action, + arm_authority).
    int ObsSize() const override {
        return FrameDim() * kHistorySteps + kGaitDim + (arm_residual() ? 1 : 0);
    }
    std::string Name() const override {
        if (arm_residual()) return "WholeBodyLegsArmResGaitH4";
        return waist_in_policy() ? "WholeBodyLegsWaistGaitH4" : "WholeBodyLegsGaitH4";
    }
    const char* contract() const {
        if (arm_residual()) return kArmResGaitContract;
        return waist_in_policy() ? kWaistGaitContract : kGaitContract;
    }
    bool ArmsFromReference() const override { return arm_residual(); }
    // Arm-residual: action 12..21 điều khiển tay 14..23; eo (12, 13) giữ bằng PD.
    bool JointIsAction(int joint) const override {
        if (!arm_residual()) return WholeBodyLegsController::JointIsAction(joint);
        return joint < kLegActionDim || joint >= kArmBegin;
    }

    void Reset(const RobotState& state) override {
        WholeBodyLegsController::Reset(state);
        history_valid_ = false;
        gait_.Reset();
        // Như ArmReferenceCommand.reset: q_ref = default tay, dq_ref = 0; trần filter
        // cố định ở giá trị lúc đánh giá (hardware ceilings).
        ArmReferenceFilter::Arm default_arm{};
        for (int k = 0; k < spec::kNumArmJoints; ++k) default_arm[k] = default_q_[kArmBegin + k];
        arm_filter_.Init(default_arm, ArmReferenceFilter::kTrainMaxVelRadS,
                         ArmReferenceFilter::kTrainMaxAccRadS2);
        arm_authority_ = 0.0f;
    }

    // Tham chiếu tay và g dùng ở Step() gần nhất (test / telemetry).
    const ArmReferenceFilter& arm_reference() const { return arm_filter_; }
    float arm_authority() const { return arm_authority_; }
    float arm_residual_rad() const { return arm_residual_rad_; }

    const GaitModeScheduler& gait() const { return gait_; }
    const GaitModeConfig& gait_config() const { return gait_config_; }
    // Obs vừa đưa vào ONNX ở Step() gần nhất (golden test / chẩn đoán).
    const std::vector<float>& last_obs() const { return last_obs_; }

protected:
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const std::array<float, 3> gyro = {
            ctx.state.gyro_raw.x(), ctx.state.gyro_raw.y(), ctx.state.gyro_raw.z()};
        const std::array<float, 3> gravity = {
            ctx.state.projected_gravity.x(), ctx.state.projected_gravity.y(),
            ctx.state.projected_gravity.z()};
        gait_.Update(std::hypot(ctx.cmd_vx, ctx.cmd_vy), std::abs(ctx.cmd_yaw), gyro,
                     ctx.state.dq_raw, gravity, spec::kPolicyDt);

        if (arm_residual()) {
            arm_filter_.Update(ctx.arm_desired_valid ? ctx.arm_desired : arm_filter_.default_q(),
                               spec::kPolicyDt);
            // Không hữu hạn -> 0 (tay chỉ theo tham chiếu): an toàn hơn là trần 1.
            arm_authority_ = std::isfinite(ctx.arm_authority)
                ? std::clamp(ctx.arm_authority, 0.0f, 1.0f) : 0.0f;
        }

        std::vector<float> frame(FrameDim());
        BuildFrame(ctx, frame.begin());
        if (arm_residual()) {
            auto ref = frame.begin() + r1::obs83::kActionsOffset + ActionSize();
            ref = std::copy(arm_filter_.q().begin(), arm_filter_.q().end(), ref);
            std::copy(arm_filter_.dq().begin(), arm_filter_.dq().end(), ref);
        }
        if (!history_valid_) {
            history_.fill(frame);  // repeat_first, như CircularBuffer lúc reset
            history_valid_ = true;
        } else {
            for (int k = 0; k + 1 < kHistorySteps; ++k) history_[k].swap(history_[k + 1]);
            history_[kHistorySteps - 1].swap(frame);
        }

        // Term-major: với mỗi term, các bước oldest -> newest.
        std::vector<int> term_dims = {3, 3, 3, 2, 24, 24, ActionSize()};
        if (arm_residual()) term_dims.insert(term_dims.end(), {spec::kNumArmJoints, spec::kNumArmJoints});
        int dst = 0;
        int term_start = 0;
        for (const int width : term_dims) {
            for (int step = 0; step < kHistorySteps; ++step)
                for (int k = 0; k < width; ++k)
                    obs[dst++] = history_[step][term_start + k];
            term_start += width;
        }
        const auto one_hot = gait_.OneHot();
        for (int k = 0; k < kGaitDim; ++k) obs[dst++] = one_hot[k];
        if (arm_residual()) obs[dst++] = arm_authority_;
        last_obs_ = obs;
    }

    void ComputeTargets(const std::vector<float>& action) override {
        if (!arm_residual()) { WholeBodyLegsController::ComputeTargets(action); return; }
        for (int i = 0; i < kLegActionDim; ++i)
            target_q_[i] = default_q_[i] + action[i] * action_scale_[i];
        for (int i = kLegActionDim; i < kArmBegin; ++i) target_q_[i] = default_q_[i];  // eo
        const float bound = arm_authority_ * arm_residual_rad_;
        for (int k = 0; k < spec::kNumArmJoints; ++k) {
            const int j = kArmBegin + k;
            const float residual = bound * std::clamp(action[kLegActionDim + k], -1.0f, 1.0f);
            target_q_[j] = std::clamp(arm_filter_.q()[k] + residual, arm_soft_lo_[k], arm_soft_hi_[k]);
        }
    }

    void ValidateExtraMetadata(const std::string& model_path) override {
        ValidateWholeBodyMetadata(model_path, contract(),
                                  FrameLayout() + " x4 steps,gait_one_hot[3]" +
                                      (arm_residual() ? ",arm_authority[1]" : ""));
        RequireMetadata(model_path, "actor_history_steps", std::to_string(kHistorySteps));
        RequireMetadata(model_path, "actor_history_order", "term_major_oldest_to_newest");
        RequireMetadata(model_path, "actor_history_padding", "repeat_first");
        RequireMetadata(model_path, "actor_history_includes_current", "true");
        RequireMetadata(model_path, "actor_history_terms", FrameLayout());
        RequireMetadata(model_path, "gait_one_hot_modes", "STAND,WALK,W2S");
        RequireMetadata(model_path, "gait_one_hot_position", "appended_after_history");
        RequireMetadata(model_path, "gait_stability_joints", "leg joints (12)");
        if (arm_residual()) ValidateArmResidualMetadata(model_path);
        if (waist_in_policy()) {
            // Overlay teleop/gesture chỉ ghi tay (14..23); eo phải là của policy.
            RequireMetadata(model_path, "upper_body_reference_joints",
                            "left_shoulder_pitch_joint,left_shoulder_roll_joint,"
                            "left_shoulder_yaw_joint,left_elbow_joint,left_wrist_roll_joint,"
                            "right_shoulder_pitch_joint,right_shoulder_roll_joint,"
                            "right_shoulder_yaw_joint,right_elbow_joint,right_wrist_roll_joint");
        }

        // Ngưỡng FSM lấy đúng giá trị lúc train; thiếu hoặc lệch là từ chối chạy.
        GaitModeConfig c;
        ReadPair(model_path, "gait_stop_request", c.stop_request_lin, c.stop_request_ang);
        ReadPair(model_path, "gait_move_request", c.move_request_lin, c.move_request_ang);
        c.settle_gyro_norm = ReadFloat(model_path, "gait_stability_ang_vel_max");
        c.settle_joint_velocity_rms = ReadFloat(model_path, "gait_stability_joint_vel_rms_max");
        // Dwell là một phần của contract one-hot (STAND xuất hiện sau đúng bấy nhiêu
        // giây lúc train), không lấy gait_settle_dwell_s của locomotion.yaml.
        c.settle_dwell_s = ReadFloat(model_path, "gait_t_settle_s");
        c.w2s_timeout_s = ReadFloat(model_path, "gait_w2s_timeout_s");
        c.stability_filter_tau_s = ReadFloat(model_path, "gait_stability_filter_tau_s");
        c.stand_upright_enabled = true;
        c.stand_entry_tilt_max_rad = ReadFloat(model_path, "gait_stand_entry_tilt_max_rad");
        c.stand_push_exit_enabled = true;
        c.push_exit_tilt_immediate_rad =
            ReadFloat(model_path, "gait_push_exit_tilt_immediate_rad");
        c.push_exit_tilt_sustained_rad =
            ReadFloat(model_path, "gait_push_exit_tilt_sustained_rad");
        c.push_exit_gyro_immediate = ReadFloat(model_path, "gait_push_exit_gyro_immediate");
        c.push_exit_gyro_sustained = ReadFloat(model_path, "gait_push_exit_gyro_sustained");
        c.push_exit_sustain_s = ReadFloat(model_path, "gait_push_exit_sustain_s");
        c.stability_legs12_only = true;
        // Bước init: loco tiếp tục xét chuyển trạng thái trong cùng bước, bản C++
        // return ngay; hai cách cho cùng mode vì điều kiện vào STAND (ổn định,
        // nghiêng <= 5°) và push exit / settling loại trừ nhau ở bước đó.
        gait_.Configure(c);  // ném lỗi nếu giá trị không hợp lệ
        gait_config_ = c;
        std::cout << "[" << Name() << "] FSM tu metadata: settle_dwell=" << c.settle_dwell_s
                  << "s stop<" << c.stop_request_lin << " move>" << c.move_request_lin
                  << " stab gyro<" << c.settle_gyro_norm << " legs12 rms<"
                  << c.settle_joint_velocity_rms << "\n";
    }

private:
    static constexpr int kArmBegin = 14;  // policy idx khớp tay đầu tiên

    void ValidateArmResidualMetadata(const std::string& model_path) {
        RequireMetadata(model_path, "arm_residual_joints", kArmJointNames);
        RequireMetadata(model_path, "arm_target_rule",
                        "clamp(arm_q_ref + g * arm_residual_rad * clip(action, -1, 1), soft joint limits)");
        RequireMetadata(model_path, "arm_q_ref_source",
                        "teleop arm target after ArmReferenceFilter (the actor's arm_q_ref, arm_dq_ref)");
        RequireMetadata(model_path, "arm_authority_position", "appended_after_gait_one_hot");
        RequireMetadata(model_path, "arm_authority_range", "0,1");
        arm_residual_rad_ = ReadFloat(model_path, "arm_residual_rad");
        if (!(arm_residual_rad_ > 0.0f && arm_residual_rad_ <= 0.5f))
            throw std::runtime_error("metadata arm_residual_rad phai trong (0, 0.5]: " + model_path);
        // Isaac soft limit = giữa ± factor x nửa dải, dải từ URDF (= USD train = RobotSpec).
        for (int k = 0; k < spec::kNumArmJoints; ++k) {
            const float lo = spec::kJointRangeMin[kArmBegin + k], hi = spec::kJointRangeMax[kArmBegin + k];
            const float mid = 0.5f * (lo + hi), half = 0.5f * (hi - lo) * kArmSoftLimitFactor;
            arm_soft_lo_[k] = mid - half;
            arm_soft_hi_[k] = mid + half;
        }
        std::cout << "[" << Name() << "] arm residual " << arm_residual_rad_
                  << " rad x g, tham chieu qua ArmReferenceFilter "
                  << ArmReferenceFilter::kTrainMaxVelRadS << " rad/s\n";
    }

    float ReadFloat(const std::string& model_path, const char* key) {
        std::string value;
        if (!policy_.ReadMetadataString(key, value))
            throw std::runtime_error(std::string("ONNX thieu metadata ") + key + ": " + model_path);
        const float f = std::stof(value);
        if (!std::isfinite(f))
            throw std::runtime_error(std::string("metadata khong huu han ") + key);
        return f;
    }

    void ReadPair(const std::string& model_path, const char* key, float& lin, float& ang) {
        std::string value;
        if (!policy_.ReadMetadataString(key, value))
            throw std::runtime_error(std::string("ONNX thieu metadata ") + key + ": " + model_path);
        const auto comma = value.find(',');
        if (comma == std::string::npos)
            throw std::runtime_error(std::string("metadata ") + key + " phai la 'lin,ang'");
        lin = std::stof(value.substr(0, comma));
        ang = std::stof(value.substr(comma + 1));
    }

    std::array<std::vector<float>, kHistorySteps> history_{};
    bool history_valid_ = false;
    GaitModeConfig gait_config_{};
    GaitModeScheduler gait_{};
    std::vector<float> last_obs_;
    ArmReferenceFilter arm_filter_{};
    float arm_authority_ = 0.0f;
    float arm_residual_rad_ = 0.0f;
    std::array<float, spec::kNumArmJoints> arm_soft_lo_{}, arm_soft_hi_{};
};
