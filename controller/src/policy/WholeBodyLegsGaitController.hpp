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
// Golden: tests/wb_legs_gait_golden_test.cpp (scripts/make_wb_legs_gait_golden.py).

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../gait/GaitModeScheduler.hpp"
#include "WholeBodyLegsController.hpp"

class WholeBodyLegsGaitController final : public WholeBodyLegsController {
public:
    static constexpr int kHistorySteps = 4;
    static constexpr int kGaitDim = 3;
    static constexpr int kGaitObsDim = kObsDim * kHistorySteps + kGaitDim;  // 287
    static constexpr const char* kGaitContract = "r1_wb_legs_gait_h4_v1_blind";

    int ObsSize() const override { return kGaitObsDim; }
    std::string Name() const override { return "WholeBodyLegsGaitH4"; }

    void Reset(const RobotState& state) override {
        WholeBodyLegsController::Reset(state);
        history_valid_ = false;
        gait_.Reset();
    }

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

        std::array<float, kObsDim> frame{};
        BuildFrame71(ctx, frame.begin());
        if (!history_valid_) {
            history_.fill(frame);  // repeat_first, như CircularBuffer lúc reset
            history_valid_ = true;
        } else {
            for (int k = 0; k + 1 < kHistorySteps; ++k) history_[k] = history_[k + 1];
            history_[kHistorySteps - 1] = frame;
        }

        // Term-major: với mỗi term, các bước oldest -> newest.
        int dst = 0;
        int term_start = 0;
        for (const int width : kTermDims) {
            for (int step = 0; step < kHistorySteps; ++step)
                for (int k = 0; k < width; ++k)
                    obs[dst++] = history_[step][term_start + k];
            term_start += width;
        }
        const auto one_hot = gait_.OneHot();
        for (int k = 0; k < kGaitDim; ++k) obs[dst++] = one_hot[k];
        last_obs_ = obs;
    }

    void ValidateExtraMetadata(const std::string& model_path) override {
        ValidateWholeBodyMetadata(model_path, kGaitContract,
                                  std::string(kFrameLayout) + " x4 steps,gait_one_hot[3]");
        RequireMetadata(model_path, "actor_history_steps", std::to_string(kHistorySteps));
        RequireMetadata(model_path, "actor_history_order", "term_major_oldest_to_newest");
        RequireMetadata(model_path, "actor_history_padding", "repeat_first");
        RequireMetadata(model_path, "actor_history_includes_current", "true");
        RequireMetadata(model_path, "actor_history_terms", kFrameLayout);
        RequireMetadata(model_path, "gait_one_hot_modes", "STAND,WALK,W2S");
        RequireMetadata(model_path, "gait_one_hot_position", "appended_after_history");
        RequireMetadata(model_path, "gait_stability_joints", "leg joints (12)");

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
    static constexpr std::array<int, 7> kTermDims = {3, 3, 3, 2, 24, 24, kActionDim};
    static_assert(3 + 3 + 3 + 2 + 24 + 24 + kActionDim == kObsDim, "term dims");

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

    std::array<std::array<float, kObsDim>, kHistorySteps> history_{};
    bool history_valid_ = false;
    GaitModeConfig gait_config_{};
    GaitModeScheduler gait_{};
    std::vector<float> last_obs_;
};
