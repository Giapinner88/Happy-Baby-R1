#pragma once

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../estimation/StateEstimator.hpp"
#include "OnnxPolicy.hpp"

// Dữ liệu điều khiển cung cấp cho controller tại mỗi bước
struct ControlContext {
    const RobotState& state;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_yaw = 0.0f;
    std::array<float, 2> gait_phase{0.0f, 0.0f};

    // Unified 105-D contract. Vị trí/vận tốc reference là góc tuyệt đối của 10
    // khớp tay (policy idx 14..23), đúng thứ tự metadata observation_layout.
    float crouch_alpha = 0.0f;
    float crouch_alpha_rate = 0.0f;
    std::array<float, spec::kNumArmJoints> arm_q_ref{};
    std::array<float, spec::kNumArmJoints> arm_dq_ref{};

    // Overlay động tác tay (mặc định TẮT -> obs dựng y hệt khi không có gesture).
    // arm_mask_keep = (1 - weight): che q_rel/dq của tay (giữ last_action).
    // gravity_x_bias / cmd_vx_bias: feedforward bù thăng bằng, CHỈ chạm obs[3]/obs[6]
    // (KHÔNG đi qua cmd_vx thật để không nhiễu cmd_norm/gait).
    bool  arm_override_active = false;
    float arm_mask_keep       = 1.0f;
    float gravity_x_bias      = 0.0f;
    float cmd_vx_bias         = 0.0f;

};

// Lớp cơ sở cho các bộ điều khiển dựa trên chính sách (Policy Controller)
class PolicyController {
public:
    virtual ~PolicyController() = default;

    void Init(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& options, const Tuning& tuning) {
        policy_.Load(model_path, env, options, ObsSize());

        const std::string expected_contract = ExpectedPolicyContract();
        if (!expected_contract.empty()) {
            std::string actual_contract, actual_obs, actual_action, actual_layout;
            const bool valid_contract = policy_.ReadMetadataString("policy_contract", actual_contract) &&
                                        actual_contract == expected_contract;
            const bool valid_dims = policy_.ReadMetadataString("observation_dim", actual_obs) &&
                                    policy_.ReadMetadataString("action_dim", actual_action) &&
                                    actual_obs == std::to_string(ObsSize()) &&
                                    actual_action == std::to_string(spec::kNumJoints);
            const std::string expected_layout = ExpectedObservationLayout();
            const bool valid_layout = expected_layout.empty() ||
                                      (policy_.ReadMetadataString("observation_layout", actual_layout) &&
                                       actual_layout == expected_layout);
            if (!valid_contract || !valid_dims || !valid_layout) {
                throw std::runtime_error(
                    "ONNX contract mismatch: expected '" + expected_contract +
                    "' with observation_dim=" + std::to_string(ObsSize()) +
                    " and action_dim=" + std::to_string(spec::kNumJoints) +
                    ", refusing to run " + model_path);
            }
        }

        default_q_    = spec::kDefaultJointPos;
        action_scale_ = spec::kActionScale;
        kp_           = spec::kKpTrain;
        kd_           = spec::kKdTrain;

        // Ưu tiên nạp thông số từ ONNX metadata.
        int missing = 0;
        missing += !LoadOrFallback("default_joint_pos", default_q_);
        missing += !LoadOrFallback("action_scale", action_scale_);
        missing += !LoadOrFallback("joint_stiffness", kp_);
        missing += !LoadOrFallback("joint_damping", kd_);

        // Bắt buộc có metadata để tránh sai lệch gain khi deploy (gây mất thăng bằng).
        if (missing > 0) {
            std::cerr << "\n"
                      << "==========================================================\n"
                      << " [LỖI] " << model_path << "\n"
                      << " ONNX thiếu " << missing << "/4 khoá metadata bắt buộc.\n"
                      << " Deploy KHÔNG được phép đoán gains -> từ chối chạy policy này.\n"
                      << "\n"
                      << " Cách sửa: export lại từ máy train (runner đã được vá để gắn\n"
                      << " metadata vào policy.onnx), hoặc backfill metadata vào file.\n"
                      << "==========================================================\n\n";
            throw std::runtime_error("ONNX thiếu metadata: " + model_path);
        }

        for (int i = 0; i < spec::kNumJoints; ++i) {
            kp_[i] *= tuning.policy_kp_scale;
            kd_[i] *= tuning.policy_kd_scale;
        }

        std::cout << "[" << Name() << "] Kp[hip]=" << kp_[0] << " Kp[ankle]=" << kp_[4]
                  << " Kp[tay]=" << kp_[14] << " Kd=" << kd_[0] << "\n";
    }

    virtual void Reset(const RobotState& /*state*/) {
        last_action_.fill(0.0f);
    }

    // Thực hiện 1 bước inference (chạy ở 50Hz)
    const std::array<float, spec::kNumJoints>& Step(const ControlContext& ctx) {
        std::vector<float> obs(ObsSize(), 0.0f);
        BuildObservation(ctx, obs);
        std::vector<float> action = policy_.Infer(obs);
        for (int i = 0; i < spec::kNumJoints; ++i) {
            last_action_[i] = action[i];
            target_q_[i] = default_q_[i] + action[i] * action_scale_[i];
        }
        return target_q_;
    }

    virtual bool IsFinished() const { return false; }
    virtual int ObsSize() const = 0;
    virtual std::string Name() const = 0;
    virtual bool UsesArmReference() const { return false; }

    const std::array<float, spec::kNumJoints>& default_q() const { return default_q_; }
    const std::array<float, spec::kNumJoints>& kp() const { return kp_; }
    const std::array<float, spec::kNumJoints>& kd() const { return kd_; }
    const std::array<float, spec::kNumJoints>& last_action() const { return last_action_; }

protected:
    virtual void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) = 0;
    virtual std::string ExpectedPolicyContract() const { return ""; }
    virtual std::string ExpectedObservationLayout() const { return ""; }

    // Đọc thông số mảng từ ONNX metadata hoặc trả về false.
    bool LoadOrFallback(const char* key, std::array<float, spec::kNumJoints>& out) {
        if (policy_.ReadMetadataArray(key, out)) {
            std::cout << "[" << Name() << "] " << key << ": Loaded from ONNX metadata.\n";
            return true;
        }
        std::cout << "[" << Name() << "] " << key << ": KHÔNG có trong metadata.\n";
        return false;
    }

    OnnxPolicy policy_;
    std::array<float, spec::kNumJoints> default_q_{};
    std::array<float, spec::kNumJoints> action_scale_{};
    std::array<float, spec::kNumJoints> kp_{};
    std::array<float, spec::kNumJoints> kd_{};
    std::array<float, spec::kNumJoints> last_action_{};
    std::array<float, spec::kNumJoints> target_q_{};
};
