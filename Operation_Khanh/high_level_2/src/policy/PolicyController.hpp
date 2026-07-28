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
};

// Lớp cơ sở cho các bộ điều khiển dựa trên chính sách (Policy Controller)
class PolicyController {
public:
    virtual ~PolicyController() = default;

    void Init(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& options, const Tuning& tuning) {
        policy_.Load(model_path, env, options, ObsSize());

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

    const std::array<float, spec::kNumJoints>& default_q() const { return default_q_; }
    const std::array<float, spec::kNumJoints>& kp() const { return kp_; }
    const std::array<float, spec::kNumJoints>& kd() const { return kd_; }
    const std::array<float, spec::kNumJoints>& last_action() const { return last_action_; }

protected:
    virtual void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) = 0;

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
