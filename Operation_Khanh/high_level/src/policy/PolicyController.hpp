#pragma once
/**
 * PolicyController.hpp — Lớp cơ sở cho các controller chạy policy RL.
 */

#include <array>
#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../estimation/StateEstimator.hpp"
#include "OnnxPolicy.hpp"

// Thông tin điều khiển trong 1 bước
struct ControlContext {
    const RobotState& state;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_yaw = 0.0f;
    std::array<float, 2> gait_phase{0.0f, 0.0f};
};

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

        // Nạp metadata từ ONNX (nếu có), fallback về RobotSpec.hpp
        LoadOrFallback("default_joint_pos", default_q_);
        LoadOrFallback("action_scale", action_scale_);
        LoadOrFallback("joint_stiffness", kp_);
        LoadOrFallback("joint_damping", kd_);

        // Scale thí nghiệm
        for (int i = 0; i < spec::kNumJoints; ++i) {
            kp_[i] *= tuning.policy_kp_scale;
            kd_[i] *= tuning.policy_kd_scale;
        }

        std::cout << "[" << Name() << "] Kp[hip]=" << kp_[0] << " Kp[ankle]=" << kp_[4]
                  << " Kp[tay]=" << kp_[14] << " Kd=" << kd_[0]
                  << " (phải khớp train: 100/40/40/2.0)\n";
    }

    virtual void Reset(const RobotState& /*state*/) {
        last_action_.fill(0.0f);
    }

    // Gọi ở 50Hz. Trả về target_q.
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

protected:
    virtual void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) = 0;

    // Đọc metadata và cập nhật giá trị
    void LoadOrFallback(const char* key, std::array<float, spec::kNumJoints>& out) {
        if (policy_.ReadMetadataArray(key, out)) {
            std::cout << "[" << Name() << "] " << key << ": từ ONNX metadata.\n";
        } else {
            std::cout << "[" << Name() << "] " << key
                      << ": KHÔNG có trong metadata -> dùng fallback RobotSpec.hpp.\n";
        }
    }

    OnnxPolicy policy_;
    std::array<float, spec::kNumJoints> default_q_{};
    std::array<float, spec::kNumJoints> action_scale_{};
    std::array<float, spec::kNumJoints> kp_{};
    std::array<float, spec::kNumJoints> kd_{};
    std::array<float, spec::kNumJoints> last_action_{};
    std::array<float, spec::kNumJoints> target_q_{};
};
