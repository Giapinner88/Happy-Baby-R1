#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

#include "onnx/OnnxModel.hpp"
#include "onnx/PolicyContract.hpp"
#include "runtime/R1Config.hpp"
#include "runtime/R1PolicyContract.hpp"

using LowState_ = unitree_hg::msg::dds_::LowState_;
using SportModeState_ = unitree_go::msg::dds_::SportModeState_;

// Compatibility interface for the existing controllers. ONNX ownership and
// tensor names are private; the application can only request inference.
class PolicyRunner {
public:
    virtual ~PolicyRunner() = default;

    virtual void Init(
        const std::string& model_path,
        Ort::Env& environment,
        const Ort::SessionOptions& options) = 0;

    virtual std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx,
        float target_vy,
        float target_yaw,
        float gait_time,
        const std::array<float, 2>& gait_phase) = 0;

    virtual std::vector<float> Infer(const std::vector<float>& observation) {
        if (!model_) throw std::runtime_error("controller ONNX model is not initialized");
        return model_->RunSingle(observation);
    }

    virtual std::array<float, R1Config::NUM_JOINTS> ComputeTargetQ(
        const std::vector<float>& action) = 0;

    virtual void Reset(const LowState_&) {}
    virtual bool IsFinished() const { return false; }
    virtual int GetInputSize() const = 0;
    // Expose contract metadata to the application for cross-component checks
    // (for example, the gait FSM thresholds must match the trained actor).
    std::string MetadataValue(const std::string& key) const {
        return ModelMetadata(key);
    }
    // Current-only gait conditioning. Legacy controllers ignore this value.
    void SetGaitMode(const std::array<float, 3>& mode) {
        float sum = 0.0f;
        for (const float value : mode) {
            if (!std::isfinite(value) || value < 0.0f || value > 1.0f)
                throw std::runtime_error("gait mode must be finite and in [0,1]");
            sum += value;
        }
        if (std::abs(sum - 1.0f) > 1.0e-4f)
            throw std::runtime_error("gait mode must be a one-hot vector");
        gait_mode_ = mode;
    }
    // Fail-safe capability: a new controller does not receive legacy 83-D arm
    // overlay unless it opts in explicitly.
    virtual bool SupportsArmOverlay() const { return false; }
    // H4/gait use a separate gesture path: mask arm q/dq on the current 83-D
    // frame before it enters the history bank. Teleop remains legacy-only.
    virtual bool SupportsHistoryGestureOverlay() const { return false; }
    virtual void SetHistoryArmMask(float /*arm_mask_keep*/) {}

    const std::array<float, R1Config::NUM_JOINTS>& DefaultPosition() const {
        return default_q_;
    }
    const std::array<float, R1Config::NUM_JOINTS>& ActionScale() const {
        return action_scale_;
    }
    const std::array<float, R1Config::NUM_JOINTS>& Stiffness() const {
        return joint_stiffness_;
    }
    const std::array<float, R1Config::NUM_JOINTS>& Damping() const {
        return joint_damping_;
    }

protected:
    void OpenModel(
        const std::string& model_path,
        Ort::Env& environment,
        const Ort::SessionOptions& options) {
        model_ = std::make_unique<r1::policy::OnnxModel>(
            environment, model_path, options);
    }

    void ValidateSingleModelContract(int expected_input_size) const {
        if (!model_ || model_->InputCount() != 1 || model_->OutputCount() != 1
            || model_->InputSize(0) != static_cast<std::size_t>(expected_input_size)
            || model_->OutputSize(0) != R1Config::NUM_JOINTS) {
            throw std::runtime_error(
                "single-policy ONNX contract mismatch; expected input="
                + std::to_string(expected_input_size) + " output=24");
        }
    }

    std::pair<std::size_t, std::size_t> SingleModelShape() const {
        if (!model_ || model_->InputCount() != 1 || model_->OutputCount() != 1) {
            return {0, 0};
        }
        return {model_->InputSize(0), model_->OutputSize(0)};
    }

    void LoadMetadata() {
        r1::policy::PolicyContract fallback{
            default_q_, action_scale_, joint_stiffness_, joint_damping_};
        const auto loaded = r1::policy::LoadPolicyContract(
            *model_, std::move(fallback), r1::contract::JointNames());
        default_q_ = loaded.default_position;
        action_scale_ = loaded.action_scale;
        joint_stiffness_ = loaded.stiffness;
        joint_damping_ = loaded.damping;
    }

    std::string ModelMetadata(const std::string& key) const {
        return model_ ? model_->Metadata(key) : std::string();
    }

    const std::array<float, 3>& CurrentGaitMode() const { return gait_mode_; }

    void ValidateStringListMetadata(
        const char* key,
        const std::vector<std::string>& expected) const {
        const std::string value = ModelMetadata(key);
        if (value.empty()) return;
        std::vector<std::string> actual;
        std::string token;
        for (const char character : value) {
            if (character == ',') {
                actual.push_back(token);
                token.clear();
            } else {
                token.push_back(character);
            }
        }
        actual.push_back(token);
        if (actual != expected) {
            throw std::runtime_error(std::string(key) + " metadata mismatch");
        }
    }

    std::array<float, R1Config::NUM_JOINTS> default_q_{};
    std::array<float, R1Config::NUM_JOINTS> action_scale_{};
    std::array<float, R1Config::NUM_JOINTS> joint_stiffness_{};
    std::array<float, R1Config::NUM_JOINTS> joint_damping_{};
    std::array<float, 3> gait_mode_{1.0f, 0.0f, 0.0f};

private:
    std::unique_ptr<r1::policy::OnnxModel> model_;
};
