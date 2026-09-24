#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "controllers/slope_meta/ExpertActionAdapter.hpp"
#include "controllers/slope_meta/SlopeMetaCommandGovernor.hpp"
#include "controllers/slope_meta/SlopeMetaSelector.hpp"
#include "onnx/OnnxModel.hpp"

namespace r1::slope_meta {

struct RegisteredExpert {
    std::string name;
    std::unique_ptr<r1::policy::OnnxModel> model;
    JointActionContract contract;
    std::string joint_names;
    std::string observation_names;
};

// Recurrent no-camera gate plus an extensible YAML registry of 83-D experts.
// The gate recomputes its fixed history window each cycle, matching training.
class SlopeMetaRuntime {
public:
    SlopeMetaRuntime(
        const std::filesystem::path& package_directory,
        Ort::Env& environment,
        const Ort::SessionOptions& options);

    std::vector<float> Step(const std::vector<float>& observation);
    void Reset();

    const JointActionContract& CommonContract() const { return common_contract_; }
    const std::vector<float>& Stiffness() const { return stiffness_; }
    const std::vector<float>& Damping() const { return damping_; }
    const std::vector<std::string>& ExpertNames() const { return expert_names_; }
    std::string SelectedClass() const;
    const std::vector<float>& Weights() const { return selector_->Weights(); }
    const std::vector<float>& Probabilities() const { return latest_probabilities_; }
    const std::array<CommandAxisConfig, 3>& CommandConfig() const {
        return command_config_;
    }
    bool SafetyHold() const { return selector_->SafetyHold(); }
    bool StartupActive() const { return startup_active_; }
    bool FaultLatched() const { return fault_latched_.load(); }
    int ConsecutiveSafetyHoldSteps() const { return consecutive_hold_steps_.load(); }
    int SafetyHoldWarningSteps() const { return safety_hold_warning_steps_; }
    std::size_t ObservationSize() const { return gate_->InputSize(0); }
    std::size_t EffectiveHistorySteps() const { return effective_history_steps_; }
    const std::string& BlendSpace() const { return blend_space_; }
    bool OuterCrossfadeEnabled() const { return outer_crossfade_enabled_; }

private:
    static bool AllFinite(const std::vector<float>& values);
    static std::vector<std::string> SplitCsv(const std::string& value);
    static std::vector<float> ParseCsvFloats(
        const std::string& value,
        std::size_t expected_size,
        const std::string& key);
    static void ValidateVector(
        const std::vector<float>& values,
        std::size_t expected_size,
        const std::string& name);

    void ValidateRegistry(const YAML::Node& config) const;
    void ValidateGateContract(const YAML::Node& config) const;
    void ValidatePdMatch(
        const std::string& expert_name,
        const std::string& term,
        const std::vector<float>& expert_values,
        const std::vector<float>& deploy_values) const;
    void ClearHistory();
    void NoteSafetyHoldStep();

    std::filesystem::path package_directory_;
    std::unique_ptr<r1::policy::OnnxModel> gate_;
    std::vector<RegisteredExpert> experts_;
    std::vector<std::string> expert_names_;
    std::unique_ptr<SlopeMetaSelector> selector_;
    JointActionContract common_contract_;
    std::vector<float> stiffness_;
    std::vector<float> damping_;
    std::array<CommandAxisConfig, 3> command_config_{};
    std::vector<std::vector<float>> observation_history_;
    std::vector<float> latest_probabilities_;
    std::vector<float> action_;
    std::vector<float> crossfade_from_action_;
    std::size_t joint_position_offset_ = 11;
    std::size_t previous_action_offset_ = 59;
    std::size_t action_size_ = 24;
    std::size_t history_steps_ = 0;
    std::size_t effective_history_steps_ = 0;
    int action_crossfade_step_ = 0;
    int safety_hold_warning_steps_ = 100;
    bool allow_pd_mismatch_ = false;
    bool outer_crossfade_enabled_ = true;
    std::string blend_space_ = "target_q";
    bool startup_active_ = true;
    bool latency_warning_emitted_ = false;
    std::atomic<int> consecutive_hold_steps_{0};
    std::atomic<bool> fault_latched_{false};
};

}  // namespace r1::slope_meta
