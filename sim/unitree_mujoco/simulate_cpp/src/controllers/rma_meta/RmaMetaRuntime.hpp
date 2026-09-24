#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "controllers/slope_meta/ExpertActionAdapter.hpp"
#include "controllers/slope_meta/SlopeMetaCommandGovernor.hpp"
#include "onnx/OnnxModel.hpp"

namespace r1::rma_meta {

struct RegisteredExpert {
    std::string name;
    std::unique_ptr<r1::policy::OnnxModel> model;
    r1::slope_meta::JointActionContract contract;
    std::string joint_names;
    std::string observation_names;
};

// RMA-inspired hierarchical policy. A fixed-history adaptation network
// estimates an 8-D latent and a categorical PPO selector chooses one frozen
// 83-D expert or HOLD. Privileged terrain/dynamics factors are train-only.
class RmaMetaRuntime {
public:
    RmaMetaRuntime(
        const std::filesystem::path& package_directory,
        Ort::Env& environment,
        const Ort::SessionOptions& options);

    std::vector<float> Step(const std::vector<float>& observation);
    void Reset();

    const r1::slope_meta::JointActionContract& CommonContract() const {
        return common_contract_;
    }
    const std::vector<float>& Stiffness() const { return stiffness_; }
    const std::vector<float>& Damping() const { return damping_; }
    const std::vector<std::string>& ExpertNames() const { return expert_names_; }
    const std::vector<float>& Probabilities() const { return probabilities_; }
    const std::vector<float>& Latent() const { return latent_; }
    const std::array<r1::slope_meta::CommandAxisConfig, 3>& CommandConfig() const {
        return command_config_;
    }

    std::string SelectedClass() const;
    std::vector<float> ExpertWeights() const;
    bool SafetyHold() const { return safety_hold_; }
    bool FaultLatched() const { return fault_latched_.load(); }
    int ConsecutiveInvalidSteps() const { return consecutive_invalid_steps_.load(); }
    int SafetyHoldWarningSteps() const { return safety_hold_warning_steps_; }
    int MetaDecimation() const { return meta_decimation_; }
    std::size_t ObservationSize() const { return observation_size_; }
    std::size_t HistorySteps() const { return history_steps_; }
    std::size_t HistoryValidSteps() const { return history_valid_steps_; }
    std::size_t MetaDecisionCount() const { return meta_decision_count_; }

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

    void ValidateConfiguration(const YAML::Node& config);
    void ValidateModelMetadata() const;
    void ValidatePdMatch(
        const std::string& expert_name,
        const std::string& term,
        const std::vector<float>& expert_values,
        const std::vector<float>& deploy_values) const;
    bool ValidProbabilities(const std::vector<float>& values) const;
    void ClearHistory();
    void NoteInvalidStep(bool clear_history);
    std::vector<float> FlattenHistory() const;

    std::filesystem::path package_directory_;
    std::unique_ptr<r1::policy::OnnxModel> adapter_;
    std::unique_ptr<r1::policy::OnnxModel> selector_;
    std::vector<RegisteredExpert> experts_;
    std::vector<std::string> expert_names_;
    std::vector<std::string> class_names_;
    r1::slope_meta::JointActionContract common_contract_;
    std::vector<float> stiffness_;
    std::vector<float> damping_;
    std::array<r1::slope_meta::CommandAxisConfig, 3> command_config_{};
    std::vector<std::vector<float>> observation_history_;
    std::vector<float> latent_;
    std::vector<float> probabilities_;
    std::vector<float> action_;
    std::vector<float> crossfade_from_action_;
    std::size_t observation_size_ = 83;
    std::size_t action_size_ = 24;
    std::size_t latent_size_ = 8;
    std::size_t history_steps_ = 50;
    std::size_t history_valid_steps_ = 0;
    std::size_t meta_decision_count_ = 0;
    std::size_t joint_position_offset_ = 11;
    std::size_t previous_action_offset_ = 59;
    float confidence_threshold_ = 0.55f;
    int initial_expert_ = 0;
    int selected_expert_ = 0;
    int meta_decimation_ = 5;
    int crossfade_steps_ = 10;
    int crossfade_step_ = 10;
    int low_level_step_ = 0;
    int safety_hold_warning_steps_ = 100;
    bool allow_pd_mismatch_ = false;
    bool safety_hold_ = false;
    bool latency_warning_emitted_ = false;
    std::atomic<int> consecutive_invalid_steps_{0};
    std::atomic<bool> fault_latched_{false};
};

}  // namespace r1::rma_meta
