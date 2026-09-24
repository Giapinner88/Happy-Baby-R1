#include "controllers/rma_meta/RmaMetaRuntime.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "runtime/R1PolicyContract.hpp"

namespace r1::rma_meta {

RmaMetaRuntime::RmaMetaRuntime(
    const std::filesystem::path& package_directory,
    Ort::Env& environment,
    const Ort::SessionOptions& options)
    : package_directory_(std::filesystem::absolute(package_directory)) {
    const auto deploy_path = package_directory_ / "params/deploy.yaml";
    if (!std::filesystem::is_regular_file(deploy_path)) {
        throw std::runtime_error(
            "RMA-meta deploy config not found: " + deploy_path.string());
    }
    const YAML::Node deploy = YAML::LoadFile(deploy_path.string());
    const YAML::Node config = deploy["rma_meta"];
    if (!config || !config.IsMap()) {
        throw std::runtime_error("deploy.yaml is missing rma_meta registry");
    }
    ValidateConfiguration(config);

    const auto adapter_path =
        package_directory_ / config["adapter_model"].as<std::string>();
    const auto selector_path =
        package_directory_ / config["selector_model"].as<std::string>();
    if (!std::filesystem::is_regular_file(adapter_path)
        || !std::filesystem::is_regular_file(selector_path)) {
        throw std::runtime_error("RMA-meta adapter or selector ONNX is missing");
    }
    adapter_ = std::make_unique<r1::policy::OnnxModel>(
        environment, adapter_path, options);
    selector_ = std::make_unique<r1::policy::OnnxModel>(
        environment, selector_path, options);

    if (adapter_->InputCount() != 1 || adapter_->OutputCount() != 1
        || adapter_->InputSize(0) != history_steps_ * observation_size_
        || adapter_->OutputSize(0) != latent_size_) {
        throw std::runtime_error("RMA adapter ONNX tensor contract is invalid");
    }
    if (selector_->InputCount() != 3 || selector_->OutputCount() != 1
        || selector_->InputSize(0) != observation_size_
        || selector_->InputSize(1) != latent_size_
        || selector_->InputSize(2) != expert_names_.size()
        || selector_->OutputSize(0) != class_names_.size()) {
        throw std::runtime_error("RMA selector ONNX tensor contract is invalid");
    }

    const YAML::Node action_config = deploy["actions"]["JointPositionAction"];
    common_contract_.scale = action_config["scale"].as<std::vector<float>>();
    common_contract_.offset = action_config["offset"].as<std::vector<float>>();
    r1::slope_meta::ValidateContract(common_contract_, action_size_);
    stiffness_ = deploy["stiffness"].as<std::vector<float>>();
    damping_ = deploy["damping"].as<std::vector<float>>();
    ValidateVector(stiffness_, action_size_, "deploy stiffness");
    ValidateVector(damping_, action_size_, "deploy damping");

    const YAML::Node command_ranges =
        deploy["commands"]["base_velocity"]["ranges"];
    const auto slew_rates =
        config["command_guard"]["slew_rates"].as<std::vector<float>>();
    const std::array<std::string, 3> range_names = {
        "lin_vel_x", "lin_vel_y", "ang_vel_z"};
    if (slew_rates.size() != command_config_.size()) {
        throw std::runtime_error("rma_meta command_guard.slew_rates must have 3 values");
    }
    for (std::size_t axis = 0; axis < command_config_.size(); ++axis) {
        const auto limits =
            command_ranges[range_names[axis]].as<std::vector<float>>();
        if (limits.size() != 2) {
            throw std::runtime_error("RMA-meta command range must have [min, max]");
        }
        command_config_[axis] = {limits[0], limits[1], slew_rates[axis]};
    }
    r1::slope_meta::SlopeMetaCommandGovernor command_validator;
    command_validator.Configure(command_config_);

    const YAML::Node expert_configs = config["experts"];
    for (const auto& expert_config : expert_configs) {
        RegisteredExpert expert;
        expert.name = expert_config["name"].as<std::string>();
        const auto model_path =
            package_directory_ / expert_config["model"].as<std::string>();
        if (!std::filesystem::is_regular_file(model_path)) {
            throw std::runtime_error(
                "RMA-meta expert ONNX is missing: " + model_path.string());
        }
        expert.model = std::make_unique<r1::policy::OnnxModel>(
            environment, model_path, options);
        if (expert.model->InputCount() != 1 || expert.model->OutputCount() != 1
            || expert.model->InputSize(0) != observation_size_
            || expert.model->OutputSize(0) != action_size_) {
            throw std::runtime_error("RMA-meta expert tensor contract is invalid");
        }
        expert.contract.scale = ParseCsvFloats(
            expert.model->Metadata("action_scale"), action_size_, "action_scale");
        expert.contract.offset = ParseCsvFloats(
            expert.model->Metadata("default_joint_pos"),
            action_size_,
            "default_joint_pos");
        expert.joint_names = expert.model->Metadata("joint_names");
        expert.observation_names = expert.model->Metadata("observation_names");
        if (SplitCsv(expert.joint_names) != r1::contract::JointNames()) {
            throw std::runtime_error("RMA-meta expert joint order is not canonical");
        }
        if (SplitCsv(expert.observation_names)
            != r1::contract::ProprioceptiveObservationNames()) {
            throw std::runtime_error("RMA-meta expert observation order is not 83-D");
        }
        r1::slope_meta::ValidateContract(expert.contract, action_size_);
        ValidatePdMatch(
            expert.name,
            "stiffness",
            ParseCsvFloats(
                expert.model->Metadata("joint_stiffness"),
                action_size_,
                "joint_stiffness"),
            stiffness_);
        ValidatePdMatch(
            expert.name,
            "damping",
            ParseCsvFloats(
                expert.model->Metadata("joint_damping"),
                action_size_,
                "joint_damping"),
            damping_);
        experts_.push_back(std::move(expert));
    }
    ValidateModelMetadata();

    observation_history_.assign(
        history_steps_, std::vector<float>(observation_size_, 0.0f));
    latent_.assign(latent_size_, 0.0f);
    probabilities_.assign(class_names_.size(), 0.0f);
    action_.assign(action_size_, 0.0f);
    crossfade_from_action_ = action_;
    Reset();
}

std::vector<float> RmaMetaRuntime::Step(
    const std::vector<float>& observation) {
    const auto started = std::chrono::steady_clock::now();
    if (observation.size() != observation_size_) {
        throw std::runtime_error("RMA-meta observation dimension mismatch");
    }
    if (fault_latched_.load()) {
        safety_hold_ = true;
        return action_;
    }
    if (!AllFinite(observation)) {
        NoteInvalidStep(true);
        return action_;
    }

    observation_history_.erase(observation_history_.begin());
    observation_history_.push_back(observation);
    history_valid_steps_ = std::min(history_steps_, history_valid_steps_ + 1);

    if (low_level_step_ % meta_decimation_ == 0) {
        const auto adapter_outputs = adapter_->Run({FlattenHistory()});
        if (adapter_outputs.size() != 1
            || !AllFinite(adapter_outputs.front())) {
            NoteInvalidStep(true);
            return action_;
        }
        latent_ = adapter_outputs.front();
        std::vector<float> previous_one_hot(experts_.size(), 0.0f);
        previous_one_hot.at(static_cast<std::size_t>(selected_expert_)) = 1.0f;
        const auto selector_outputs = selector_->Run(
            {observation, latent_, previous_one_hot});
        if (selector_outputs.size() != 1
            || !ValidProbabilities(selector_outputs.front())) {
            NoteInvalidStep(true);
            return action_;
        }
        probabilities_ = selector_outputs.front();
        ++meta_decision_count_;
        const auto best =
            std::max_element(probabilities_.begin(), probabilities_.end());
        const int proposed = static_cast<int>(
            std::distance(probabilities_.begin(), best));
        if (*best >= confidence_threshold_
            && proposed >= 0
            && proposed < static_cast<int>(experts_.size())
            && proposed != selected_expert_) {
            selected_expert_ = proposed;
            crossfade_from_action_ = action_;
            crossfade_step_ = 0;
        }
    }

    const auto& expert = experts_.at(static_cast<std::size_t>(selected_expert_));
    const auto expert_observation = r1::slope_meta::AdaptObservation(
        observation,
        common_contract_,
        expert.contract,
        joint_position_offset_,
        previous_action_offset_);
    const auto raw_expert_action = expert.model->RunSingle(expert_observation);
    if (!AllFinite(raw_expert_action)) {
        NoteInvalidStep(false);
        return action_;
    }
    const auto target =
        r1::slope_meta::RawActionToJointTarget(raw_expert_action, expert.contract);
    auto common_action =
        r1::slope_meta::JointTargetToRawAction(target, common_contract_);
    if (!AllFinite(common_action)) {
        NoteInvalidStep(false);
        return action_;
    }
    if (crossfade_step_ < crossfade_steps_) {
        ++crossfade_step_;
        const float alpha =
            static_cast<float>(crossfade_step_) / crossfade_steps_;
        for (std::size_t joint = 0; joint < action_.size(); ++joint) {
            action_[joint] = (1.0f - alpha) * crossfade_from_action_[joint]
                + alpha * common_action[joint];
        }
    } else {
        action_ = std::move(common_action);
    }

    ++low_level_step_;
    safety_hold_ = false;
    consecutive_invalid_steps_.store(0);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (elapsed_ms > 20.0 && !latency_warning_emitted_) {
        std::cerr << "[RmaMeta] warning: inference exceeded 20 ms: "
                  << elapsed_ms << " ms\n";
        latency_warning_emitted_ = true;
    }
    return action_;
}

void RmaMetaRuntime::Reset() {
    ClearHistory();
    std::fill(latent_.begin(), latent_.end(), 0.0f);
    std::fill(probabilities_.begin(), probabilities_.end(), 0.0f);
    probabilities_.back() = 1.0f;
    std::fill(action_.begin(), action_.end(), 0.0f);
    crossfade_from_action_ = action_;
    selected_expert_ = initial_expert_;
    crossfade_step_ = crossfade_steps_;
    low_level_step_ = 0;
    history_valid_steps_ = 0;
    meta_decision_count_ = 0;
    safety_hold_ = false;
    latency_warning_emitted_ = false;
    consecutive_invalid_steps_.store(0);
    fault_latched_.store(false);
}

std::string RmaMetaRuntime::SelectedClass() const {
    if (selected_expert_ < 0
        || static_cast<std::size_t>(selected_expert_) >= expert_names_.size()) {
        return "INVALID";
    }
    return expert_names_[static_cast<std::size_t>(selected_expert_)];
}

std::vector<float> RmaMetaRuntime::ExpertWeights() const {
    std::vector<float> weights(experts_.size(), 0.0f);
    if (selected_expert_ >= 0
        && static_cast<std::size_t>(selected_expert_) < weights.size()) {
        weights[static_cast<std::size_t>(selected_expert_)] = 1.0f;
    }
    return weights;
}

bool RmaMetaRuntime::AllFinite(const std::vector<float>& values) {
    return std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
    });
}

std::vector<std::string> RmaMetaRuntime::SplitCsv(const std::string& value) {
    std::vector<std::string> result;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) result.push_back(item);
    return result;
}

std::vector<float> RmaMetaRuntime::ParseCsvFloats(
    const std::string& value,
    std::size_t expected_size,
    const std::string& key) {
    if (value.empty()) {
        throw std::runtime_error("ONNX is missing " + key + " metadata");
    }
    std::vector<float> result;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) result.push_back(std::stof(item));
    if (result.size() != expected_size || !AllFinite(result)) {
        throw std::runtime_error("ONNX metadata " + key + " is invalid");
    }
    return result;
}

void RmaMetaRuntime::ValidateVector(
    const std::vector<float>& values,
    std::size_t expected_size,
    const std::string& name) {
    if (values.size() != expected_size || !AllFinite(values)) {
        throw std::runtime_error(name + " has invalid size or value");
    }
}

void RmaMetaRuntime::ValidateConfiguration(const YAML::Node& config) {
    if (config["config_version"].as<int>(0) != 1) {
        throw std::runtime_error("RMA-meta config_version must be 1");
    }
    observation_size_ = config["observation_dim"].as<std::size_t>(0);
    action_size_ = config["action_dim"].as<std::size_t>(0);
    latent_size_ = config["latent_dim"].as<std::size_t>(0);
    history_steps_ = config["history_steps"].as<std::size_t>(0);
    joint_position_offset_ =
        config["joint_position_offset"].as<std::size_t>(11);
    previous_action_offset_ =
        config["previous_action_offset"].as<std::size_t>(59);
    confidence_threshold_ =
        config["confidence_threshold"].as<float>(0.55f);
    allow_pd_mismatch_ = config["allow_pd_mismatch"].as<bool>(false);
    if (observation_size_ == 0 || action_size_ == 0 || latent_size_ == 0
        || history_steps_ == 0
        || joint_position_offset_ + action_size_ > observation_size_
        || previous_action_offset_ + action_size_ > observation_size_
        || !std::isfinite(confidence_threshold_)
        || confidence_threshold_ < 0.0f || confidence_threshold_ > 1.0f) {
        throw std::runtime_error("RMA-meta dimensions/threshold are invalid");
    }

    const YAML::Node expert_configs = config["experts"];
    if (!expert_configs.IsSequence() || expert_configs.size() == 0) {
        throw std::runtime_error("RMA-meta needs a non-empty expert registry");
    }
    for (const auto& expert_config : expert_configs) {
        const std::string name = expert_config["name"].as<std::string>();
        if (name.empty() || name == "HOLD"
            || std::find(expert_names_.begin(), expert_names_.end(), name)
                != expert_names_.end()) {
            throw std::runtime_error("RMA-meta expert name is invalid or duplicated");
        }
        expert_names_.push_back(name);
    }
    if (config["expert_names"].as<std::vector<std::string>>() != expert_names_) {
        throw std::runtime_error("RMA-meta declared expert order differs");
    }
    class_names_ = config["class_names"].as<std::vector<std::string>>();
    auto expected_classes = expert_names_;
    expected_classes.push_back("HOLD");
    if (class_names_ != expected_classes) {
        throw std::runtime_error("RMA-meta classes must be expert order plus HOLD");
    }
    const std::string initial_name = config["initial_expert"].as<std::string>();
    const auto initial =
        std::find(expert_names_.begin(), expert_names_.end(), initial_name);
    if (initial == expert_names_.end()) {
        throw std::runtime_error("RMA-meta initial_expert is not registered");
    }
    initial_expert_ =
        static_cast<int>(std::distance(expert_names_.begin(), initial));

    const float low_level_hz = config["low_level_hz"].as<float>(0.0f);
    const float meta_hz = config["meta_hz"].as<float>(0.0f);
    const float crossfade_s = config["crossfade_s"].as<float>(-1.0f);
    const float warning_s = config["safety_hold_warning_s"].as<float>(0.0f);
    if (!(std::isfinite(low_level_hz) && std::isfinite(meta_hz)
          && std::isfinite(crossfade_s) && std::isfinite(warning_s)
          && low_level_hz > 0.0f && meta_hz > 0.0f
          && crossfade_s >= 0.0f && warning_s > 0.0f)) {
        throw std::runtime_error("RMA-meta frequency/timing config is invalid");
    }
    const float ratio = low_level_hz / meta_hz;
    meta_decimation_ = static_cast<int>(std::lround(ratio));
    if (meta_decimation_ <= 0 || std::abs(ratio - meta_decimation_) > 1.0e-5f) {
        throw std::runtime_error("RMA-meta frequencies need an integer ratio");
    }
    crossfade_steps_ = std::max(
        1, static_cast<int>(std::lround(crossfade_s * low_level_hz)));
    safety_hold_warning_steps_ = std::max(
        1, static_cast<int>(std::lround(warning_s * low_level_hz)));
}

void RmaMetaRuntime::ValidateModelMetadata() const {
    const auto validate = [&](const r1::policy::OnnxModel& model,
                              const std::string& module) {
        if (model.Metadata("rma_meta_config_version") != "1"
            || model.Metadata("module") != module
            || model.Metadata("checkpoint_stage") != "deployment_finetune"
            || SplitCsv(model.Metadata("expert_names")) != expert_names_
            || SplitCsv(model.Metadata("class_names")) != class_names_
            || model.Metadata("history_steps") != std::to_string(history_steps_)
            || model.Metadata("observation_dim") != std::to_string(observation_size_)
            || model.Metadata("latent_dim") != std::to_string(latent_size_)) {
            throw std::runtime_error(
                "RMA-meta ONNX metadata does not match deployment config");
        }
    };
    validate(*adapter_, "adaptation");
    validate(*selector_, "meta_selector");
    if (adapter_->Metadata("checkpoint_sha256").empty()
        || adapter_->Metadata("checkpoint_sha256")
            != selector_->Metadata("checkpoint_sha256")
        || adapter_->Metadata("registry_sha256").empty()
        || adapter_->Metadata("registry_sha256")
            != selector_->Metadata("registry_sha256")) {
        throw std::runtime_error("RMA adapter and selector provenance differ");
    }
}

void RmaMetaRuntime::ValidatePdMatch(
    const std::string& expert_name,
    const std::string& term,
    const std::vector<float>& expert_values,
    const std::vector<float>& deploy_values) const {
    float maximum_difference = 0.0f;
    for (std::size_t index = 0; index < expert_values.size(); ++index) {
        maximum_difference = std::max(
            maximum_difference,
            std::abs(expert_values[index] - deploy_values[index]));
    }
    if (maximum_difference <= 1.0e-4f) return;
    const std::string message = "RMA-meta expert " + expert_name + " " + term
        + " differs from common PD; max_abs_diff="
        + std::to_string(maximum_difference);
    if (!allow_pd_mismatch_) throw std::runtime_error(message);
    std::cerr << "[RmaMeta] warning: " << message << '\n';
}

bool RmaMetaRuntime::ValidProbabilities(
    const std::vector<float>& values) const {
    if (values.size() != class_names_.size() || !AllFinite(values)) return false;
    float sum = 0.0f;
    for (const float value : values) {
        if (value < 0.0f || value > 1.0f) return false;
        sum += value;
    }
    return std::abs(sum - 1.0f) <= 1.0e-3f;
}

void RmaMetaRuntime::ClearHistory() {
    for (auto& observation : observation_history_) {
        std::fill(observation.begin(), observation.end(), 0.0f);
    }
    history_valid_steps_ = 0;
}

void RmaMetaRuntime::NoteInvalidStep(bool clear_history) {
    safety_hold_ = true;
    if (clear_history) ClearHistory();
    const int count = consecutive_invalid_steps_.fetch_add(1) + 1;
    if (count >= safety_hold_warning_steps_
        && !fault_latched_.exchange(true)) {
        std::cerr << "[RmaMeta] error: invalid inference persisted; holding "
                     "the last finite target and requiring reset\n";
    }
}

std::vector<float> RmaMetaRuntime::FlattenHistory() const {
    std::vector<float> flattened;
    flattened.reserve(history_steps_ * observation_size_);
    for (const auto& observation : observation_history_) {
        flattened.insert(
            flattened.end(), observation.begin(), observation.end());
    }
    return flattened;
}

}  // namespace r1::rma_meta
