#include "controllers/slope_meta/SlopeMetaRuntime.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "runtime/R1PolicyContract.hpp"

namespace r1::slope_meta {

SlopeMetaRuntime::SlopeMetaRuntime(
    const std::filesystem::path& package_directory,
    Ort::Env& environment,
    const Ort::SessionOptions& options)
    : package_directory_(std::filesystem::absolute(package_directory)) {
    const auto registry_path = package_directory_ / "params/deploy.yaml";
    if (!std::filesystem::is_regular_file(registry_path)) {
        throw std::runtime_error(
            "slope_meta registry not found: " + registry_path.string());
    }

    const YAML::Node deploy = YAML::LoadFile(registry_path.string());
    const YAML::Node config = deploy["meta_policy"];
    if (!config || !config.IsMap()) {
        throw std::runtime_error("deploy.yaml is missing meta_policy registry");
    }
    ValidateRegistry(config);

    const YAML::Node expert_configs = config["experts"];
    gate_ = std::make_unique<r1::policy::OnnxModel>(
        environment,
        package_directory_ / config["gate_model"].as<std::string>(),
        options);
    selector_ = std::make_unique<SlopeMetaSelector>(
        config["selector"], expert_configs.size());

    if (gate_->InputCount() != 2 || gate_->OutputCount() != 2) {
        throw std::runtime_error("meta gate must have observation/hidden I/O");
    }
    joint_position_offset_ = config["joint_position_offset"].as<std::size_t>(11);
    previous_action_offset_ = config["previous_action_offset"].as<std::size_t>(59);
    action_size_ = config["action_size"].as<std::size_t>(24);
    history_steps_ = config["history_steps"].as<std::size_t>(0);
    effective_history_steps_ = config["effective_history_steps"].as<std::size_t>(
        history_steps_);
    allow_pd_mismatch_ = config["allow_pd_mismatch"].as<bool>(false);
    outer_crossfade_enabled_ = config["outer_crossfade_enabled"].as<bool>(true);
    blend_space_ = config["blend_space"].as<std::string>("target_q");
    if (history_steps_ == 0) {
        throw std::runtime_error("meta history_steps must be positive");
    }
    if (effective_history_steps_ == 0
        || effective_history_steps_ > history_steps_) {
        throw std::runtime_error(
            "meta effective_history_steps must be within [1, history_steps]");
    }
    if (blend_space_ != "target_q" && blend_space_ != "raw_action") {
        throw std::runtime_error(
            "meta blend_space must be target_q or raw_action");
    }

    const float policy_hz = config["selector"]["policy_hz"].as<float>(50.0f);
    const float cold_start_s = config["selector"]["cold_start_s"].as<float>(1.0f);
    const float warning_s = config["safety_hold_warning_s"].as<float>(2.0f);
    if (!(warning_s > cold_start_s)) {
        throw std::runtime_error(
            "safety_hold_warning_s must be greater than cold_start_s");
    }
    safety_hold_warning_steps_ = std::max(
        1, static_cast<int>(std::lround(warning_s * policy_hz)));

    const YAML::Node action = deploy["actions"]["JointPositionAction"];
    common_contract_.scale = action["scale"].as<std::vector<float>>();
    common_contract_.offset = action["offset"].as<std::vector<float>>();
    ValidateContract(common_contract_, action_size_);
    stiffness_ = deploy["stiffness"].as<std::vector<float>>();
    damping_ = deploy["damping"].as<std::vector<float>>();
    ValidateVector(stiffness_, action_size_, "deploy stiffness");
    ValidateVector(damping_, action_size_, "deploy damping");

    const YAML::Node command_ranges =
        deploy["commands"]["base_velocity"]["ranges"];
    const YAML::Node command_guard = config["command_guard"];
    const auto slew_rates = command_guard["slew_rates"].as<std::vector<float>>();
    const std::array<std::string, 3> range_names = {
        "lin_vel_x", "lin_vel_y", "ang_vel_z"};
    if (slew_rates.size() != command_config_.size()) {
        throw std::runtime_error("meta command_guard.slew_rates must have 3 values");
    }
    for (std::size_t axis = 0; axis < command_config_.size(); ++axis) {
        const auto limits =
            command_ranges[range_names[axis]].as<std::vector<float>>();
        if (limits.size() != 2) {
            throw std::runtime_error("meta command range must have [min, max]");
        }
        command_config_[axis] = {limits[0], limits[1], slew_rates[axis]};
    }
    // Reuse the governor's strict validation at package load time.
    SlopeMetaCommandGovernor command_validator;
    command_validator.Configure(command_config_);

    for (const auto& expert_config : expert_configs) {
        RegisteredExpert expert;
        expert.name = expert_config["name"].as<std::string>();
        expert_names_.push_back(expert.name);
        expert.model = std::make_unique<r1::policy::OnnxModel>(
            environment,
            package_directory_ / expert_config["model"].as<std::string>(),
            options);
        if (expert.model->InputCount() != 1 || expert.model->OutputCount() != 1) {
            throw std::runtime_error("each slope expert must have one input and output");
        }
        if (expert.model->InputSize(0) != gate_->InputSize(0)
            || expert.model->OutputSize(0) != action_size_) {
            throw std::runtime_error("expert observation/action contract mismatch");
        }

        expert.contract.scale = ParseCsvFloats(
            expert.model->Metadata("action_scale"), action_size_, "action_scale");
        expert.contract.offset = ParseCsvFloats(
            expert.model->Metadata("default_joint_pos"),
            action_size_,
            "default_joint_pos");
        expert.joint_names = expert.model->Metadata("joint_names");
        expert.observation_names = expert.model->Metadata("observation_names");
        if (expert.joint_names.empty() || expert.observation_names.empty()) {
            throw std::runtime_error(
                "expert ONNX is missing joint_names or observation_names metadata");
        }
        if (SplitCsv(expert.joint_names) != r1::contract::JointNames()) {
            throw std::runtime_error("expert joint order is not canonical R1 order");
        }
        if (SplitCsv(expert.observation_names)
            != r1::contract::ProprioceptiveObservationNames()) {
            throw std::runtime_error("expert observation order is not the 83-D contract");
        }
        if (!experts_.empty()) {
            if (expert.joint_names != experts_.front().joint_names) {
                throw std::runtime_error("expert joint order mismatch");
            }
            if (expert.observation_names != experts_.front().observation_names) {
                throw std::runtime_error("expert observation order mismatch");
            }
        }

        ValidateContract(expert.contract, action_size_);
        const auto expert_stiffness = ParseCsvFloats(
            expert.model->Metadata("joint_stiffness"),
            action_size_,
            "joint_stiffness");
        const auto expert_damping = ParseCsvFloats(
            expert.model->Metadata("joint_damping"),
            action_size_,
            "joint_damping");
        ValidatePdMatch(expert.name, "stiffness", expert_stiffness, stiffness_);
        ValidatePdMatch(expert.name, "damping", expert_damping, damping_);
        experts_.push_back(std::move(expert));
    }

    ValidateGateContract(config);
    observation_history_.assign(
        history_steps_, std::vector<float>(gate_->InputSize(0), 0.0f));
    action_.assign(action_size_, 0.0f);
    latest_probabilities_.assign(experts_.size() + 1, 0.0f);
    latest_probabilities_.back() = 1.0f;
    crossfade_from_action_ = action_;
    Reset();
}

std::vector<float> SlopeMetaRuntime::Step(
    const std::vector<float>& observation) {
    const auto started = std::chrono::steady_clock::now();
    if (observation.size() != gate_->InputSize(0)) {
        throw std::runtime_error("meta observation dimension mismatch");
    }
    if (fault_latched_.load()) return action_;
    if (!AllFinite(observation)) {
        startup_active_ = false;
        ClearHistory();
        selector_->EnterSafetyHold();
        std::fill(latest_probabilities_.begin(), latest_probabilities_.end(), 0.0f);
        latest_probabilities_.back() = 1.0f;
        NoteSafetyHoldStep();
        return action_;
    }

    observation_history_.erase(observation_history_.begin());
    observation_history_.push_back(observation);
    std::vector<float> hidden(gate_->InputSize(1), 0.0f);
    std::vector<float> probabilities;
    bool gate_outputs_finite = true;
    const std::vector<float> zero_observation(gate_->InputSize(0), 0.0f);
    const std::size_t active_history_start =
        history_steps_ - effective_history_steps_;
    for (std::size_t index = 0; index < observation_history_.size(); ++index) {
        const auto& history_observation = index < active_history_start
            ? zero_observation
            : observation_history_[index];
        auto outputs = gate_->Run({history_observation, hidden});
        probabilities = std::move(outputs.at(0));
        hidden = std::move(outputs.at(1));
        gate_outputs_finite = gate_outputs_finite
            && AllFinite(probabilities) && AllFinite(hidden);
    }

    bool selection_changed = false;
    if (!gate_outputs_finite) {
        startup_active_ = false;
        ClearHistory();
        selector_->EnterSafetyHold();
        std::fill(latest_probabilities_.begin(), latest_probabilities_.end(), 0.0f);
        latest_probabilities_.back() = 1.0f;
        selection_changed = true;
    } else {
        latest_probabilities_ = probabilities;
        selection_changed = selector_->Update(probabilities);
        startup_active_ = selector_->ColdStartActive();
    }

    // A package can expose cold start as a command-locking hold while still
    // running its initial expert weights (slope_meta_v6), or run its fallback
    // normally from step zero (meta_policy). A post-startup SAFETY_HOLD is always
    // different: it preserves the last valid target and skips expert inference.
    if (selector_->SafetyHold() && !startup_active_) {
        NoteSafetyHoldStep();
        return action_;
    }
    consecutive_hold_steps_.store(0);

    std::vector<std::vector<float>> expert_targets;
    std::vector<std::vector<float>> expert_raw_actions;
    expert_targets.reserve(experts_.size());
    expert_raw_actions.reserve(experts_.size());
    bool expert_outputs_finite = true;
    for (const auto& expert : experts_) {
        const auto expert_observation = AdaptObservation(
            observation,
            common_contract_,
            expert.contract,
            joint_position_offset_,
            previous_action_offset_);
        const auto raw_action = expert.model->RunSingle(expert_observation);
        expert_outputs_finite = expert_outputs_finite && AllFinite(raw_action);
        expert_targets.push_back(RawActionToJointTarget(raw_action, expert.contract));
        expert_raw_actions.push_back(raw_action);
    }
    if (!expert_outputs_finite) {
        selector_->EnterSafetyHold();
        crossfade_from_action_ = action_;
        action_crossfade_step_ = selector_->CrossfadeSteps();
        NoteSafetyHoldStep();
        return action_;
    }

    const auto& weights = selector_->Weights();
    std::vector<float> target(action_size_, 0.0f);
    if (blend_space_ == "raw_action") {
        for (std::size_t expert_index = 0; expert_index < experts_.size(); ++expert_index) {
            for (std::size_t joint = 0; joint < action_size_; ++joint) {
                target[joint] +=
                    weights[expert_index] * expert_raw_actions[expert_index][joint];
            }
        }
    } else {
        std::vector<float> blended_target(action_size_, 0.0f);
        for (std::size_t expert_index = 0; expert_index < experts_.size(); ++expert_index) {
            for (std::size_t joint = 0; joint < action_size_; ++joint) {
                blended_target[joint] +=
                    weights[expert_index] * expert_targets[expert_index][joint];
            }
        }
        target = JointTargetToRawAction(blended_target, common_contract_);
    }

    if (outer_crossfade_enabled_ && selection_changed) {
        crossfade_from_action_ = action_;
        action_crossfade_step_ = 0;
    }
    if (outer_crossfade_enabled_
        && action_crossfade_step_ < selector_->CrossfadeSteps()) {
        ++action_crossfade_step_;
        const float alpha = static_cast<float>(action_crossfade_step_)
            / selector_->CrossfadeSteps();
        for (std::size_t joint = 0; joint < action_size_; ++joint) {
            action_[joint] = (1.0f - alpha) * crossfade_from_action_[joint]
                + alpha * target[joint];
        }
    } else {
        action_ = std::move(target);
    }

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (elapsed_ms > 20.0 && !latency_warning_emitted_) {
        std::cerr << "[SlopeMeta] warning: inference exceeded 20 ms: "
                  << elapsed_ms << " ms\n";
        latency_warning_emitted_ = true;
    }
    return action_;
}

void SlopeMetaRuntime::Reset() {
    ClearHistory();
    std::fill(action_.begin(), action_.end(), 0.0f);
    crossfade_from_action_ = action_;
    action_crossfade_step_ = 0;
    selector_->Reset();
    std::fill(latest_probabilities_.begin(), latest_probabilities_.end(), 0.0f);
    latest_probabilities_.back() = 1.0f;
    consecutive_hold_steps_.store(0);
    fault_latched_.store(false);
    startup_active_ = true;
}

std::string SlopeMetaRuntime::SelectedClass() const {
    const int selected = selector_->Selected();
    if (selected == selector_->NeutralIndex()) return "NEUTRAL";
    if (selected < 0 || static_cast<std::size_t>(selected) >= expert_names_.size()) {
        return "INVALID";
    }
    return expert_names_[selected];
}

bool SlopeMetaRuntime::AllFinite(const std::vector<float>& values) {
    return std::all_of(values.begin(), values.end(), [](float value) {
        return std::isfinite(value);
    });
}

std::vector<std::string> SlopeMetaRuntime::SplitCsv(const std::string& value) {
    std::vector<std::string> result;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) result.push_back(item);
    return result;
}

std::vector<float> SlopeMetaRuntime::ParseCsvFloats(
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
        throw std::runtime_error("ONNX metadata " + key + " has invalid values");
    }
    return result;
}

void SlopeMetaRuntime::ValidateVector(
    const std::vector<float>& values,
    std::size_t expected_size,
    const std::string& name) {
    if (values.size() != expected_size || !AllFinite(values)) {
        throw std::runtime_error(name + " has invalid size or value");
    }
}

void SlopeMetaRuntime::ValidateRegistry(const YAML::Node& config) const {
    if (config["label_semantics_version"].as<int>(0) != 3) {
        throw std::runtime_error("meta registry must use label semantics version 3");
    }
    const YAML::Node experts = config["experts"];
    if (!experts.IsSequence() || experts.size() == 0) {
        throw std::runtime_error("meta experts must be a non-empty sequence");
    }
    std::vector<std::string> expected_classes;
    for (const auto& expert : experts) {
        const std::string name = expert["name"].as<std::string>();
        if (name.empty() || name == "NEUTRAL"
            || name.find('.') != std::string::npos
            || std::find(expected_classes.begin(), expected_classes.end(), name)
                != expected_classes.end()) {
            throw std::runtime_error("meta registry contains an invalid expert name");
        }
        expected_classes.push_back(name);
    }
    expected_classes.push_back("NEUTRAL");
    if (config["class_names"].as<std::vector<std::string>>() != expected_classes) {
        throw std::runtime_error(
            "meta class_names must follow expert order plus NEUTRAL");
    }
}

void SlopeMetaRuntime::ValidateGateContract(const YAML::Node& config) const {
    if (gate_->OutputSize(0) != experts_.size() + 1
        || gate_->OutputSize(1) != gate_->InputSize(1)) {
        throw std::runtime_error("meta gate probability/hidden contract is invalid");
    }
    const auto classes = config["class_names"].as<std::vector<std::string>>();
    if (gate_->Metadata("label_semantics_version") != "3"
        || SplitCsv(gate_->Metadata("expert_names")) != expert_names_
        || SplitCsv(gate_->Metadata("class_names")) != classes
        || gate_->Metadata("history_steps") != std::to_string(history_steps_)) {
        throw std::runtime_error("meta gate ONNX metadata does not match registry");
    }
}

void SlopeMetaRuntime::ValidatePdMatch(
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

    const std::string message = "expert " + expert_name + " " + term
        + " differs from deploy contract; max_abs_diff="
        + std::to_string(maximum_difference)
        + ". The action adapter cannot compensate PD gain mismatch.";
    if (!allow_pd_mismatch_) throw std::runtime_error(message);
    std::cerr << "[SlopeMeta] warning: " << message << '\n';
}

void SlopeMetaRuntime::ClearHistory() {
    for (auto& observation : observation_history_) {
        std::fill(observation.begin(), observation.end(), 0.0f);
    }
}

void SlopeMetaRuntime::NoteSafetyHoldStep() {
    const int hold_steps = consecutive_hold_steps_.fetch_add(1) + 1;
    if (hold_steps >= safety_hold_warning_steps_
        && !fault_latched_.exchange(true)) {
        std::cerr << "[SlopeMeta] ERROR: SAFETY_HOLD exceeded the configured "
                     "duration; locomotion target remains held until reset\n";
    }
}

}  // namespace r1::slope_meta
