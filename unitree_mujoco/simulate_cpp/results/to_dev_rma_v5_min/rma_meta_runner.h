// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// RMA-inspired adapter + PPO selector over an extensible frozen expert registry.

#pragma once

#include "isaaclab/algorithms/algorithms.h"
#include "isaaclab/algorithms/expert_action_adapter.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace isaaclab
{

class RmaOrtModel
{
public:
    explicit RmaOrtModel(const std::filesystem::path& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, model_path.filename().string().c_str())
    {
        if (!std::filesystem::is_regular_file(model_path)) {
            throw std::runtime_error("RMA-meta ONNX does not exist: " + model_path.string());
        }
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        session_ = std::make_unique<Ort::Session>(
            env_, model_path.string().c_str(), session_options_);
        load_tensor_contract(true);
        load_tensor_contract(false);
    }

    std::size_t input_size(std::size_t index) const { return input_sizes_.at(index); }
    std::size_t output_size(std::size_t index) const { return output_sizes_.at(index); }
    std::size_t input_count() const { return input_names_.size(); }
    std::size_t output_count() const { return output_names_.size(); }

    std::string metadata(const std::string& key) const
    {
        auto metadata = session_->GetModelMetadata();
        auto value = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator_);
        return value ? std::string(value.get()) : std::string();
    }

    std::vector<std::vector<float>> run(
        const std::vector<std::vector<float>>& inputs)
    {
        if (inputs.size() != input_names_.size()) {
            throw std::runtime_error("RMA-meta ONNX input count mismatch");
        }
        auto memory_info = Ort::MemoryInfo::CreateCpu(
            OrtDeviceAllocator, OrtMemTypeCPU);
        std::vector<Ort::Value> tensors;
        tensors.reserve(inputs.size());
        for (std::size_t index = 0; index < inputs.size(); ++index) {
            if (inputs[index].size() != input_sizes_[index]) {
                throw std::runtime_error(
                    "RMA-meta ONNX input size mismatch for " + input_names_[index]);
            }
            tensors.push_back(Ort::Value::CreateTensor<float>(
                memory_info,
                const_cast<float*>(inputs[index].data()),
                inputs[index].size(),
                input_shapes_[index].data(),
                input_shapes_[index].size()));
        }
        std::vector<const char*> input_names;
        std::vector<const char*> output_names;
        for (const auto& name : input_names_) input_names.push_back(name.c_str());
        for (const auto& name : output_names_) output_names.push_back(name.c_str());
        auto values = session_->Run(
            Ort::RunOptions{nullptr},
            input_names.data(), tensors.data(), tensors.size(),
            output_names.data(), output_names.size());
        std::vector<std::vector<float>> outputs;
        outputs.reserve(values.size());
        for (std::size_t index = 0; index < values.size(); ++index) {
            const float* data = values[index].GetTensorData<float>();
            outputs.emplace_back(data, data + output_sizes_[index]);
        }
        return outputs;
    }

private:
    static std::size_t tensor_size(std::vector<int64_t>& shape)
    {
        std::size_t size = 1;
        for (auto& dimension : shape) {
            if (dimension <= 0) dimension = 1;
            size *= static_cast<std::size_t>(dimension);
        }
        return size;
    }

    void load_tensor_contract(bool input)
    {
        const std::size_t count = input
            ? session_->GetInputCount() : session_->GetOutputCount();
        for (std::size_t index = 0; index < count; ++index) {
            Ort::TypeInfo type = input
                ? session_->GetInputTypeInfo(index)
                : session_->GetOutputTypeInfo(index);
            auto shape = type.GetTensorTypeAndShapeInfo().GetShape();
            auto name = input
                ? session_->GetInputNameAllocated(index, allocator_)
                : session_->GetOutputNameAllocated(index, allocator_);
            if (input) {
                input_names_.emplace_back(name.get());
                input_sizes_.push_back(tensor_size(shape));
                input_shapes_.push_back(std::move(shape));
            } else {
                output_names_.emplace_back(name.get());
                output_sizes_.push_back(tensor_size(shape));
                output_shapes_.push_back(std::move(shape));
            }
        }
    }

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    mutable Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
    std::vector<std::size_t> input_sizes_;
    std::vector<std::size_t> output_sizes_;
};

struct RmaRegisteredExpert
{
    std::string name;
    std::unique_ptr<RmaOrtModel> model;
    JointActionContract contract;
    std::string joint_names;
    std::string observation_names;
};

class RmaMetaRunner : public Algorithms
{
public:
    RmaMetaRunner(
        const std::filesystem::path& policy_dir,
        const YAML::Node& cfg,
        const YAML::Node& deploy_cfg)
    : adapter_(policy_dir / cfg["adapter_model"].as<std::string>()),
      selector_(policy_dir / cfg["selector_model"].as<std::string>())
    {
        if (cfg["config_version"].as<int>(0) != 1) {
            throw std::runtime_error("RMA-meta config_version must be 1");
        }
        const YAML::Node expert_cfgs = cfg["experts"];
        if (!expert_cfgs.IsSequence() || expert_cfgs.size() == 0) {
            throw std::runtime_error("RMA-meta needs a non-empty expert registry");
        }
        observation_key_ = cfg["observation_key"].as<std::string>("obs");
        observation_size_ = cfg["observation_dim"].as<std::size_t>(83);
        action_size_ = cfg["action_dim"].as<std::size_t>(24);
        latent_size_ = cfg["latent_dim"].as<std::size_t>(8);
        history_steps_ = cfg["history_steps"].as<std::size_t>(50);
        joint_position_offset_ = cfg["joint_position_offset"].as<std::size_t>(11);
        previous_action_offset_ = cfg["previous_action_offset"].as<std::size_t>(59);
        confidence_threshold_ = cfg["confidence_threshold"].as<float>(0.55f);
        const float low_level_hz = cfg["low_level_hz"].as<float>(50.0f);
        const float meta_hz = cfg["meta_hz"].as<float>(10.0f);
        const float crossfade_s = cfg["crossfade_s"].as<float>(0.2f);
        const float warning_s = cfg["safety_hold_warning_s"].as<float>(2.0f);
        allow_pd_mismatch_ = cfg["allow_pd_mismatch"].as<bool>(false);
        // Experts are registered further down, so keep the name and resolve
        // it once the registry exists.
        via_expert_name_ = cfg["handover_via_expert"].as<std::string>(std::string());
        if (!(low_level_hz > 0.0f && meta_hz > 0.0f && crossfade_s >= 0.0f
              && warning_s > 0.0f && confidence_threshold_ >= 0.0f
              && confidence_threshold_ <= 1.0f)) {
            throw std::runtime_error("RMA-meta runtime frequency/threshold is invalid");
        }
        const float frequency_ratio = low_level_hz / meta_hz;
        meta_decimation_ = static_cast<int>(std::lround(frequency_ratio));
        if (meta_decimation_ <= 0
            || std::abs(frequency_ratio - meta_decimation_) > 1.0e-5f) {
            throw std::runtime_error("RMA-meta frequencies need an integer ratio");
        }
        crossfade_steps_ = std::max(1, static_cast<int>(std::lround(
            crossfade_s * low_level_hz)));
        safety_hold_warning_steps_ = std::max(1, static_cast<int>(std::lround(
            warning_s * low_level_hz)));

        if (adapter_.input_count() != 1 || adapter_.output_count() != 1
            || adapter_.input_size(0) != history_steps_ * observation_size_
            || adapter_.output_size(0) != latent_size_) {
            throw std::runtime_error("RMA adapter ONNX tensor contract is invalid");
        }
        if (selector_.input_count() != 3 || selector_.output_count() != 1
            || selector_.input_size(0) != observation_size_
            || selector_.input_size(1) != latent_size_
            || selector_.input_size(2) != expert_cfgs.size()
            || selector_.output_size(0) != expert_cfgs.size() + 1) {
            throw std::runtime_error("RMA selector ONNX tensor contract is invalid");
        }

        const YAML::Node action_cfg = deploy_cfg["actions"]["JointPositionAction"];
        common_contract_.scale = action_cfg["scale"].as<std::vector<float>>();
        common_contract_.offset = action_cfg["offset"].as<std::vector<float>>();
        validate_joint_action_contract(common_contract_, action_size_);
        const auto common_stiffness = deploy_cfg["stiffness"].as<std::vector<float>>();
        const auto common_damping = deploy_cfg["damping"].as<std::vector<float>>();
        validate_pd_vector(common_stiffness, "deploy stiffness");
        validate_pd_vector(common_damping, "deploy damping");

        std::vector<std::string> expert_names;
        for (const auto& expert_cfg : expert_cfgs) {
            RmaRegisteredExpert expert;
            expert.name = expert_cfg["name"].as<std::string>();
            if (expert.name.empty()
                || std::find(expert_names.begin(), expert_names.end(), expert.name)
                    != expert_names.end()) {
                throw std::runtime_error("RMA-meta expert name is empty or duplicated");
            }
            expert_names.push_back(expert.name);
            expert.model = std::make_unique<RmaOrtModel>(
                policy_dir / expert_cfg["model"].as<std::string>());
            if (expert.model->input_count() != 1 || expert.model->output_count() != 1
                || expert.model->input_size(0) != observation_size_
                || expert.model->output_size(0) != action_size_) {
                throw std::runtime_error("RMA-meta expert tensor contract is invalid");
            }
            expert.contract.scale = parse_csv_floats(
                expert.model->metadata("action_scale"), "action_scale");
            expert.contract.offset = parse_csv_floats(
                expert.model->metadata("default_joint_pos"), "default_joint_pos");
            expert.joint_names = expert.model->metadata("joint_names");
            expert.observation_names = expert.model->metadata("observation_names");
            if (expert.joint_names.empty() || expert.observation_names.empty()) {
                throw std::runtime_error("RMA-meta expert is missing ONNX metadata");
            }
            if (!experts_.empty()
                && (expert.joint_names != experts_.front().joint_names
                    || expert.observation_names != experts_.front().observation_names)) {
                throw std::runtime_error("RMA-meta expert observation/joint order differs");
            }
            validate_joint_action_contract(expert.contract, action_size_);
            validate_pd_match(
                expert.name,
                "stiffness",
                parse_csv_floats(
                    expert.model->metadata("joint_stiffness"), "joint_stiffness"),
                common_stiffness);
            validate_pd_match(
                expert.name,
                "damping",
                parse_csv_floats(
                    expert.model->metadata("joint_damping"), "joint_damping"),
                common_damping);
            experts_.push_back(std::move(expert));
        }
        const auto declared_experts = cfg["expert_names"].as<std::vector<std::string>>();
        auto expected_classes = expert_names;
        expected_classes.push_back("HOLD");
        if (declared_experts != expert_names
            || cfg["class_names"].as<std::vector<std::string>>() != expected_classes) {
            throw std::runtime_error("RMA-meta registry order/classes are invalid");
        }
        const std::string initial_name = cfg["initial_expert"].as<std::string>();
        auto initial = std::find(expert_names.begin(), expert_names.end(), initial_name);
        if (initial == expert_names.end()) {
            throw std::runtime_error("RMA-meta initial_expert is not registered");
        }
        initial_expert_ = static_cast<int>(std::distance(expert_names.begin(), initial));
        if (!via_expert_name_.empty()) {
            auto via = std::find(
                expert_names.begin(), expert_names.end(), via_expert_name_);
            if (via == expert_names.end()) {
                throw std::runtime_error(
                    "handover_via_expert does not name a registered expert");
            }
            via_index_ = static_cast<int>(std::distance(expert_names.begin(), via));
        }
        validate_model_metadata(expert_names, expected_classes);
        observation_history_.assign(
            history_steps_, std::vector<float>(observation_size_, 0.0f));
        action.assign(action_size_, 0.0f);
        crossfade_from_action_ = action;
        reset();
    }

    void reset() override
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        clear_history();
        std::fill(action.begin(), action.end(), 0.0f);
        crossfade_from_action_ = action;
        crossfade_step_ = crossfade_steps_;
        selected_expert_ = initial_expert_;
        low_level_step_ = 0;
        consecutive_invalid_steps_.store(0);
        safety_fault_latched_.store(false);
    }

    bool fault_latched() const override { return safety_fault_latched_.load(); }

    int selected_expert() const { return selected_expert_; }
    int consecutive_invalid_steps() const { return consecutive_invalid_steps_.load(); }

    std::vector<float> act(
        std::unordered_map<std::string, std::vector<float>> observations) override
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        const auto start = std::chrono::steady_clock::now();
        const auto found = observations.find(observation_key_);
        if (found == observations.end() || found->second.size() != observation_size_) {
            throw std::runtime_error("RMA-meta canonical observation is missing/wrong");
        }
        const auto& observation = found->second;
        if (safety_fault_latched_.load()) return action;
        if (!all_finite(observation)) {
            note_invalid_step();
            clear_history();
            return action;
        }
        observation_history_.erase(observation_history_.begin());
        observation_history_.push_back(observation);

        if (low_level_step_ % meta_decimation_ == 0) {
            std::vector<float> flattened_history;
            flattened_history.reserve(history_steps_ * observation_size_);
            for (const auto& item : observation_history_) {
                flattened_history.insert(
                    flattened_history.end(), item.begin(), item.end());
            }
            const auto latent = adapter_.run({flattened_history}).at(0);
            std::vector<float> previous_one_hot(experts_.size(), 0.0f);
            previous_one_hot.at(static_cast<std::size_t>(selected_expert_)) = 1.0f;
            const auto probabilities = selector_.run(
                {observation, latent, previous_one_hot}).at(0);
            if (!valid_probabilities(probabilities)) {
                note_invalid_step();
                clear_history();
                return action;
            }
            consecutive_invalid_steps_.store(0);
            auto best = std::max_element(probabilities.begin(), probabilities.end());
            const float confidence = *best;
            const int proposed = static_cast<int>(
                std::distance(probabilities.begin(), best));
            // The last class is HOLD.  Finite low-confidence output is also HOLD,
            // not a safety fault and never selects a different expert.
            if (confidence >= confidence_threshold_
                && proposed >= 0
                && proposed < static_cast<int>(experts_.size())
                && proposed != selected_expert_) {
                int next = proposed;
                // slope_up <-> slope_down hold opposite postures; measured in
                // simulation a direct swap falls at 0.0102-0.0119 per step
                // against a 0.0036 baseline, while a handover into the neutral
                // expert falls at 0.0014-0.0018.  Go through the neutral one.
                if (via_index_ >= 0 && selected_expert_ != via_index_
                    && proposed != via_index_) {
                    next = via_index_;
                    pending_expert_ = proposed;
                } else {
                    pending_expert_ = -1;
                }
                crossfade_from_expert_ = selected_expert_;
                selected_expert_ = next;
                crossfade_from_action_ = action;
                crossfade_step_ = 0;
            }
        }

        const auto& expert = experts_.at(static_cast<std::size_t>(selected_expert_));
        const auto expert_observation = adapt_expert_observation(
            observation,
            common_contract_,
            expert.contract,
            joint_position_offset_,
            previous_action_offset_);
        const auto raw_expert_action = expert.model->run({expert_observation}).at(0);
        if (!all_finite(raw_expert_action)) {
            note_invalid_step();
            return action;
        }
        const auto target = raw_action_to_joint_target(
            raw_expert_action, expert.contract);
        auto common_action = joint_target_to_raw_action(target, common_contract_);
        if (crossfade_step_ < crossfade_steps_) {
            // Keep the outgoing expert live for the blend.  Fading in from the
            // action captured at the switch holds a stale pose while the body
            // keeps moving; in simulation that made the fall rate grow with
            // crossfade length (0.53 at 0.2 s, 0.98 at 1.2 s).
            std::vector<float> from_action = crossfade_from_action_;
            if (crossfade_from_expert_ >= 0
                && crossfade_from_expert_ < static_cast<int>(experts_.size())) {
                const auto& previous = experts_.at(
                    static_cast<std::size_t>(crossfade_from_expert_));
                const auto previous_observation = adapt_expert_observation(
                    observation, common_contract_, previous.contract,
                    joint_position_offset_, previous_action_offset_);
                const auto previous_raw =
                    previous.model->run({previous_observation}).at(0);
                if (all_finite(previous_raw)) {
                    from_action = joint_target_to_raw_action(
                        raw_action_to_joint_target(previous_raw, previous.contract),
                        common_contract_);
                }
            }
            ++crossfade_step_;
            const float alpha = static_cast<float>(crossfade_step_) / crossfade_steps_;
            for (std::size_t joint = 0; joint < action.size(); ++joint) {
                action[joint] = (1.0f - alpha) * from_action[joint]
                    + alpha * common_action[joint];
            }
            if (crossfade_step_ >= crossfade_steps_) {
                crossfade_from_expert_ = -1;
                if (pending_expert_ >= 0 && pending_expert_ != selected_expert_) {
                    // The neutral detour is a transient, not a destination:
                    // continue to the expert that was actually requested.
                    crossfade_from_expert_ = selected_expert_;
                    selected_expert_ = pending_expert_;
                    crossfade_from_action_ = action;
                    crossfade_step_ = 0;
                }
                pending_expert_ = -1;
            }
        } else {
            action = std::move(common_action);
        }
        ++low_level_step_;
        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed_ms > 20.0 && !latency_warning_emitted_) {
            std::cerr << "[WARN] RMA-meta inference exceeded 20 ms: "
                      << elapsed_ms << " ms\n";
            latency_warning_emitted_ = true;
        }
        return action;
    }

private:
    static bool all_finite(const std::vector<float>& values)
    {
        return std::all_of(values.begin(), values.end(), [](float value) {
            return std::isfinite(value);
        });
    }

    bool valid_probabilities(const std::vector<float>& values) const
    {
        if (values.size() != experts_.size() + 1 || !all_finite(values)) return false;
        float sum = 0.0f;
        for (float value : values) {
            if (value < 0.0f || value > 1.0f) return false;
            sum += value;
        }
        return std::abs(sum - 1.0f) <= 1.0e-3f;
    }

    static std::vector<std::string> split_csv(const std::string& value)
    {
        std::vector<std::string> output;
        std::stringstream stream(value);
        std::string item;
        while (std::getline(stream, item, ',')) output.push_back(item);
        return output;
    }

    std::vector<float> parse_csv_floats(
        const std::string& value, const std::string& key) const
    {
        if (value.empty()) throw std::runtime_error("ONNX is missing " + key);
        std::vector<float> output;
        std::stringstream stream(value);
        std::string item;
        while (std::getline(stream, item, ',')) output.push_back(std::stof(item));
        if (output.size() != action_size_ || !all_finite(output)) {
            throw std::runtime_error("ONNX metadata " + key + " is invalid");
        }
        return output;
    }

    void validate_pd_vector(
        const std::vector<float>& values, const std::string& name) const
    {
        if (values.size() != action_size_ || !all_finite(values)) {
            throw std::runtime_error(name + " has invalid size/value");
        }
    }

    void validate_pd_match(
        const std::string& expert_name,
        const std::string& term,
        const std::vector<float>& expert_values,
        const std::vector<float>& common_values) const
    {
        float difference = 0.0f;
        for (std::size_t index = 0; index < action_size_; ++index) {
            difference = std::max(
                difference, std::abs(expert_values[index] - common_values[index]));
        }
        if (difference > 1.0e-4f) {
            const std::string message = "RMA-meta expert " + expert_name + " "
                + term + " differs from common PD; max_abs_diff="
                + std::to_string(difference);
            if (!allow_pd_mismatch_) throw std::runtime_error(message);
            std::cerr << "[WARN] " << message << "\n";
        }
    }

    void validate_model_metadata(
        const std::vector<std::string>& expert_names,
        const std::vector<std::string>& class_names) const
    {
        const auto validate = [&](const RmaOrtModel& model, const std::string& module) {
            if (model.metadata("rma_meta_config_version") != "1"
                || model.metadata("module") != module
                || split_csv(model.metadata("expert_names")) != expert_names
                || split_csv(model.metadata("class_names")) != class_names
                || model.metadata("history_steps") != std::to_string(history_steps_)
                || model.metadata("observation_dim") != std::to_string(observation_size_)
                || model.metadata("latent_dim") != std::to_string(latent_size_)) {
                throw std::runtime_error(
                    "RMA-meta ONNX metadata does not match deployment config");
            }
        };
        validate(adapter_, "adaptation");
        validate(selector_, "meta_selector");
        if (adapter_.metadata("checkpoint_sha256")
                != selector_.metadata("checkpoint_sha256")
            || adapter_.metadata("registry_sha256")
                != selector_.metadata("registry_sha256")) {
            throw std::runtime_error("RMA adapter and selector provenance differ");
        }
    }

    void clear_history()
    {
        for (auto& item : observation_history_) {
            std::fill(item.begin(), item.end(), 0.0f);
        }
    }

    void note_invalid_step()
    {
        const int count = consecutive_invalid_steps_.fetch_add(1) + 1;
        if (count >= safety_hold_warning_steps_
            && !safety_fault_latched_.exchange(true)) {
            std::cerr << "[ERROR] RMA-meta invalid inference persisted; holding "
                         "the last finite target and requiring operator attention\n";
        }
    }

    std::vector<RmaRegisteredExpert> experts_;
    RmaOrtModel adapter_;
    RmaOrtModel selector_;
    JointActionContract common_contract_;
    std::string observation_key_ = "obs";
    std::size_t observation_size_ = 83;
    std::size_t action_size_ = 24;
    std::size_t latent_size_ = 8;
    std::size_t history_steps_ = 50;
    std::size_t joint_position_offset_ = 11;
    std::size_t previous_action_offset_ = 59;
    std::vector<std::vector<float>> observation_history_;
    std::vector<float> crossfade_from_action_;
    // Handover state: the outgoing expert stays live for the length of the
    // crossfade, and a direct expert-to-expert change is routed through the
    // neutral expert with the requested one queued behind it.
    int crossfade_from_expert_ = -1;
    std::string via_expert_name_;
    int via_index_ = -1;
    int pending_expert_ = -1;
    float confidence_threshold_ = 0.55f;
    int initial_expert_ = 0;
    int selected_expert_ = 0;
    int meta_decimation_ = 5;
    int crossfade_steps_ = 10;
    int crossfade_step_ = 10;
    int low_level_step_ = 0;
    int safety_hold_warning_steps_ = 100;
    bool allow_pd_mismatch_ = false;
    bool latency_warning_emitted_ = false;
    std::atomic<int> consecutive_invalid_steps_{0};
    std::atomic<bool> safety_fault_latched_{false};
};

}  // namespace isaaclab
