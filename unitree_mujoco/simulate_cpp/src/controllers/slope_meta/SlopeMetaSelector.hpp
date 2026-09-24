#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace r1::slope_meta {

// Stateful hysteresis, dwell and cross-fade logic. Class order is the registry
// expert order followed by NEUTRAL.
class SlopeMetaSelector {
public:
    SlopeMetaSelector(const YAML::Node& config, std::size_t expert_count)
        : expert_count_(expert_count), neutral_index_(static_cast<int>(expert_count)) {
        if (expert_count_ == 0) {
            throw std::invalid_argument("meta selector needs at least one expert");
        }

        const float policy_hz = config["policy_hz"].as<float>(50.0f);
        const float cold_start_s = config["cold_start_s"].as<float>(1.0f);
        const float minimum_dwell_s = config["minimum_dwell_s"].as<float>(0.5f);
        const float crossfade_s = config["crossfade_s"].as<float>(0.2f);
        agreement_frames_ = config["agreement_frames"].as<int>(5);
        enter_confidence_ = config["enter_confidence"].as<float>(0.75f);
        exit_confidence_ = config["exit_confidence"].as<float>(0.55f);
        startup_safety_hold_ =
            config["startup_safety_hold"].as<bool>(true);
        hold_on_low_confidence_ =
            config["hold_on_low_confidence"].as<bool>(true);
        allow_neutral_ = config["allow_neutral"].as<bool>(true);

        if (!(policy_hz > 0.0f)) {
            throw std::invalid_argument("meta policy_hz must be positive");
        }
        if (cold_start_s < 0.0f || minimum_dwell_s < 0.0f || crossfade_s < 0.0f) {
            throw std::invalid_argument("meta selector times must be non-negative");
        }
        if (agreement_frames_ <= 0) {
            throw std::invalid_argument("meta agreement_frames must be positive");
        }
        if (!(0.0f <= exit_confidence_ && exit_confidence_ <= enter_confidence_
              && enter_confidence_ <= 1.0f)) {
            throw std::invalid_argument(
                "meta confidence thresholds must satisfy 0 <= exit <= enter <= 1");
        }

        initial_weights_ = config["initial_weights"].as<std::vector<float>>(
            std::vector<float>(expert_count_, 1.0f / expert_count_));
        if (initial_weights_.size() != expert_count_) {
            throw std::invalid_argument("meta initial_weights must match expert count");
        }
        float sum = 0.0f;
        for (const float weight : initial_weights_) {
            if (!std::isfinite(weight) || weight < 0.0f) {
                throw std::invalid_argument(
                    "meta initial_weights must be finite and non-negative");
            }
            sum += weight;
        }
        if (!(sum > 0.0f)) {
            throw std::invalid_argument("meta initial_weights need positive mass");
        }
        for (float& weight : initial_weights_) weight /= sum;

        cold_start_steps_ = SecondsToSteps(cold_start_s, policy_hz);
        minimum_dwell_steps_ = SecondsToSteps(minimum_dwell_s, policy_hz);
        crossfade_steps_ = SecondsToSteps(crossfade_s, policy_hz);
        Reset();
    }

    void Reset() {
        step_count_ = 0;
        dwell_steps_ = 0;
        candidate_frames_ = 0;
        candidate_ = neutral_index_;
        selected_ = neutral_index_;
        last_expert_ = -1;
        safety_hold_ = startup_safety_hold_;
        blend_step_ = crossfade_steps_;
        weights_ = initial_weights_;
        blend_from_ = weights_;
        blend_to_ = weights_;
    }

    void EnterSafetyHold() {
        Reset();
        safety_hold_ = true;
    }

    bool Update(const std::vector<float>& probabilities) {
        const bool previous_hold = safety_hold_;
        bool valid = probabilities.size() == expert_count_ + 1;
        float sum = 0.0f;
        for (const float value : probabilities) {
            valid = valid && std::isfinite(value) && value >= 0.0f && value <= 1.0f;
            sum += value;
        }
        if (!valid || std::abs(sum - 1.0f) > 1.0e-3f) {
            EnterSafetyHold();
            return true;
        }

        const auto best = std::max_element(probabilities.begin(), probabilities.end());
        const float confidence = *best;
        int proposed = static_cast<int>(std::distance(probabilities.begin(), best));
        ++step_count_;
        ++dwell_steps_;
        safety_hold_ = false;

        if (step_count_ <= cold_start_steps_) {
            proposed = neutral_index_;
            safety_hold_ = startup_safety_hold_;
        } else if (!allow_neutral_) {
            const auto best_expert = std::max_element(
                probabilities.begin(), probabilities.begin() + neutral_index_);
            proposed = static_cast<int>(
                std::distance(probabilities.begin(), best_expert));
            if (IsExpert(selected_)
                && probabilities[selected_] >= exit_confidence_) {
                proposed = selected_;
            }
        } else if (IsExpert(selected_)) {
            if (probabilities[selected_] >= exit_confidence_) {
                proposed = selected_;
            } else if (confidence < enter_confidence_) {
                proposed = neutral_index_;
                safety_hold_ = hold_on_low_confidence_;
            }
        } else if (confidence < enter_confidence_) {
            proposed = neutral_index_;
            safety_hold_ = hold_on_low_confidence_;
        }

        if (proposed == candidate_) {
            ++candidate_frames_;
        } else {
            candidate_ = proposed;
            candidate_frames_ = 1;
        }

        const bool dwell_ok = selected_ == neutral_index_
            || dwell_steps_ >= minimum_dwell_steps_;
        bool selection_changed = false;
        if (candidate_frames_ >= agreement_frames_
            && dwell_ok
            && candidate_ != selected_) {
            RequestSwitch(candidate_);
            selection_changed = true;
        }
        AdvanceBlend();
        return selection_changed || safety_hold_ != previous_hold;
    }

    int Selected() const { return selected_; }
    int NeutralIndex() const { return neutral_index_; }
    int LastExpert() const { return last_expert_; }
    bool SafetyHold() const { return safety_hold_; }
    bool ColdStartActive() const { return step_count_ <= cold_start_steps_; }
    int CrossfadeSteps() const { return crossfade_steps_; }
    const std::vector<float>& Weights() const { return weights_; }

private:
    bool IsExpert(int value) const {
        return value >= 0 && value < neutral_index_;
    }

    static int SecondsToSteps(float seconds, float hz) {
        return std::max(1, static_cast<int>(std::lround(seconds * hz)));
    }

    std::vector<float> TargetWeights(int target) const {
        if (IsExpert(target)) {
            std::vector<float> result(expert_count_, 0.0f);
            result[target] = 1.0f;
            return result;
        }
        if (last_expert_ >= 0) {
            std::vector<float> result(expert_count_, 0.0f);
            result[last_expert_] = 1.0f;
            return result;
        }
        return initial_weights_;
    }

    void RequestSwitch(int target) {
        blend_from_ = weights_;
        blend_to_ = TargetWeights(target);
        blend_step_ = 0;
        selected_ = target;
        dwell_steps_ = 0;
        if (IsExpert(target)) last_expert_ = target;
    }

    void AdvanceBlend() {
        if (blend_step_ < crossfade_steps_) {
            ++blend_step_;
            const float alpha = static_cast<float>(blend_step_) / crossfade_steps_;
            for (std::size_t index = 0; index < expert_count_; ++index) {
                weights_[index] = (1.0f - alpha) * blend_from_[index]
                    + alpha * blend_to_[index];
            }
        } else {
            weights_ = blend_to_;
        }
    }

    std::size_t expert_count_ = 0;
    int neutral_index_ = 0;
    int cold_start_steps_ = 50;
    int minimum_dwell_steps_ = 25;
    int crossfade_steps_ = 10;
    int agreement_frames_ = 5;
    float enter_confidence_ = 0.75f;
    float exit_confidence_ = 0.55f;
    bool startup_safety_hold_ = true;
    bool hold_on_low_confidence_ = true;
    bool allow_neutral_ = true;
    int step_count_ = 0;
    int dwell_steps_ = 0;
    int candidate_frames_ = 0;
    int blend_step_ = 0;
    int candidate_ = 0;
    int selected_ = 0;
    int last_expert_ = -1;
    bool safety_hold_ = true;
    std::vector<float> initial_weights_;
    std::vector<float> weights_;
    std::vector<float> blend_from_;
    std::vector<float> blend_to_;
};

}  // namespace r1::slope_meta
