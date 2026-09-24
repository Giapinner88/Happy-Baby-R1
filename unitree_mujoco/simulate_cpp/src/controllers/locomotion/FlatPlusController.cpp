#include "FlatPlusController.hpp"

#include <iostream>
#include <stdexcept>

namespace {

void RequireMetadata(const std::string& key,
                     const std::string& actual,
                     const std::string& expected) {
    if (actual != expected) {
        throw std::runtime_error(
            "flat_plus contract mismatch: " + key + " expected '" + expected
            + "', model has '" + (actual.empty() ? "<missing>" : actual) + "'");
    }
}

}  // namespace

void FlatPlusController::Init(const std::string& model_path, Ort::Env& env,
                              const Ort::SessionOptions& session_options) {
    OpenModel(model_path, env, session_options);
    ValidateSingleModelContract(GetInputSize());
    ValidateHistoryMetadata();

    default_q_ = R1Config::DEFAULT_JOINT_POS;
    action_scale_ = R1Config::ACTION_SCALE;
    joint_stiffness_ = R1Config::KP_ARRAY;
    joint_damping_ = R1Config::KD_ARRAY;
    LoadMetadata();

    history_.Reset();
    std::cout << "[FlatPlusController] " << r1::history::kFlatPlusContract
              << " (" << GetInputSize() << "-D, history "
              << r1::history::kH4Steps << " x " << r1::history::kBaseDim
              << ") loaded: " << model_path << std::endl;
}

void FlatPlusController::ValidateHistoryMetadata() const {
    RequireMetadata("policy_contract", ModelMetadata("policy_contract"),
                    r1::history::kFlatPlusContract);
    RequireMetadata("actor_input_schema", ModelMetadata("actor_input_schema"),
                    r1::history::kFlatPlusContract);
    RequireMetadata("base_observation_schema",
                    ModelMetadata("base_observation_schema"),
                    r1::history::kBaseSchema);
    RequireMetadata("actor_history_order", ModelMetadata("actor_history_order"),
                    r1::history::kH4Order);
    RequireMetadata("actor_history_padding",
                    ModelMetadata("actor_history_padding"),
                    r1::history::kH4Padding);
    RequireMetadata("actor_history_steps", ModelMetadata("actor_history_steps"),
                    std::to_string(r1::history::kH4Steps));
    RequireMetadata("actor_input_dim", ModelMetadata("actor_input_dim"),
                    std::to_string(r1::history::kH4ActorDim));
    RequireMetadata("base_observation_dim",
                    ModelMetadata("base_observation_dim"),
                    std::to_string(r1::history::kBaseDim));

    std::vector<std::string> terms;
    terms.reserve(r1::history::kTerms.size());
    for (const auto& term : r1::history::kTerms) terms.emplace_back(term.name);
    ValidateStringListMetadata("observation_terms", terms);
}

std::vector<float> FlatPlusController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) {
    // 1. current canonical frame; `actions` still holds the previous step's raw
    //    action because ComputeTargetQ() has not run for this step yet.
    std::vector<float> frame = BuildBaseObservation83(
        robot_state, sport_state, target_vx, target_vy, target_yaw,
        gait_time, gait_phase);
    ApplyHistoryArmMask(frame);

    // 2. exactly one append per policy step.
    history_.Append(frame);

    // 3. term-major view over the window.
    history_.PackTermMajorH4(packed_);
    return packed_;
}

void FlatPlusController::Reset(const LowState_& current_state) {
    FlatController::Reset(current_state);
    history_.Reset();
}
