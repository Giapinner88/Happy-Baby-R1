#include "FlatHistoryController.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

void RequireMetadata(const std::string& key,
                     const std::string& actual,
                     const std::string& expected) {
    if (actual != expected) {
        throw std::runtime_error(
            "flat history contract mismatch: " + key + " expected '" + expected
            + "', model has '" + (actual.empty() ? "<missing>" : actual) + "'");
    }
}

}  // namespace

FlatHistoryController::FlatHistoryController(int history_steps, std::string contract)
    : contract_(std::move(contract)), history_(history_steps) {}

void FlatHistoryController::Init(const std::string& model_path, Ort::Env& env,
                                 const Ort::SessionOptions& session_options) {
    OpenModel(model_path, env, session_options);
    ValidateSingleModelContract(GetInputSize());
    ValidateHistoryMetadata();
    ValidateVariantMetadata();

    default_q_ = R1Config::DEFAULT_JOINT_POS;
    action_scale_ = R1Config::ACTION_SCALE;
    joint_stiffness_ = R1Config::KP_ARRAY;
    joint_damping_ = R1Config::KD_ARRAY;
    LoadMetadata();
    history_.Reset();
    std::cout << "[FlatHistory] " << contract_ << " (" << GetInputSize()
              << "-D, history=" << history_.steps() << ") loaded: "
              << model_path << std::endl;
}

void FlatHistoryController::ValidateHistoryMetadata() const {
    RequireMetadata("policy_contract", ModelMetadata("policy_contract"), contract_);
    RequireMetadata("actor_input_schema", ModelMetadata("actor_input_schema"),
                    ActorContract());
    RequireMetadata("base_observation_schema", ModelMetadata("base_observation_schema"),
                    r1::history::kBaseSchema);
    RequireMetadata("actor_history_order", ModelMetadata("actor_history_order"),
                    r1::history::kH4Order);
    RequireMetadata("actor_history_padding", ModelMetadata("actor_history_padding"),
                    r1::history::kH4Padding);
    RequireMetadata("actor_history_steps", ModelMetadata("actor_history_steps"),
                    std::to_string(history_.steps()));
    RequireMetadata("actor_input_dim", ModelMetadata("actor_input_dim"),
                    std::to_string(ExpectedActorInputDim()));
    RequireMetadata("base_observation_dim", ModelMetadata("base_observation_dim"),
                    std::to_string(r1::history::kBaseDim));

    std::vector<std::string> terms;
    terms.reserve(r1::history::kTerms.size());
    for (const auto& term : r1::history::kTerms) terms.emplace_back(term.name);
    if (ModelMetadata("observation_terms").empty()) {
        throw std::runtime_error("flat history contract missing observation_terms metadata");
    }
    ValidateStringListMetadata("observation_terms", terms);
}

std::vector<float> FlatHistoryController::BuildHistoryObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) {
    auto frame = BuildBaseObservation83(
        robot_state, sport_state, target_vx, target_vy, target_yaw,
        gait_time, gait_phase);
    ApplyHistoryArmMask(frame);
    history_.Append(frame);
    history_.PackTermMajor(packed_);
    return packed_;
}

std::vector<float> FlatHistoryController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) {
    return BuildHistoryObservation(robot_state, sport_state, target_vx, target_vy,
                                   target_yaw, gait_time, gait_phase);
}

void FlatHistoryController::Reset(const LowState_& current_state) {
    FlatController::Reset(current_state);
    history_.Reset();
    history_arm_mask_keep_ = 1.0f;
}

void FlatHistoryController::ApplyHistoryArmMask(std::vector<float>& frame) const {
    // The canonical 83-D frame is [gyro, gravity, command, phase, q, dq, action].
    // Mask before packing term-major; fixed offsets in a packed H4 vector are wrong.
    constexpr int kJointPosOffset = 11;
    constexpr int kJointVelOffset = 35;
    for (int joint = 14; joint < R1Config::NUM_JOINTS; ++joint) {
        frame[static_cast<std::size_t>(kJointPosOffset + joint)] *= history_arm_mask_keep_;
        frame[static_cast<std::size_t>(kJointVelOffset + joint)] *= history_arm_mask_keep_;
    }
}
