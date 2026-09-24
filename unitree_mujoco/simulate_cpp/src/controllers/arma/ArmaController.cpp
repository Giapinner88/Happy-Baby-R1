#include "controllers/arma/ArmaController.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

void ArmaController::Init(const std::string& package_directory,
                          Ort::Env& environment,
                          const Ort::SessionOptions& options) {
    runtime_ = std::make_unique<r1::arma::ArmaRuntime>(
        package_directory, environment, options);
    actor_input_dim_ = runtime_->ActorInputSize();
    actor_base_dim_ = runtime_->ActorBaseDim();
    actor_history_ = r1::history::BaseObservationHistory(
        runtime_->ActorHistorySteps());
    const auto& contract = runtime_->Contract();
    std::copy(contract.default_position.begin(), contract.default_position.end(),
              default_q_.begin());
    std::copy(contract.action_scale.begin(), contract.action_scale.end(),
              action_scale_.begin());
    std::copy(contract.stiffness.begin(), contract.stiffness.end(),
              joint_stiffness_.begin());
    std::copy(contract.damping.begin(), contract.damping.end(),
              joint_damping_.begin());
    last_arma_action_.assign(R1Config::NUM_JOINTS, 0.0f);
    actor_history_.Reset();
    std::cout << "[ArmaController] " << runtime_->BundleContract()
              << " actor=" << actor_input_dim_ << " history="
              << runtime_->HistorySteps() << "x83 latent="
              << runtime_->LatentSize() << " base_contract="
              << runtime_->BasePolicyContract() << " view="
              << runtime_->ActorView() << " loaded\n";
}

std::vector<float> ArmaController::BuildActorBaseObservation(
    const std::vector<float>& frame) {
    const std::string& view = runtime_->ActorView();
    if (view == "base83") return frame;

    actor_history_.Append(frame);
    std::vector<float> actor_base;
    actor_history_.PackTermMajor(actor_base);
    if (view == "flat_plus_gait_h4") {
        const auto& gait_mode = CurrentGaitMode();
        actor_base.insert(actor_base.end(), gait_mode.begin(), gait_mode.end());
    }
    return actor_base;
}

std::vector<float> ArmaController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) {
    if (!runtime_) throw std::runtime_error("A-RMA controller is not initialized");
    const auto frame = BuildBaseObservation83(
        robot_state, sport_state, target_vx, target_vy, target_yaw,
        gait_time, gait_phase);
    const auto latent = runtime_->Adapt(frame);

    std::vector<float> actor_observation = BuildActorBaseObservation(frame);
    actor_observation.insert(actor_observation.end(), latent.begin(), latent.end());
    if (static_cast<int>(actor_observation.size()) != actor_input_dim_)
        throw std::runtime_error("A-RMA composed actor observation size mismatch");
    return actor_observation;
}

std::vector<float> ArmaController::Infer(const std::vector<float>& observation) {
    if (!runtime_) throw std::runtime_error("A-RMA controller is not initialized");
    if (runtime_->SafetyHold()) return last_arma_action_;
    try {
        const auto action = runtime_->RunActor(observation);
        last_arma_action_ = action;
        return action;
    } catch (const std::exception&) {
        return last_arma_action_;
    }
}

void ArmaController::Reset(const LowState_& current_state) {
    FlatController::Reset(current_state);
    if (runtime_) runtime_->Reset();
    actor_history_.Reset();
    last_arma_action_.assign(R1Config::NUM_JOINTS, 0.0f);
}
