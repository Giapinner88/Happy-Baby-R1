#include "controllers/slope_meta/SlopeMetaController.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

void SlopeMetaController::Init(
    const std::string& package_directory,
    Ort::Env& environment,
    const Ort::SessionOptions& options) {
    runtime_ = std::make_unique<r1::slope_meta::SlopeMetaRuntime>(
        package_directory, environment, options);
    command_governor_.Configure(runtime_->CommandConfig());
    if (runtime_->ObservationSize() != static_cast<std::size_t>(GetInputSize())) {
        throw std::runtime_error("slope_meta controller requires an 83-D actor observation");
    }

    const auto& contract = runtime_->CommonContract();
    const auto& stiffness = runtime_->Stiffness();
    const auto& damping = runtime_->Damping();
    std::copy(contract.offset.begin(), contract.offset.end(), default_q_.begin());
    std::copy(contract.scale.begin(), contract.scale.end(), action_scale_.begin());
    std::copy(stiffness.begin(), stiffness.end(), joint_stiffness_.begin());
    std::copy(damping.begin(), damping.end(), joint_damping_.begin());

    std::cout << "[SlopeMeta] loaded package: " << package_directory << "\n"
              << "[SlopeMeta] experts=";
    for (std::size_t index = 0; index < runtime_->ExpertNames().size(); ++index) {
        if (index) std::cout << ',';
        std::cout << runtime_->ExpertNames()[index];
    }
    std::cout << " history=50 actor_obs=83 action=24\n";
}

std::vector<float> SlopeMetaController::Infer(
    const std::vector<float>& observation) {
    if (!runtime_) throw std::runtime_error("slope_meta runtime is not initialized");
    auto action = runtime_->Step(observation);

    const std::string state = runtime_->StartupActive()
        ? "STARTUP"
        : runtime_->SafetyHold()
        ? (runtime_->FaultLatched() ? "FAULT_LATCHED" : "SAFETY_HOLD")
        : runtime_->SelectedClass();
    if (state != last_reported_state_) {
        std::cout << "[SlopeMeta] state=" << state << " weights=";
        const auto& weights = runtime_->Weights();
        for (std::size_t index = 0; index < weights.size(); ++index) {
            if (index) std::cout << ',';
            std::cout << weights[index];
        }
        std::cout << " probabilities=";
        const auto& probabilities = runtime_->Probabilities();
        for (std::size_t index = 0; index < probabilities.size(); ++index) {
            if (index) std::cout << ',';
            std::cout << probabilities[index];
        }
        if (observation.size() >= 9) {
            std::cout << " cmd=(" << observation[6] << ',' << observation[7]
                      << ',' << observation[8] << ") gravity=("
                      << observation[3] << ',' << observation[4] << ','
                      << observation[5] << ')';
        }
        std::cout << '\n';
        last_reported_state_ = state;
    }
    return action;
}

void SlopeMetaController::Reset(const LowState_& current_state) {
    FlatController::Reset(current_state);
    if (runtime_) runtime_->Reset();
    command_governor_.Reset();
    last_reported_state_.clear();
    command_guard_reported_ = false;
}

bool SlopeMetaController::SafetyHold() const {
    return runtime_ && runtime_->SafetyHold();
}

bool SlopeMetaController::FaultLatched() const {
    return runtime_ && runtime_->FaultLatched();
}

std::string SlopeMetaController::SelectedClass() const {
    return runtime_ ? runtime_->SelectedClass() : "UNINITIALIZED";
}

std::vector<std::string> SlopeMetaController::ExpertNames() const {
    return runtime_ ? runtime_->ExpertNames() : std::vector<std::string>{};
}

std::vector<float> SlopeMetaController::ExpertWeights() const {
    return runtime_ ? runtime_->Weights() : std::vector<float>{};
}

std::vector<float> SlopeMetaController::GateProbabilities() const {
    return runtime_ ? runtime_->Probabilities() : std::vector<float>{};
}

std::array<float, 3> SlopeMetaController::FilterCommands(
    const std::array<float, 3>& requested,
    float dt) {
    const bool commands_allowed = runtime_ && !runtime_->SafetyHold()
        && !runtime_->FaultLatched();
    if (!commands_allowed && !command_guard_reported_) {
        std::cout << "[SlopeMeta] command_guard=LOCKED; command forced to zero\n";
        command_guard_reported_ = true;
    } else if (commands_allowed && command_guard_reported_) {
        std::cout << "[SlopeMeta] command_guard=RAMPING; gate ready\n";
        command_guard_reported_ = false;
    }
    return command_governor_.Update(requested, commands_allowed, dt);
}
