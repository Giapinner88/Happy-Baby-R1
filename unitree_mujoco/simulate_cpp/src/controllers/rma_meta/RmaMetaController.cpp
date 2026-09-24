#include "controllers/rma_meta/RmaMetaController.hpp"

#include <algorithm>
#include <iostream>
#include <stdexcept>

void RmaMetaController::Init(
    const std::string& package_directory,
    Ort::Env& environment,
    const Ort::SessionOptions& options) {
    runtime_ = std::make_unique<r1::rma_meta::RmaMetaRuntime>(
        package_directory, environment, options);
    command_governor_.Configure(runtime_->CommandConfig());
    if (runtime_->ObservationSize() != static_cast<std::size_t>(GetInputSize())) {
        throw std::runtime_error("RMA-meta controller requires an 83-D observation");
    }

    const auto& contract = runtime_->CommonContract();
    std::copy(contract.offset.begin(), contract.offset.end(), default_q_.begin());
    std::copy(contract.scale.begin(), contract.scale.end(), action_scale_.begin());
    std::copy(
        runtime_->Stiffness().begin(),
        runtime_->Stiffness().end(),
        joint_stiffness_.begin());
    std::copy(
        runtime_->Damping().begin(),
        runtime_->Damping().end(),
        joint_damping_.begin());

    std::cout << "[RmaMeta] loaded package: " << package_directory << "\n"
              << "[RmaMeta] experts=";
    const auto& names = runtime_->ExpertNames();
    for (std::size_t index = 0; index < names.size(); ++index) {
        if (index) std::cout << ',';
        std::cout << names[index];
    }
    std::cout << " history=" << runtime_->HistorySteps()
              << " meta_decimation=" << runtime_->MetaDecimation()
              << " actor_obs=83 latent=8 action=24\n";
}

std::vector<float> RmaMetaController::Infer(
    const std::vector<float>& observation) {
    if (!runtime_) throw std::runtime_error("RMA-meta runtime is not initialized");
    auto action = runtime_->Step(observation);
    const std::string state = runtime_->SafetyHold()
        ? (runtime_->FaultLatched() ? "FAULT_LATCHED" : "SAFETY_HOLD")
        : runtime_->SelectedClass();
    if (state != last_reported_state_) {
        std::cout << "[RmaMeta] state=" << state << " probabilities=";
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

void RmaMetaController::Reset(const LowState_& current_state) {
    FlatController::Reset(current_state);
    if (runtime_) runtime_->Reset();
    command_governor_.Reset();
    last_reported_state_.clear();
    command_guard_reported_ = false;
}

bool RmaMetaController::SafetyHold() const {
    return runtime_ && runtime_->SafetyHold();
}

bool RmaMetaController::FaultLatched() const {
    return runtime_ && runtime_->FaultLatched();
}

std::string RmaMetaController::SelectedClass() const {
    return runtime_ ? runtime_->SelectedClass() : "UNINITIALIZED";
}

std::vector<std::string> RmaMetaController::ExpertNames() const {
    return runtime_ ? runtime_->ExpertNames() : std::vector<std::string>{};
}

std::vector<float> RmaMetaController::ExpertWeights() const {
    return runtime_ ? runtime_->ExpertWeights() : std::vector<float>{};
}

std::vector<float> RmaMetaController::GateProbabilities() const {
    return runtime_ ? runtime_->Probabilities() : std::vector<float>{};
}

std::array<float, 3> RmaMetaController::FilterCommands(
    const std::array<float, 3>& requested,
    float dt) {
    const bool commands_allowed = runtime_ && !runtime_->SafetyHold()
        && !runtime_->FaultLatched();
    if (!commands_allowed && !command_guard_reported_) {
        std::cout << "[RmaMeta] command_guard=LOCKED; command forced to zero\n";
        command_guard_reported_ = true;
    } else if (commands_allowed && command_guard_reported_) {
        std::cout << "[RmaMeta] command_guard=RAMPING; inference valid\n";
        command_guard_reported_ = false;
    }
    return command_governor_.Update(requested, commands_allowed, dt);
}
