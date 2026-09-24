#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "controllers/locomotion/FlatController.hpp"
#include "controllers/slope_meta/SlopeMetaCommandGovernor.hpp"
#include "controllers/slope_meta/SlopeMetaRuntime.hpp"

class SlopeMetaController final : public FlatController {
public:
    void Init(
        const std::string& package_directory,
        Ort::Env& environment,
        const Ort::SessionOptions& options) override;
    std::vector<float> Infer(const std::vector<float>& observation) override;
    void Reset(const LowState_& current_state) override;

    bool SafetyHold() const;
    bool FaultLatched() const;
    std::string SelectedClass() const;
    std::vector<std::string> ExpertNames() const;
    std::vector<float> ExpertWeights() const;
    std::vector<float> GateProbabilities() const;
    std::array<float, 3> FilterCommands(
        const std::array<float, 3>& requested,
        float dt);

private:
    std::unique_ptr<r1::slope_meta::SlopeMetaRuntime> runtime_;
    r1::slope_meta::SlopeMetaCommandGovernor command_governor_;
    std::string last_reported_state_;
    bool command_guard_reported_ = false;
};
