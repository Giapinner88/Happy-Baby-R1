#ifndef FLAT_HISTORY_CONTROLLER_HPP
#define FLAT_HISTORY_CONTROLLER_HPP

#include <algorithm>
#include <string>
#include <vector>

#include "BaseObservationHistory.hpp"
#include "FlatController.hpp"

// Shared implementation for flat history policies. The canonical source is
// always one 83-D frame; only the history view and contract vary.
class FlatHistoryController : public FlatController {
public:
    FlatHistoryController(int history_steps, std::string contract);
    ~FlatHistoryController() override = default;

    void Init(const std::string& model_path, Ort::Env& env,
              const Ort::SessionOptions& session_options) override;

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase) override;

    void Reset(const LowState_& current_state) override;
    int GetInputSize() const override {
        return history_.steps() * r1::history::kBaseDim;
    }
    bool SupportsArmOverlay() const override { return false; }
    void SetHistoryArmMask(float arm_mask_keep) override {
        history_arm_mask_keep_ = std::clamp(arm_mask_keep, 0.0f, 1.0f);
    }

    const r1::history::BaseObservationHistory& history() const { return history_; }
    int HistorySteps() const { return history_.steps(); }

protected:
    virtual int ActorExtraDim() const { return 0; }
    virtual void ValidateVariantMetadata() const {}
    virtual std::string ActorContract() const { return contract_; }
    virtual int ExpectedActorInputDim() const {
        return history_.steps() * r1::history::kBaseDim + ActorExtraDim();
    }

    void ValidateHistoryMetadata() const;
    std::vector<float> BuildHistoryObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase);
    void ApplyHistoryArmMask(std::vector<float>& frame) const;

    std::string contract_;
    r1::history::BaseObservationHistory history_;
    std::vector<float> packed_;
    float history_arm_mask_keep_ = 1.0f;
};

#endif
