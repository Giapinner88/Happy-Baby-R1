#pragma once

// Shared implementation for Flat-Plus history contracts. The canonical input
// is always one 83-D base frame; H4/H5 only change the history view.

#include <string>
#include <utility>
#include <vector>

#include "BaseObservation83.hpp"
#include "BaseObservationHistory.hpp"
#include "PolicyController.hpp"

class FlatHistoryController : public PolicyController {
public:
    FlatHistoryController(int history_steps, std::string contract)
        : history_(history_steps), contract_(std::move(contract)) {}
    ~FlatHistoryController() override = default;

    int ObsSize() const override {
        return history_.steps() * r1::history::kBaseDim + ActorExtraDim();
    }
    std::string Name() const override { return contract_; }
    bool SupportsArmOverlay() const override { return false; }

    void Reset(const RobotState& state) override {
        PolicyController::Reset(state);
        history_.Reset();
        ResetVariant();
    }

    const r1::history::BaseObservationHistory& history() const { return history_; }
    int HistorySteps() const { return history_.steps(); }

protected:
    virtual int ActorExtraDim() const { return 0; }
    virtual void ResetVariant() {}
    virtual void PrepareObservationContext(ControlContext& /*ctx*/) {}
    virtual void AppendActorExtras(const ControlContext& /*ctx*/,
                                   std::vector<float>& /*obs*/) {}
    virtual void ValidateVariantMetadata() {}

    std::string ExpectedPolicyContract() const override { return contract_; }

    void ValidateExtraMetadata(const std::string& model_path) override {
        model_path_ = model_path;
        RequireMetadata(model_path, "actor_input_schema", contract_);
        RequireMetadata(model_path, "base_observation_schema",
                        r1::history::kBaseSchema);
        RequireMetadata(model_path, "actor_history_order",
                        r1::history::kH4Order);
        RequireMetadata(model_path, "actor_history_padding",
                        r1::history::kH4Padding);
        RequireMetadata(model_path, "actor_history_steps",
                        std::to_string(history_.steps()));
        RequireMetadata(model_path, "actor_input_dim", std::to_string(ObsSize()));
        RequireMetadata(model_path, "base_observation_dim",
                        std::to_string(r1::history::kBaseDim));

        std::string terms;
        for (std::size_t i = 0; i < r1::history::kTerms.size(); ++i) {
            if (i) terms += ",";
            terms += r1::history::kTerms[i].name;
        }
        RequireMetadata(model_path, "observation_terms", terms);
        ValidateVariantMetadata();
    }

    void BuildObservation(const ControlContext& ctx,
                          std::vector<float>& obs) override {
        ControlContext prepared = ctx;
        PrepareObservationContext(prepared);
        std::vector<float> frame(r1::history::kBaseDim, 0.0f);
        r1::obs83::Build(prepared, default_q_, last_action_, frame);
        if (prepared.arm_override_active)
            r1::obs83::MaskArmState(frame, prepared.arm_mask_keep);
        history_.Append(frame);  // exactly once per 50 Hz policy step
        history_.PackTermMajor(packed_);
        obs = packed_;
        AppendActorExtras(prepared, obs);
        if (static_cast<int>(obs.size()) != ObsSize()) {
            throw std::runtime_error(
                "history policy built observation size " + std::to_string(obs.size()) +
                ", expected " + std::to_string(ObsSize()));
        }
    }

    std::string contract_;
    std::string model_path_;
    r1::history::BaseObservationHistory history_;
    std::vector<float> packed_;
};
