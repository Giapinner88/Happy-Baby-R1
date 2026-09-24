#ifndef ARMA_CONTROLLER_HPP
#define ARMA_CONTROLLER_HPP

#include <memory>
#include <vector>

#include "controllers/locomotion/BaseObservationHistory.hpp"
#include "controllers/locomotion/FlatController.hpp"
#include "controllers/arma/ArmaRuntime.hpp"

// Composite low-level controller for one Flat-family A-RMA bundle. This is
// deliberately separate from rma_meta (which selects frozen experts
// categorically): the bundle owns exactly one base actor contract.
class ArmaController final : public FlatController {
public:
    void Init(const std::string& package_directory,
              Ort::Env& environment,
              const Ort::SessionOptions& options) override;

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase) override;

    std::vector<float> Infer(const std::vector<float>& observation) override;
    void Reset(const LowState_& current_state) override;
    int GetInputSize() const override { return actor_input_dim_; }
    bool SupportsArmOverlay() const override { return false; }

    bool SafetyHold() const { return runtime_ && runtime_->SafetyHold(); }
    const std::vector<float>& Latent() const { return runtime_->Latent(); }
    bool GaitConditioned() const { return runtime_ && runtime_->GaitConditioned(); }
    const std::string& BasePolicyContract() const {
        return runtime_->BasePolicyContract();
    }

private:
    std::vector<float> BuildActorBaseObservation(const std::vector<float>& frame);

    std::unique_ptr<r1::arma::ArmaRuntime> runtime_;
    r1::history::BaseObservationHistory actor_history_{r1::history::kH4Steps};
    int actor_input_dim_ = 91;
    int actor_base_dim_ = r1::history::kBaseDim;
    std::vector<float> last_arma_action_;
};

#endif
