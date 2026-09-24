#ifndef FLAT_PLUS_GAIT_CONTROLLER_HPP
#define FLAT_PLUS_GAIT_CONTROLLER_HPP

#include "FlatHistoryController.hpp"

// H4 history plus one current-only STAND/WALK/W2S one-hot.
class FlatPlusGaitController final : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH4ActorDim + 3;  // 335

    FlatPlusGaitController()
        : FlatHistoryController(r1::history::kH4Steps,
                                 "flat_plus_gait_h4_v1") {}
    ~FlatPlusGaitController() override = default;

    int GetInputSize() const override { return kObsSize; }
    bool SupportsHistoryGestureOverlay() const override { return true; }

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase) override;

protected:
    int ActorExtraDim() const override { return 3; }
    std::string ActorContract() const override { return "flat_plus_gait_h4_v1"; }
    void ValidateVariantMetadata() const override;
};

#endif
