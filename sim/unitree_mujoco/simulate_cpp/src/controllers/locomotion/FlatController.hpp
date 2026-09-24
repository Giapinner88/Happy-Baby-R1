#ifndef FLAT_CONTROLLER_HPP
#define FLAT_CONTROLLER_HPP

#include "runtime/PolicyRunner.hpp"
#include "runtime/R1Config.hpp"

class FlatController : public PolicyRunner {
public:
    static constexpr int kObsSize = 83;

    FlatController();
    ~FlatController() override = default;

    void Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) override;

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase
    ) override;

    std::array<float, R1Config::NUM_JOINTS> ComputeTargetQ(const std::vector<float>& action) override;

    void Reset(const LowState_& current_state) override;

    int GetInputSize() const override { return kObsSize; }
    bool SupportsArmOverlay() const override { return true; }

protected:
    // Pure 83-D base frame. Always sized kObsSize regardless of what a derived
    // controller reports as its policy input size, so a history controller can
    // reuse the gravity/joint-order/last-action formulas instead of restating
    // them. Golden-tested in tests/unit/flat_plus_history_test.cpp.
    std::vector<float> BuildBaseObservation83(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase) const;

    std::array<float, R1Config::NUM_JOINTS> last_action_;
};

#endif
