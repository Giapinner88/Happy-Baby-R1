#include "controllers/locomotion/GaitModeSchedulerV3.hpp"
#include "runtime/R1Config.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

GaitModeV3Config Config() {
    return GaitModeV3Config{};
}

std::array<float, 12> HomeLegQ() {
    return {{
        -0.1f, 0.0349f, -0.0477f, 0.3f, -0.2f, -0.0349f,
        -0.1f, -0.0349f, 0.0477f, 0.3f, -0.2f, 0.0349f,
    }};
}

std::array<float, 24> QuietDq() { return {}; }

}  // namespace

int main() {
    try {
        const auto q = HomeLegQ();
        const auto dq = QuietDq();
        const std::array<float, 3> quiet_gyro{0.0f, 0.0f, 0.0f};
        const std::array<float, 3> gravity{0.0f, 0.0f, -1.0f};

        LegFK fk;
        const auto home_metrics = fk.Compute(q, gravity);
        Require(std::abs(home_metrics.width_err) < 1.0e-3f,
                "R1 FK home width matches the training contract");
        Require(std::abs(home_metrics.dx_err) < 1.0e-5f
                    && std::abs(home_metrics.dyaw_err) < 1.0e-5f,
                "R1 FK home errors are zero");

        GaitModeSchedulerV3 b(GaitV3Variant::B);
        b.Configure(Config());
        b.Reset();
        b.Update(0.0f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
        Require(b.mode() == GaitMode::Stand, "B reset can enter a valid stand");
        Require(b.PhaseObservation()[0] == 0.0f && b.PhaseObservation()[1] == 0.0f,
                "stand phase is zero");
        b.Update(0.20f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
        Require(b.mode() == GaitMode::Walk, "B move exits stand");
        b.Update(0.0f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
        Require(b.mode() == GaitMode::W2S, "B stop enters W2S");
        Require(b.substate() == GaitW2SSubstate::Settle,
                "B may settle immediately only at double support");

        GaitModeSchedulerV3 f(GaitV3Variant::F);
        f.Configure(Config());
        f.Reset();
        f.Update(0.20f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
        f.Update(0.0f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
        Require(f.mode() == GaitMode::W2S && f.substate() == GaitW2SSubstate::Gather,
                "F always gathers on stop entry");
        bool observed_settle = false;
        float frozen_phase = -1.0f;
        for (int i = 0; i < 160; ++i) {
            f.Update(0.0f, 0.0f, quiet_gyro, dq, q, gravity, 0.02f);
            if (f.substate() == GaitW2SSubstate::Settle) {
                observed_settle = true;
                if (frozen_phase < 0.0f) frozen_phase = f.phase_time_s();
                else Require(std::abs(f.phase_time_s() - frozen_phase) < 1.0e-6f,
                             "F settle freezes the phase clock");
                const auto phase = f.PhaseObservation();
                Require(phase[0] == 0.0f && phase[1] == 0.0f,
                        "F settle phase observation is zero");
                if (f.mode() == GaitMode::Stand) break;
            }
        }
        Require(observed_settle, "F reaches SETTLE after a gather stride");
        Require(f.mode() == GaitMode::Stand, "F stable settle reaches stand");

        const float tilt = 0.20f;
        const std::array<float, 3> tilted_gravity{std::sin(tilt), 0.0f, -std::cos(tilt)};
        f.Update(0.0f, 0.0f, quiet_gyro, dq, q, tilted_gravity, 0.02f);
        Require(f.mode() == GaitMode::W2S && f.substate() == GaitW2SSubstate::Gather,
                "F push-exit returns to W2S gather");

        std::cout << "gait_v3_scheduler_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "gait_v3_scheduler_test: FAIL " << error.what() << '\n';
        return 1;
    }
}
