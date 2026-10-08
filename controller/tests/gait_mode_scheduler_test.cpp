#include "../src/gait/GaitModeScheduler.hpp"
#include "../src/gait/GaitScheduler.hpp"

#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
void Require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::abort();
    }
}
}  // namespace

int main() {
    GaitModeScheduler scheduler;
    scheduler.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                         0.25f, 0.60f, 1.50f, 5.00f, 0.20f});
    const std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};
    const std::array<float, GaitModeScheduler::kNumJoints> dq{};

    Require(scheduler.mode() == GaitMode::Walk, "reset waits for first sample");
    scheduler.Update(0.20f, 0.0f, gyro, dq, 0.02f);
    Require(scheduler.mode() == GaitMode::Walk, "command enters Walk");
    scheduler.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Require(scheduler.mode() == GaitMode::W2S, "zero command enters W2S");
    for (int i = 0; i < 76; ++i)
        scheduler.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Require(scheduler.mode() == GaitMode::Stand, "stable dwell reaches Stand");

    GaitModeScheduler configurable_settle;
    configurable_settle.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                                   0.25f, 0.60f, 0.50f, 5.00f, 0.20f});
    configurable_settle.Update(0.20f, 0.0f, gyro, dq, 0.02f);
    configurable_settle.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    for (int i = 0; i < 24; ++i)
        configurable_settle.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Require(configurable_settle.mode() == GaitMode::W2S,
            "0.5 s configured dwell must not finish before 0.5 s");
    configurable_settle.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Require(configurable_settle.mode() == GaitMode::Stand,
            "0.5 s configured dwell reaches Stand after 0.5 s");

    scheduler.Update(0.20f, 0.0f, gyro, dq, 0.02f);
    const std::array<float, 3> unstable_gyro{1.0f, 0.0f, 0.0f};
    scheduler.Update(0.0f, 0.0f, unstable_gyro, dq, 0.02f);
    for (int i = 0; i < 100; ++i)
        scheduler.Update(0.0f, 0.0f, unstable_gyro, dq, 0.02f);
    Require(scheduler.mode() == GaitMode::W2S, "unstable W2S does not stand");

    const auto one_hot = scheduler.OneHot();
    Require(one_hot[2] == 1.0f && one_hot[0] == 0.0f && one_hot[1] == 0.0f,
            "one-hot order is Stand Walk W2S");

    // 09/17 is opt-in: gate STAND on uprightness and exit STAND when pushed.
    GaitModeScheduler recovery;
    recovery.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                        0.25f, 0.60f, 1.50f, 5.00f, 0.20f,
                        true, 0.08727f, true,
                        0.13963f, 0.11345f, 0.75f, 0.50f, 0.30f});
    const std::array<float, 3> upright{0.0f, 0.0f, -1.0f};
    const std::array<float, 3> tilted{0.20f, 0.0f, -0.98f};
    GaitModeScheduler legacy_tilt;
    legacy_tilt.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                           0.25f, 0.60f, 1.50f, 5.00f, 0.20f});
    legacy_tilt.Update(0.0f, 0.0f, gyro, dq, tilted, 0.02f);
    Require(legacy_tilt.mode() == GaitMode::Stand,
            "legacy contract must not enable the 09/17 tilt gate");

    recovery.Reset();
    recovery.Update(0.0f, 0.0f, gyro, dq, tilted, 0.02f);
    Require(recovery.mode() == GaitMode::W2S,
            "09/17 tilted startup must not enter Stand");
    recovery.Update(0.0f, 0.0f, gyro, dq, upright, 0.02f);
    for (int i = 0; i < 76; ++i)
        recovery.Update(0.0f, 0.0f, gyro, dq, upright, 0.02f);
    Require(recovery.mode() == GaitMode::Stand,
            "09/17 upright stable dwell reaches Stand");
    recovery.Update(0.0f, 0.0f, gyro, dq, tilted, 0.02f);
    Require(recovery.mode() == GaitMode::W2S,
            "09/17 immediate tilt exits Stand to W2S");

    GaitModeScheduler sustained;
    sustained.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                         0.25f, 0.60f, 1.50f, 5.00f, 0.20f,
                         true, 0.08727f, true,
                         0.13963f, 0.11345f, 0.75f, 0.50f, 0.30f});
    sustained.Update(0.0f, 0.0f, gyro, dq, upright, 0.02f);
    Require(sustained.mode() == GaitMode::Stand,
            "09/17 upright startup enters Stand");
    const std::array<float, 3> mildly_tilted{0.12f, 0.0f, -0.9928f};
    for (int i = 0; i < 14; ++i)
        sustained.Update(0.0f, 0.0f, gyro, dq, mildly_tilted, 0.02f);
    Require(sustained.mode() == GaitMode::Stand,
            "09/17 sustained tilt must respect its dwell");
    sustained.Update(0.0f, 0.0f, gyro, dq, mildly_tilted, 0.02f);
    Require(sustained.mode() == GaitMode::W2S,
            "09/17 sustained tilt exits Stand after dwell");

    // Phase is zero/frozen in STAND and restarts from zero on STAND exit.
    GaitScheduler phase;
    phase.Configure(0.60f);
    phase.UpdateStandRecovery(false, false, 0.02f);
    Require(phase.time_s() > 0.0f, "09/17 phase advances outside Stand");
    phase.UpdateStandRecovery(false, true, 0.02f);
    const float frozen_time = phase.time_s();
    phase.UpdateStandRecovery(true, true, 0.02f);
    Require(phase.time_s() == frozen_time, "09/17 phase freezes in Stand");
    const auto stand_phase = phase.PhaseObsStandRecovery(true);
    Require(stand_phase[0] == 0.0f && stand_phase[1] == 0.0f,
            "09/17 phase observation is zero in Stand");
    phase.UpdateStandRecovery(true, false, 0.02f);
    Require(phase.time_s() == 0.0f, "09/17 phase resets on Stand exit");
    const auto restart_phase = phase.PhaseObsStandRecovery(false);
    Require(restart_phase[0] == 0.0f && restart_phase[1] == 1.0f,
            "09/17 phase restarts at sin=0 cos=1");
    std::cout << "gait_mode_scheduler_test: PASS\n";
    return 0;
}
