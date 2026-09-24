#include "controllers/locomotion/GaitModeScheduler.hpp"

#include <array>
#include <iostream>
#include <stdexcept>

namespace {

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    GaitModeScheduler scheduler;
    scheduler.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                         0.25f, 0.60f, 1.50f, 5.00f, 0.20f});
    const std::array<float, 3> gyro{0.0f, 0.0f, 0.0f};
    const std::array<float, GaitModeScheduler::kNumJoints> dq{};
    Check(scheduler.mode() == GaitMode::Walk, "reset waits for first sample");

    scheduler.Update(0.20f, 0.0f, gyro, dq, 0.02f);
    Check(scheduler.mode() == GaitMode::Walk, "command enters Walk");
    scheduler.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Check(scheduler.mode() == GaitMode::W2S, "zero command enters W2S");
    for (int i = 0; i < 76; ++i)
        scheduler.Update(0.0f, 0.0f, gyro, dq, 0.02f);
    Check(scheduler.mode() == GaitMode::Stand, "stable dwell reaches Stand");

    scheduler.Update(0.20f, 0.0f, gyro, dq, 0.02f);
    const std::array<float, 3> unstable_gyro{1.0f, 0.0f, 0.0f};
    scheduler.Update(0.0f, 0.0f, unstable_gyro, dq, 0.02f);
    for (int i = 0; i < 100; ++i)
        scheduler.Update(0.0f, 0.0f, unstable_gyro, dq, 0.02f);
    Check(scheduler.mode() == GaitMode::W2S, "unstable W2S does not stand");
    const auto one_hot = scheduler.OneHot();
    Check(one_hot[2] == 1.0f && one_hot[0] == 0.0f && one_hot[1] == 0.0f,
          "one-hot order is Stand Walk W2S");

    // 09/17 is an opt-in overlay: the old scheduler above still uses its
    // original behavior, while the new contract gates STAND on uprightness
    // and exits STAND immediately on a large tilt.
    GaitModeScheduler recovery;
    recovery.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                         0.25f, 0.60f, 1.50f, 5.00f, 0.20f,
                         true, 0.08727f, true,
                         0.13963f, 0.11345f, 0.75f, 0.50f, 0.30f});
    const std::array<float, 3> upright{0.0f, 0.0f, -1.0f};
    const std::array<float, 3> tilted{0.20f, 0.0f, -0.98f};
    recovery.Reset();
    recovery.Update(0.0f, 0.0f, gyro, dq, tilted, 0.02f);
    Check(recovery.mode() == GaitMode::W2S,
          "09/17 tilted startup must not enter Stand");
    recovery.Update(0.0f, 0.0f, gyro, dq, upright, 0.02f);
    for (int i = 0; i < 76; ++i)
        recovery.Update(0.0f, 0.0f, gyro, dq, upright, 0.02f);
    Check(recovery.mode() == GaitMode::Stand,
          "09/17 upright stable dwell reaches Stand");
    recovery.Update(0.0f, 0.0f, gyro, dq, tilted, 0.02f);
    Check(recovery.mode() == GaitMode::W2S,
          "09/17 immediate tilt exits Stand to W2S");
    return 0;
}
