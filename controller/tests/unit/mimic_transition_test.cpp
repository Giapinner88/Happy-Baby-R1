#include "../../src/motion/MimicTransition.hpp"

#include <array>
#include <cassert>
#include <cmath>

namespace {

void CheckNear(float actual, float expected, float tolerance = 1.0e-5f) {
    assert(std::isfinite(actual));
    assert(std::fabs(actual - expected) <= tolerance);
}

}  // namespace

int main() {
    constexpr float duration = 1.7f;
    const std::array<float, 3> q0{0.2f, -0.4f, 0.7f};
    const std::array<float, 3> dq0{0.3f, -0.2f, 0.0f};
    const std::array<float, 3> q1{1.1f, 0.1f, -0.2f};
    const std::array<float, 3> dq1{-0.1f, 0.4f, 0.2f};
    std::array<float, 3> q{};
    std::array<float, 3> dq{};

    mimic_transition::QuinticBoundary(q0, dq0, q1, dq1, duration, 0.0f, q, dq);
    for (size_t i = 0; i < q.size(); ++i) {
        CheckNear(q[i], q0[i]);
        CheckNear(dq[i], dq0[i]);
    }

    mimic_transition::QuinticBoundary(q0, dq0, q1, dq1, duration, duration, q, dq);
    for (size_t i = 0; i < q.size(); ++i) {
        CheckNear(q[i], q1[i]);
        CheckNear(dq[i], dq1[i]);
    }

    assert(mimic_transition::QuinticEase(0.0f) == 0.0f);
    assert(mimic_transition::QuinticEase(1.0f) == 1.0f);
    assert(mimic_transition::QuinticEase(0.5f) > 0.49f);
    assert(mimic_transition::QuinticEase(0.5f) < 0.51f);

    float quiet_dwell = 0.0f;
    quiet_dwell = mimic_transition::AdvanceQuietDwell(
        quiet_dwell, 0.1f, true, 0.3f);
    assert(!mimic_transition::QuietDwellReady(quiet_dwell, 0.3f));
    quiet_dwell = mimic_transition::AdvanceQuietDwell(
        quiet_dwell, 0.2f, true, 0.3f);
    assert(mimic_transition::QuietDwellReady(quiet_dwell, 0.3f));
    quiet_dwell = mimic_transition::AdvanceQuietDwell(
        quiet_dwell, 0.1f, false, 0.3f);
    assert(quiet_dwell == 0.0f);
    return 0;
}
