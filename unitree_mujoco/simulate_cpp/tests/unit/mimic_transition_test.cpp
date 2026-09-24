#include "motion/MimicTransition.hpp"

#include <array>
#include <cassert>
#include <cmath>

int main() {
    const std::array<float, 2> q0{0.2f, -0.4f};
    const std::array<float, 2> dq0{0.3f, -0.2f};
    const std::array<float, 2> q1{1.1f, 0.1f};
    const std::array<float, 2> dq1{-0.1f, 0.4f};
    std::array<float, 2> q{};
    std::array<float, 2> dq{};
    mimic_transition::QuinticBoundary(q0, dq0, q1, dq1, 1.7f, 0.0f, q, dq);
    for (size_t i = 0; i < q.size(); ++i) {
        assert(std::fabs(q[i] - q0[i]) < 1.0e-5f);
        assert(std::fabs(dq[i] - dq0[i]) < 1.0e-5f);
    }
    mimic_transition::QuinticBoundary(q0, dq0, q1, dq1, 1.7f, 1.7f, q, dq);
    for (size_t i = 0; i < q.size(); ++i) {
        assert(std::fabs(q[i] - q1[i]) < 1.0e-5f);
        assert(std::fabs(dq[i] - dq1[i]) < 1.0e-5f);
    }
    assert(std::fabs(mimic_transition::QuinticEase(0.0f)) < 1.0e-6f);
    assert(std::fabs(mimic_transition::QuinticEase(1.0f) - 1.0f) < 1.0e-6f);

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
