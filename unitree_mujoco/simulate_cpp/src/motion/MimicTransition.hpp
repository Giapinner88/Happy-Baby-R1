#ifndef MIMIC_TRANSITION_HPP
#define MIMIC_TRANSITION_HPP

#include <algorithm>
#include <array>

namespace mimic_transition {

inline float AdvanceQuietDwell(float dwell_s, float dt_s, bool quiet,
                               float required_s) {
    const float required = std::max(0.0f, required_s);
    if (!quiet) return 0.0f;
    return std::min(required, std::max(0.0f, dwell_s) + std::max(0.0f, dt_s));
}

inline bool QuietDwellReady(float dwell_s, float required_s) {
    return dwell_s >= std::max(0.0f, required_s) - 1.0e-5f;
}

inline float QuinticEase(float u) {
    u = std::clamp(u, 0.0f, 1.0f);
    return u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
}

template <size_t N>
inline void QuinticBoundary(const std::array<float, N>& q0,
                            const std::array<float, N>& dq0,
                            const std::array<float, N>& q1,
                            const std::array<float, N>& dq1,
                            float duration_s,
                            float elapsed_s,
                            std::array<float, N>& q,
                            std::array<float, N>& dq) {
    const float T = std::max(duration_s, 1.0e-5f);
    const float u = std::clamp(elapsed_s / T, 0.0f, 1.0f);
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
    const float h10 = u3 - 2.0f * u2 + u;
    const float h01 = -2.0f * u3 + 3.0f * u2;
    const float h11 = u3 - u2;
    const float dh00 = 6.0f * u2 - 6.0f * u;
    const float dh10 = 3.0f * u2 - 4.0f * u + 1.0f;
    const float dh01 = -6.0f * u2 + 6.0f * u;
    const float dh11 = 3.0f * u2 - 2.0f * u;
    for (size_t i = 0; i < N; ++i) {
        q[i] = h00 * q0[i] + h10 * T * dq0[i] + h01 * q1[i] + h11 * T * dq1[i];
        dq[i] = (dh00 * q0[i] + dh10 * T * dq0[i] +
                 dh01 * q1[i] + dh11 * T * dq1[i]) / T;
    }
}

}  // namespace mimic_transition

#endif
