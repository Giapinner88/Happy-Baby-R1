#pragma once
#include <cmath>
#include <initializer_list>

// Qualification of a terminal HOME hold, not a general dance shortcut.
inline bool HomeDanceExitReady(float remaining_frames, float reference_gap,
                               float measured_gap, float tilt_rad, float gyro,
                               float max_dq) {
    for (float value : {remaining_frames, reference_gap, measured_gap, tilt_rad, gyro, max_dq})
        if (!std::isfinite(value)) return false;
    return remaining_frames >= 0.0f && remaining_frames <= 25.0f &&
           reference_gap <= 0.10f && measured_gap <= 0.30f &&
           tilt_rad <= 0.15f && gyro <= 0.5f && max_dq <= 0.6f;
}
