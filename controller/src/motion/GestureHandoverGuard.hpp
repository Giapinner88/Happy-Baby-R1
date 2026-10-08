#pragma once

#include <algorithm>

// Thuần logic, không phụ thuộc DDS/ONNX: chỉ tích lũy dwell khi policy target,
// tay thật và IMU đều còn ổn định liên tục. Một mẫu xấu reset đồng hồ handover.
class GestureHandoverGuard {
public:
    explicit GestureHandoverGuard(float dwell_s = 0.5f) { Configure(dwell_s); }

    void Configure(float dwell_s) {
        dwell_s_ = std::max(0.0f, dwell_s);
        stable_for_s_ = 0.0f;
    }

    bool Advance(bool target_matches, float arm_dq_rms, float tilt_rad, float gyro_norm,
                 float arm_dq_limit, float tilt_limit, float gyro_limit, float dt) {
        const bool stable = target_matches
            && arm_dq_rms <= std::max(0.0f, arm_dq_limit)
            && tilt_rad <= std::max(0.0f, tilt_limit)
            && gyro_norm <= std::max(0.0f, gyro_limit);
        stable_for_s_ = stable ? std::min(dwell_s_, stable_for_s_ + std::max(0.0f, dt)) : 0.0f;
        return stable_for_s_ >= dwell_s_;
    }

    float StableForS() const { return stable_for_s_; }
    void Reset() { stable_for_s_ = 0.0f; }

private:
    float dwell_s_ = 0.5f;
    float stable_for_s_ = 0.0f;
};
