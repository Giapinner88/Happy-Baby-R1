#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "../config/RobotSpec.hpp"

// Converts a gesture target into the arm_q_ref/arm_dq_ref pair seen by the
// Unified policy. This is deliberately a reference generator, never a direct
// motor overlay. The same class is the future insertion point for live teleop.
class ArmReferenceFilter {
public:
    using Arm = std::array<float, spec::kNumArmJoints>;

    void Init(const Arm& default_q, float max_vel_rad_s, float max_acc_rad_s2) {
        default_q_ = default_q;
        max_vel_rad_s_ = std::clamp(max_vel_rad_s, 0.05f, kTrainMaxVelRadS);
        max_acc_rad_s2_ = std::clamp(max_acc_rad_s2, 0.05f, kTrainMaxAccRadS2);
        Reset();
    }

    void Reset() {
        q_ = default_q_;
        dq_.fill(0.0f);
    }

    void Update(const Arm& desired, float dt) {
        if (!(dt > 0.0f) || !std::isfinite(dt)) return;
        for (int j = 0; j < spec::kNumArmJoints; ++j) {
            const float delta = desired[j] - q_[j];
            const float direction = delta >= 0.0f ? 1.0f : -1.0f;
            // Start braking at the physical stopping distance. Unlike snapping q
            // to desired, this keeps dq_ref continuous during gesture retraction.
            const float braking_velocity = std::sqrt(
                std::max(0.0f, 2.0f * max_acc_rad_s2_ * std::fabs(delta)));
            const float position_limited_velocity = direction * std::min(
                max_vel_rad_s_, braking_velocity);
            const float velocity_step = max_acc_rad_s2_ * dt;
            const float next_dq = dq_[j] + std::clamp(
                position_limited_velocity - dq_[j], -velocity_step, velocity_step);
            // Snap only when both remaining distance and velocity can be removed
            // inside one acceleration-limited tick. Otherwise a tiny overshoot is
            // preferable to emitting an impossible dq_ref discontinuity.
            if (std::fabs(delta) <= velocity_step * dt &&
                std::fabs(dq_[j]) <= velocity_step &&
                std::fabs(next_dq) <= velocity_step) {
                q_[j] = desired[j];
                dq_[j] = 0.0f;
            } else {
                q_[j] += next_dq * dt;
                dq_[j] = next_dq;
            }
        }
    }

    const Arm& q() const { return q_; }
    const Arm& dq() const { return dq_; }
    const Arm& default_q() const { return default_q_; }
    float max_vel_rad_s() const { return max_vel_rad_s_; }
    float max_acc_rad_s2() const { return max_acc_rad_s2_; }

    static constexpr float kTrainMaxVelRadS = 4.0f;
    static constexpr float kTrainMaxAccRadS2 = 14.0f;

private:
    Arm default_q_{};
    Arm q_{};
    Arm dq_{};
    float max_vel_rad_s_ = 3.0f;
    float max_acc_rad_s2_ = 10.0f;
};
