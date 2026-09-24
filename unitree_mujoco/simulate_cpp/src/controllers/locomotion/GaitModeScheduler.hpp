#ifndef GAIT_MODE_SCHEDULER_HPP
#define GAIT_MODE_SCHEDULER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

enum class GaitMode { Stand = 0, Walk = 1, W2S = 2 };

// C++ counterpart of gait_contract.py/gait_command.py.  Linear and angular
// command thresholds are deliberately separate; one 3-D norm is not the same
// predicate used by training.
struct GaitModeConfig {
    float stop_request_lin = 0.10f;
    float stop_request_ang = 0.10f;
    float move_request_lin = 0.15f;
    float move_request_ang = 0.15f;
    float settle_gyro_norm = 0.25f;
    float settle_joint_velocity_rms = 0.60f;
    float settle_dwell_s = 1.50f;
    float w2s_timeout_s = 5.00f;
    float stability_filter_tau_s = 0.20f;
    // Optional 09/17 STAND-recovery overlay. Disabled by default so the
    // original v1 scheduler remains bit-for-bit compatible for old artifacts.
    bool stand_upright_enabled = false;
    float stand_entry_tilt_max_rad = 0.08727f;
    bool stand_push_exit_enabled = false;
    float push_exit_tilt_immediate_rad = 0.13963f;
    float push_exit_tilt_sustained_rad = 0.11345f;
    float push_exit_gyro_immediate = 0.75f;
    float push_exit_gyro_sustained = 0.50f;
    float push_exit_sustain_s = 0.30f;
};

// One deployable FSM for gait-conditioned policies. It consumes only command,
// IMU gyro and encoder velocity; no MuJoCo contact/terrain truth is involved.
// Stability filters signed vectors first and takes norms afterwards, matching
// filtered_imu_ang_vel_norm_and_joint_vel_rms_v2 in the training contract.
class GaitModeScheduler {
public:
    static constexpr std::size_t kNumJoints = 24;

    void Configure(const GaitModeConfig& config) {
        if (!std::isfinite(config.stop_request_lin)
            || !std::isfinite(config.stop_request_ang)
            || !std::isfinite(config.move_request_lin)
            || !std::isfinite(config.move_request_ang)
            || !std::isfinite(config.settle_gyro_norm)
            || !std::isfinite(config.settle_joint_velocity_rms)
            || !std::isfinite(config.settle_dwell_s)
            || !std::isfinite(config.w2s_timeout_s)
            || !std::isfinite(config.stability_filter_tau_s)
            || !std::isfinite(config.stand_entry_tilt_max_rad)
            || !std::isfinite(config.push_exit_tilt_immediate_rad)
            || !std::isfinite(config.push_exit_tilt_sustained_rad)
            || !std::isfinite(config.push_exit_gyro_immediate)
            || !std::isfinite(config.push_exit_gyro_sustained)
            || !std::isfinite(config.push_exit_sustain_s)
            || config.stop_request_lin < 0.0f
            || config.stop_request_ang < 0.0f
            || config.move_request_lin <= config.stop_request_lin
            || config.move_request_ang <= config.stop_request_ang
            || config.settle_gyro_norm < 0.0f
            || config.settle_joint_velocity_rms < 0.0f
            || config.settle_dwell_s <= 0.0f
            || config.w2s_timeout_s <= 0.0f
            || config.stability_filter_tau_s <= 0.0f
            || config.stand_entry_tilt_max_rad < 0.0f
            || config.push_exit_tilt_immediate_rad < 0.0f
            || config.push_exit_tilt_sustained_rad < 0.0f
            || config.push_exit_gyro_immediate < 0.0f
            || config.push_exit_gyro_sustained < 0.0f
            || config.push_exit_sustain_s <= 0.0f) {
            throw std::invalid_argument("invalid gait mode scheduler configuration");
        }
        config_ = config;
    }

    void Reset() {
        // Match ReferenceGaitFsm.reset(): the first sample derives the mode
        // from the final command and the freshly seeded stability filter.
        mode_ = GaitMode::Walk;
        settle_time_s_ = 0.0f;
        w2s_time_s_ = 0.0f;
        stop_elapsed_s_ = 0.0f;
        stand_unstable_time_s_ = 0.0f;
        filtered_gyro_ = {0.0f, 0.0f, 0.0f};
        filtered_joint_velocity_.fill(0.0f);
        filter_primed_ = false;
        needs_init_ = true;
    }

    void Update(float command_linear_norm, float command_yaw_abs,
                const std::array<float, 3>& gyro,
                const std::array<float, kNumJoints>& joint_velocity,
                float dt) {
        // Legacy callers do not have a gravity vector. The upright vector
        // keeps the optional 09/17 overlay inactive for those callers.
        Update(command_linear_norm, command_yaw_abs, gyro, joint_velocity,
               {0.0f, 0.0f, -1.0f}, dt);
    }

    void Update(float command_linear_norm, float command_yaw_abs,
                const std::array<float, 3>& gyro,
                const std::array<float, kNumJoints>& joint_velocity,
                const std::array<float, 3>& projected_gravity,
                float dt) {
        if (!std::isfinite(command_linear_norm)
            || !std::isfinite(command_yaw_abs) || command_linear_norm < 0.0f
            || command_yaw_abs < 0.0f || !std::isfinite(dt) || dt < 0.0f) {
            throw std::invalid_argument("non-finite gait scheduler command input");
        }
        for (const float value : gyro) {
            if (!std::isfinite(value))
                throw std::invalid_argument("non-finite gait scheduler gyro input");
        }
        for (const float value : joint_velocity) {
            if (!std::isfinite(value))
                throw std::invalid_argument("non-finite gait scheduler joint input");
        }
        for (const float value : projected_gravity) {
            if (!std::isfinite(value))
                throw std::invalid_argument("non-finite gait scheduler gravity input");
        }

        UpdateFilter(gyro, joint_velocity, dt);

        const bool stop = command_linear_norm < config_.stop_request_lin
            && command_yaw_abs < config_.stop_request_ang;
        const bool move = command_linear_norm > config_.move_request_lin
            || command_yaw_abs > config_.move_request_ang;

        stop_elapsed_s_ = stop ? stop_elapsed_s_ + dt : 0.0f;
        const bool stable = Stable();
        const float tilt = std::atan2(
            std::hypot(projected_gravity[0], projected_gravity[1]),
            -projected_gravity[2]);
        const bool stand_entry_ok = !config_.stand_upright_enabled
            || tilt <= config_.stand_entry_tilt_max_rad;

        if (needs_init_) {
            if (move) {
                mode_ = GaitMode::Walk;
            } else if (stop && stable && stand_entry_ok) {
                mode_ = GaitMode::Stand;
            } else {
                mode_ = GaitMode::W2S;
            }
            settle_time_s_ = 0.0f;
            w2s_time_s_ = 0.0f;
            needs_init_ = false;
            AdvanceW2STimer(dt);
            return;
        }

        const bool to_walk = move
            && (mode_ == GaitMode::Stand || mode_ == GaitMode::W2S);
        const bool to_w2s = stop && mode_ == GaitMode::Walk;
        const bool settling = mode_ == GaitMode::W2S && stop && stable
            && stand_entry_ok && !to_walk;
        settle_time_s_ = settling ? settle_time_s_ + dt : 0.0f;
        const bool to_stand = settling
            && settle_time_s_ >= config_.settle_dwell_s - 0.5f * dt;

        float filtered_gyro_sq = 0.0f;
        for (const float value : filtered_gyro_) filtered_gyro_sq += value * value;
        const float filtered_gyro_norm = std::sqrt(filtered_gyro_sq);
        const bool is_stand = mode_ == GaitMode::Stand;
        const bool stand_push_immediate =
            tilt > config_.push_exit_tilt_immediate_rad
            || filtered_gyro_norm > config_.push_exit_gyro_immediate;
        const bool stand_push_sustained =
            tilt > config_.push_exit_tilt_sustained_rad
            || filtered_gyro_norm > config_.push_exit_gyro_sustained;
        stand_unstable_time_s_ = config_.stand_push_exit_enabled && is_stand
            && stand_push_sustained
            ? stand_unstable_time_s_ + dt : 0.0f;
        const bool stand_push_exit = config_.stand_push_exit_enabled && is_stand
            && !to_walk
            && (stand_push_immediate || stand_unstable_time_s_ >=
                config_.push_exit_sustain_s - 0.5f * dt);

        if (to_walk) {
            mode_ = GaitMode::Walk;
        } else if (to_w2s || stand_push_exit) {
            mode_ = GaitMode::W2S;
        }
        if (to_stand) {
            mode_ = GaitMode::Stand;
        }
        if (to_walk || to_stand || stand_push_exit) {
            settle_time_s_ = 0.0f;
        }
        if (to_walk || to_w2s || stand_push_exit || !is_stand) {
            stand_unstable_time_s_ = 0.0f;
        }
        AdvanceW2STimer(dt);
    }

    GaitMode mode() const { return mode_; }
    float settle_time_s() const { return settle_time_s_; }
    float w2s_time_s() const { return w2s_time_s_; }
    float stop_elapsed_s() const { return stop_elapsed_s_; }
    const std::array<float, 3>& filtered_gyro() const { return filtered_gyro_; }
    const std::array<float, kNumJoints>& filtered_joint_velocity() const {
        return filtered_joint_velocity_;
    }
    bool stable() const { return Stable(); }

    std::array<float, 3> OneHot() const {
        std::array<float, 3> result{0.0f, 0.0f, 0.0f};
        result[static_cast<std::size_t>(mode_)] = 1.0f;
        return result;
    }

private:
    void AdvanceW2STimer(float dt) {
        w2s_time_s_ = mode_ == GaitMode::W2S ? w2s_time_s_ + dt : 0.0f;
        if (w2s_time_s_ >= config_.w2s_timeout_s) w2s_time_s_ = 0.0f;
    }

    void UpdateFilter(const std::array<float, 3>& gyro,
                      const std::array<float, kNumJoints>& joint_velocity,
                      float dt) {
        if (!filter_primed_) {
            filtered_gyro_ = gyro;
            filtered_joint_velocity_ = joint_velocity;
            filter_primed_ = true;
            return;
        }
        const float alpha = std::min(
            dt / std::max(config_.stability_filter_tau_s, 1.0e-6f), 1.0f);
        for (std::size_t index = 0; index < filtered_gyro_.size(); ++index) {
            filtered_gyro_[index] += alpha * (gyro[index] - filtered_gyro_[index]);
        }
        for (std::size_t index = 0; index < filtered_joint_velocity_.size(); ++index) {
            filtered_joint_velocity_[index] +=
                alpha * (joint_velocity[index] - filtered_joint_velocity_[index]);
        }
    }

    bool Stable() const {
        float gyro_sq = 0.0f;
        for (const float value : filtered_gyro_) gyro_sq += value * value;
        float joint_sq = 0.0f;
        for (const float value : filtered_joint_velocity_) joint_sq += value * value;
        const float joint_rms = std::sqrt(joint_sq / static_cast<float>(kNumJoints));
        return std::sqrt(gyro_sq) < config_.settle_gyro_norm
            && joint_rms < config_.settle_joint_velocity_rms;
    }

    GaitModeConfig config_{};
    GaitMode mode_ = GaitMode::Walk;
    float settle_time_s_ = 0.0f;
    float w2s_time_s_ = 0.0f;
    float stop_elapsed_s_ = 0.0f;
    float stand_unstable_time_s_ = 0.0f;
    std::array<float, 3> filtered_gyro_{0.0f, 0.0f, 0.0f};
    std::array<float, kNumJoints> filtered_joint_velocity_{};
    bool filter_primed_ = false;
    bool needs_init_ = true;
};

#endif
