#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "LegFK.hpp"
#include "GaitModeScheduler.hpp"

enum class GaitV3Variant { B, F };
enum class GaitW2SSubstate { Gather = 0, Settle = 1 };

// Deployable FSM v3 shared by gait B and the F/G pair.  The public mode stays
// the existing 3-way one-hot contract; the W2S substate and phase clock are
// internal runtime state.  No MuJoCo contact or terrain truth is consumed.
struct GaitModeV3Config {
    float stop_request_lin = 0.10f;
    float stop_request_ang = 0.10f;
    float move_request_lin = 0.15f;
    float move_request_ang = 0.15f;
    float settle_gyro_norm = 0.25f;
    float settle_joint_velocity_rms = 0.60f;
    float stability_filter_tau_s = 0.20f;
    float gait_period_s = 0.60f;
    float t_settle_s = 1.00f;
    float w2s_timeout_s = 5.00f;
    float stance_width_tol_in_m = 0.04f;
    float stance_width_tol_out_m = 0.08f;
    float stance_dx_tol_m = 0.06f;
    float stance_yaw_tol_rad = 0.30f;
    float stance_exit_factor = 1.50f;
    int gather_max_strides = 3;
    float gather_widen_factor = 2.0f;
    int gather_force_settle_strides = 4;
    float forced_settle_exit_margin = 1.25f;
    float stance_threshold = 0.56f;
    float push_exit_tilt_immediate_rad = 0.13963f;
    float push_exit_tilt_sustained_rad = 0.11345f;
    float push_exit_gyro_immediate = 0.75f;
    float push_exit_gyro_sustained = 0.50f;
    float push_exit_joint_rms_sustained = 1.20f;
    float push_exit_sustain_s = 0.30f;
    float stand_entry_tilt_max_rad = 0.08727f;
    float stand_entry_gyro_max = 0.50f;
};

class GaitModeSchedulerV3 {
public:
    static constexpr std::size_t kNumJoints = 24;

    explicit GaitModeSchedulerV3(GaitV3Variant variant = GaitV3Variant::B)
        : variant_(variant) {}

    void Configure(const GaitModeV3Config& config);
    void Reset();

    void Update(float command_linear_norm, float command_yaw_abs,
                const std::array<float, 3>& gyro,
                const std::array<float, kNumJoints>& joint_velocity,
                const std::array<float, 12>& leg_q,
                const std::array<float, 3>& projected_gravity,
                float dt);

    GaitMode mode() const { return mode_; }
    GaitW2SSubstate substate() const { return substate_; }
    float settle_time_s() const { return settle_time_s_; }
    float stop_elapsed_s() const { return stop_elapsed_s_; }
    float w2s_time_s() const { return w2s_time_s_; }
    float gather_time_s() const { return gather_time_s_; }
    float stand_unstable_time_s() const { return stand_unstable_time_s_; }
    float phase_time_s() const { return phase_time_s_; }
    bool stable() const { return Stable(); }
    bool stance_ok() const { return stance_ok_; }
    bool stance_far() const { return stance_far_; }
    float stance_err_ratio() const { return stance_err_ratio_; }
    const LegFK::Metrics& stance_metrics() const { return stance_metrics_; }
    const std::array<float, 3>& filtered_gyro() const { return filtered_gyro_; }
    const std::array<float, kNumJoints>& filtered_joint_velocity() const {
        return filtered_joint_velocity_;
    }

    std::array<float, 3> OneHot() const;
    std::array<float, 2> PhaseObservation() const;

private:
    void ValidateConfig() const;
    void UpdateFilter(const std::array<float, 3>& gyro,
                      const std::array<float, kNumJoints>& joint_velocity,
                      float dt);
    bool Stable() const;
    void UpdateStance(const std::array<float, 12>& leg_q,
                      const std::array<float, 3>& projected_gravity);
    bool DoubleSupport() const;
    bool ToSettle(bool gather, bool ok, bool double_support, bool forced,
                  bool stable, bool upright, float tilt, float gyro,
                  float joint_rms) const;
    bool BackToGather(bool settle, bool far, float tilt, float gyro,
                      float joint_rms) const;
    static bool Finite(float value);

    GaitV3Variant variant_ = GaitV3Variant::B;
    GaitModeV3Config config_{};
    LegFK fk_{};
    GaitMode mode_ = GaitMode::Walk;
    GaitW2SSubstate substate_ = GaitW2SSubstate::Gather;
    float settle_time_s_ = 0.0f;
    float stop_elapsed_s_ = 0.0f;
    float w2s_time_s_ = 0.0f;
    float gather_time_s_ = 0.0f;
    float stand_unstable_time_s_ = 0.0f;
    float phase_time_s_ = 0.0f;
    float last_dt_s_ = 0.02f;
    float stance_exit_scale_ = 1.0f;
    float stance_err_ratio_ = 0.0f;
    bool stance_ok_ = false;
    bool stance_far_ = false;
    bool filter_primed_ = false;
    bool needs_init_ = true;
    std::array<float, 3> filtered_gyro_{};
    std::array<float, kNumJoints> filtered_joint_velocity_{};
    LegFK::Metrics stance_metrics_{};
};
