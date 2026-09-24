#include "GaitModeSchedulerV3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

constexpr float kPhaseOffsetLeft = 0.0f;
constexpr float kPhaseOffsetRight = 0.5f;

float Max3(float a, float b, float c) { return std::max(a, std::max(b, c)); }

}  // namespace

bool GaitModeSchedulerV3::Finite(float value) {
    return std::isfinite(value);
}

void GaitModeSchedulerV3::ValidateConfig() const {
    const float values[] = {
        config_.stop_request_lin, config_.stop_request_ang,
        config_.move_request_lin, config_.move_request_ang,
        config_.settle_gyro_norm, config_.settle_joint_velocity_rms,
        config_.stability_filter_tau_s, config_.gait_period_s,
        config_.t_settle_s, config_.w2s_timeout_s,
        config_.stance_width_tol_in_m, config_.stance_width_tol_out_m,
        config_.stance_dx_tol_m, config_.stance_yaw_tol_rad,
        config_.stance_exit_factor, config_.gather_widen_factor,
        config_.forced_settle_exit_margin, config_.stance_threshold,
        config_.push_exit_tilt_immediate_rad,
        config_.push_exit_tilt_sustained_rad,
        config_.push_exit_gyro_immediate, config_.push_exit_gyro_sustained,
        config_.push_exit_joint_rms_sustained, config_.push_exit_sustain_s,
        config_.stand_entry_tilt_max_rad, config_.stand_entry_gyro_max,
    };
    for (const float value : values) {
        if (!Finite(value)) throw std::invalid_argument("non-finite gait v3 configuration");
    }
    if (config_.stop_request_lin < 0.0f || config_.stop_request_ang < 0.0f
        || config_.move_request_lin <= config_.stop_request_lin
        || config_.move_request_ang <= config_.stop_request_ang
        || config_.stability_filter_tau_s <= 0.0f
        || config_.gait_period_s <= 0.0f || config_.t_settle_s <= 0.0f
        || config_.w2s_timeout_s <= 0.0f || config_.stance_width_tol_in_m <= 0.0f
        || config_.stance_width_tol_out_m <= 0.0f || config_.stance_dx_tol_m <= 0.0f
        || config_.stance_yaw_tol_rad <= 0.0f || config_.stance_exit_factor <= 0.0f
        || config_.gather_max_strides <= 0 || config_.gather_widen_factor < 1.0f
        || config_.gather_force_settle_strides <= 0
        || config_.forced_settle_exit_margin < 1.0f
        || config_.stance_threshold <= 0.0f || config_.stance_threshold >= 1.0f
        || config_.push_exit_sustain_s <= 0.0f) {
        throw std::invalid_argument("invalid gait v3 configuration");
    }
}

void GaitModeSchedulerV3::Configure(const GaitModeV3Config& config) {
    config_ = config;
    ValidateConfig();
}

void GaitModeSchedulerV3::Reset() {
    mode_ = GaitMode::Walk;
    substate_ = GaitW2SSubstate::Gather;
    settle_time_s_ = 0.0f;
    stop_elapsed_s_ = 0.0f;
    w2s_time_s_ = 0.0f;
    gather_time_s_ = 0.0f;
    stand_unstable_time_s_ = 0.0f;
    phase_time_s_ = 0.0f;
    last_dt_s_ = 0.02f;
    stance_exit_scale_ = 1.0f;
    stance_err_ratio_ = 0.0f;
    stance_ok_ = false;
    stance_far_ = false;
    filter_primed_ = false;
    needs_init_ = true;
    filtered_gyro_.fill(0.0f);
    filtered_joint_velocity_.fill(0.0f);
    stance_metrics_ = {};
}

void GaitModeSchedulerV3::UpdateFilter(
    const std::array<float, 3>& gyro,
    const std::array<float, kNumJoints>& joint_velocity,
    float dt) {
    if (!filter_primed_) {
        filtered_gyro_ = gyro;
        filtered_joint_velocity_ = joint_velocity;
        filter_primed_ = true;
        return;
    }
    const float alpha = std::min(dt / std::max(config_.stability_filter_tau_s, 1.0e-6f), 1.0f);
    for (std::size_t i = 0; i < filtered_gyro_.size(); ++i)
        filtered_gyro_[i] += alpha * (gyro[i] - filtered_gyro_[i]);
    for (std::size_t i = 0; i < filtered_joint_velocity_.size(); ++i)
        filtered_joint_velocity_[i] += alpha * (joint_velocity[i] - filtered_joint_velocity_[i]);
}

bool GaitModeSchedulerV3::Stable() const {
    float gyro_sq = 0.0f, joint_sq = 0.0f;
    for (float value : filtered_gyro_) gyro_sq += value * value;
    for (float value : filtered_joint_velocity_) joint_sq += value * value;
    return std::sqrt(gyro_sq) < config_.settle_gyro_norm
        && std::sqrt(joint_sq / static_cast<float>(kNumJoints))
               < config_.settle_joint_velocity_rms;
}

void GaitModeSchedulerV3::UpdateStance(
    const std::array<float, 12>& leg_q,
    const std::array<float, 3>& projected_gravity) {
    stance_metrics_ = fk_.Compute(leg_q, projected_gravity);
    const float width_ratio = stance_metrics_.width_err < 0.0f
        ? -stance_metrics_.width_err / config_.stance_width_tol_in_m
        : stance_metrics_.width_err / config_.stance_width_tol_out_m;
    const float dx_ratio = std::abs(stance_metrics_.dx_err) / config_.stance_dx_tol_m;
    const float yaw_ratio = std::abs(stance_metrics_.dyaw_err) / config_.stance_yaw_tol_rad;
    stance_err_ratio_ = Max3(width_ratio, dx_ratio, yaw_ratio);
    const float widen = gather_time_s_ >=
            static_cast<float>(config_.gather_max_strides) * config_.gait_period_s - 1.0e-6f
        ? config_.gather_widen_factor : 1.0f;
    stance_ok_ = stance_err_ratio_ <= widen;
    stance_far_ = stance_err_ratio_ > config_.stance_exit_factor * stance_exit_scale_;
}

bool GaitModeSchedulerV3::DoubleSupport() const {
    const float ratio = phase_time_s_ / config_.gait_period_s;
    const float left = ratio - std::floor(ratio + kPhaseOffsetLeft);
    const float right = ratio + kPhaseOffsetRight
        - std::floor(ratio + kPhaseOffsetRight);
    return left < config_.stance_threshold && right < config_.stance_threshold;
}

bool GaitModeSchedulerV3::ToSettle(bool gather, bool ok, bool double_support,
                                   bool forced, bool stable, bool upright,
                                   float tilt, float gyro, float joint_rms) const {
    if (!gather) return false;
    if (variant_ == GaitV3Variant::B)
        return (ok && double_support) || forced;

    (void)upright;
    const bool quiet = stable && tilt <= config_.stand_entry_tilt_max_rad
        && gyro <= config_.stand_entry_gyro_max;
    const bool min_gather_done = gather_time_s_ >=
        config_.gait_period_s - 0.5f * last_dt_s_;
    const bool severe = tilt > config_.push_exit_tilt_immediate_rad
        || gyro > config_.push_exit_gyro_immediate
        || joint_rms > config_.push_exit_joint_rms_sustained;
    const bool ready = ok && double_support && (quiet || min_gather_done);
    return ready || (forced && !severe);
}

bool GaitModeSchedulerV3::BackToGather(bool settle, bool far, float tilt,
                                        float gyro, float joint_rms) const {
    if (!settle) return false;
    if (variant_ == GaitV3Variant::B) return far;
    const bool unstable = tilt > config_.push_exit_tilt_sustained_rad
        || gyro > config_.push_exit_gyro_sustained
        || joint_rms > config_.push_exit_joint_rms_sustained;
    return far || unstable;
}

void GaitModeSchedulerV3::Update(
    float command_linear_norm, float command_yaw_abs,
    const std::array<float, 3>& gyro,
    const std::array<float, kNumJoints>& joint_velocity,
    const std::array<float, 12>& leg_q,
    const std::array<float, 3>& projected_gravity,
    float dt) {
    if (!Finite(command_linear_norm) || !Finite(command_yaw_abs)
        || command_linear_norm < 0.0f || command_yaw_abs < 0.0f
        || !Finite(dt) || dt < 0.0f)
        throw std::invalid_argument("invalid gait v3 update input");
    last_dt_s_ = dt;
    for (float value : gyro) if (!Finite(value)) throw std::invalid_argument("non-finite gait gyro");
    for (float value : joint_velocity) if (!Finite(value)) throw std::invalid_argument("non-finite gait joint velocity");
    for (float value : leg_q) if (!Finite(value)) throw std::invalid_argument("non-finite gait leg position");
    for (float value : projected_gravity) if (!Finite(value)) throw std::invalid_argument("non-finite gait gravity");

    UpdateFilter(gyro, joint_velocity, dt);
    const bool stop = command_linear_norm < config_.stop_request_lin
        && command_yaw_abs < config_.stop_request_ang;
    const bool move = command_linear_norm > config_.move_request_lin
        || command_yaw_abs > config_.move_request_ang;
    stop_elapsed_s_ = stop ? stop_elapsed_s_ + dt : 0.0f;
    const GaitMode previous = mode_;
    UpdateStance(leg_q, projected_gravity);
    const bool double_support = DoubleSupport();
    const float gravity_norm_xy = std::hypot(projected_gravity[0], projected_gravity[1]);
    const float tilt = std::atan2(gravity_norm_xy, -projected_gravity[2]);
    float gyro_sq = 0.0f, joint_sq = 0.0f;
    for (float value : filtered_gyro_) gyro_sq += value * value;
    for (float value : filtered_joint_velocity_) joint_sq += value * value;
    const float gyro_norm = std::sqrt(gyro_sq);
    const float joint_rms = std::sqrt(joint_sq / static_cast<float>(kNumJoints));
    const bool upright = tilt <= config_.stand_entry_tilt_max_rad
        && gyro_norm <= config_.stand_entry_gyro_max;

    const GaitW2SSubstate entry_substate =
        (variant_ == GaitV3Variant::B && stance_ok_ && double_support)
            ? GaitW2SSubstate::Settle : GaitW2SSubstate::Gather;

    if (needs_init_) {
        if (move) mode_ = GaitMode::Walk;
        else {
            mode_ = (stop && Stable() && stance_ok_) ? GaitMode::Stand : GaitMode::W2S;
            substate_ = entry_substate;
        }
        settle_time_s_ = 0.0f;
        w2s_time_s_ = 0.0f;
        gather_time_s_ = 0.0f;
        stand_unstable_time_s_ = 0.0f;
        stance_exit_scale_ = 1.0f;
        needs_init_ = false;
    }

    const bool is_stand = mode_ == GaitMode::Stand;
    const bool is_walk = mode_ == GaitMode::Walk;
    const bool is_w2s = mode_ == GaitMode::W2S;
    const bool to_walk = move && (is_stand || is_w2s);
    const bool to_w2s = stop && is_walk;
    const bool immediate = tilt > config_.push_exit_tilt_immediate_rad
        || gyro_norm > config_.push_exit_gyro_immediate || stance_far_;
    const bool sustained = tilt > config_.push_exit_tilt_sustained_rad
        || gyro_norm > config_.push_exit_gyro_sustained
        || joint_rms > config_.push_exit_joint_rms_sustained;
    stand_unstable_time_s_ = is_stand && sustained
        ? stand_unstable_time_s_ + dt : 0.0f;
    const bool push_exit = is_stand && !to_walk &&
        (immediate || stand_unstable_time_s_ >= config_.push_exit_sustain_s - 0.5f * dt);

    const bool staying = is_w2s && !to_walk;
    const bool gather = staying && substate_ == GaitW2SSubstate::Gather;
    const bool settle = staying && substate_ == GaitW2SSubstate::Settle;
    gather_time_s_ = gather ? gather_time_s_ + dt : gather_time_s_;
    const bool forced = gather && double_support && gather_time_s_ >=
        static_cast<float>(config_.gather_force_settle_strides) * config_.gait_period_s
            - 0.5f * dt;
    const bool to_settle = ToSettle(gather, stance_ok_, double_support, forced,
                                    Stable(), upright, tilt, gyro_norm, joint_rms);
    const bool back_to_gather = BackToGather(settle, stance_far_, tilt, gyro_norm, joint_rms);
    const bool settling = settle && !back_to_gather && stop && Stable() && upright;
    settle_time_s_ = settling ? settle_time_s_ + dt : 0.0f;
    const bool to_stand = settling && settle_time_s_ >= config_.t_settle_s - 0.5f * dt;

    if (to_walk) mode_ = GaitMode::Walk;
    else if (to_w2s || push_exit) mode_ = GaitMode::W2S;
    if (to_stand) mode_ = GaitMode::Stand;

    if (to_settle) substate_ = GaitW2SSubstate::Settle;
    if (back_to_gather || push_exit) substate_ = GaitW2SSubstate::Gather;
    if (to_w2s) substate_ = entry_substate;
    if (to_walk) substate_ = GaitW2SSubstate::Gather;

    if (forced && to_settle) {
        stance_exit_scale_ = std::max(
            1.0f, stance_err_ratio_ / config_.stance_exit_factor
                * config_.forced_settle_exit_margin);
    }
    if ((stance_ok_ && !(forced && to_settle)
         && gather_time_s_ < config_.gather_max_strides * config_.gait_period_s)
        || to_walk) {
        stance_exit_scale_ = 1.0f;
    }

    if (back_to_gather || push_exit || to_w2s || to_walk) gather_time_s_ = 0.0f;
    if (to_walk || to_stand || back_to_gather || push_exit || to_w2s)
        settle_time_s_ = 0.0f;

    w2s_time_s_ = mode_ == GaitMode::W2S ? w2s_time_s_ + dt : 0.0f;
    if (w2s_time_s_ >= config_.w2s_timeout_s) w2s_time_s_ = 0.0f;

    const bool left_stand = previous == GaitMode::Stand && mode_ != GaitMode::Stand;
    const float advanced = std::fmod(phase_time_s_ + dt, config_.gait_period_s);
    const bool clock_running = mode_ == GaitMode::Walk
        || (mode_ == GaitMode::W2S && substate_ == GaitW2SSubstate::Gather);
    if (left_stand) phase_time_s_ = 0.0f;
    else if (clock_running) phase_time_s_ = advanced;
}

std::array<float, 3> GaitModeSchedulerV3::OneHot() const {
    std::array<float, 3> result{0.0f, 0.0f, 0.0f};
    result[static_cast<std::size_t>(mode_)] = 1.0f;
    return result;
}

std::array<float, 2> GaitModeSchedulerV3::PhaseObservation() const {
    if (mode_ == GaitMode::Stand
        || (mode_ == GaitMode::W2S && substate_ == GaitW2SSubstate::Settle))
        return {0.0f, 0.0f};
    const float angle = 2.0f * static_cast<float>(M_PI)
        * phase_time_s_ / config_.gait_period_s;
    return {std::sin(angle), std::cos(angle)};
}
