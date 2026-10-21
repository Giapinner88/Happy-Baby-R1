// Per-dance mimic telemetry sessions.
// Application is split across src/app/Application*.cpp; see Application.hpp.
#include "Application.hpp"
#include "ApplicationInternal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;
using namespace app_internal;

void Application::StartMimicTelemetry(int dance_key) {
    if (!tuning_.mimic_telemetry_enabled) return;

    const int idx = dance_key - 1;
    const std::string label = (idx >= 0 && idx < Tuning::kMaxDances)
                                ? tuning_.dance_folder[idx]
                                : "unknown";
    std::string dir = tuning_.mimic_telemetry_dir;
    if (dir.empty()) dir = "logs/telemetry";
    if (dir.front() != '/') dir = proj_dir_ + "/" + dir;

    const int hz = std::clamp(tuning_.mimic_telemetry_hz, 1, 500);
    telemetry_rate_accum_ = 0;
    telemetry_seen_mimic_ = false;
    telemetry_post_ticks_ = -1;
    telemetry_.StartSession(dir, dance_key, label, hz);
}

void Application::CaptureMimicTelemetry(const LowState_& low) {
    if (!telemetry_.Active()) return;

    if (state_ == AppState::kMimic) {
        telemetry_seen_mimic_ = true;
        telemetry_post_ticks_ = -1;
    } else if (telemetry_seen_mimic_) {
        if (telemetry_post_ticks_ < 0) {
            telemetry_post_ticks_ = std::max(1, static_cast<int>(
                std::max(0.0f, tuning_.mimic_telemetry_post_s) / spec::kLoopDt));
        } else if (telemetry_post_ticks_ == 0) {
            telemetry_.StopAsync();
            telemetry_seen_mimic_ = false;
            return;
        }
        --telemetry_post_ticks_;
    }

    // Tích lũy tần số gửi telemetry (Bresenham-style).
    const int hz = std::clamp(tuning_.mimic_telemetry_hz, 1, 500);
    telemetry_rate_accum_ += hz;
    if (telemetry_rate_accum_ < 500) return;
    telemetry_rate_accum_ -= 500;

    MimicTelemetrySample s;
    s.monotonic_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    s.elapsed_us = s.monotonic_us - telemetry_.start_us();
    s.loop_tick = tick_;
    s.lowstate_tick = low.tick();
    s.app_state = static_cast<int>(state_);
    s.dance_key = pending_dance_key_;

    const RobotState& rs = estimator_.state();
    s.quat = {rs.quat.w(), rs.quat.x(), rs.quat.y(), rs.quat.z()};
    s.gravity = {rs.projected_gravity.x(), rs.projected_gravity.y(),
                 rs.projected_gravity.z()};
    s.gyro = {rs.gyro.x(), rs.gyro.y(), rs.gyro.z()};
    s.gyro_raw = {rs.gyro_raw.x(), rs.gyro_raw.y(), rs.gyro_raw.z()};
    s.accel_raw = {rs.accel_raw.x(), rs.accel_raw.y(), rs.accel_raw.z()};
    s.q = rs.q;
    s.dq = rs.dq;
    s.dq_raw = rs.dq_raw;

    auto mit = mimics_.find(pending_dance_key_);
    if (mit != mimics_.end()) {
        const auto& mimic = *mit->second;
        s.clip_phase = mimic.TelemetryPhase();
        s.clip_frame = mimic.TelemetryFrame();
        s.ref_q = mimic.TelemetryReferenceQ();
        const Eigen::Quaternionf rq = mimic.TelemetryReferenceTorsoWorld();
        s.ref_quat = {rq.w(), rq.x(), rq.y(), rq.z()};
        if (state_ == AppState::kMimic) s.mimic_stage = mimic.TelemetryStage();
    }

    if (active_) {
        s.action = active_->last_action();
        if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
            s.kp = active_->kp();
            s.kd = active_->kd();
        }
    }
    if (state_ == AppState::kReturningToDefault) {
        s.kp = return_gains_kp_;   // cổ chân = return_kp_ankle, không phải stand_kp_leg
        s.kd = stand_gains_kd_;
    } else if (state_ == AppState::kStandUp || state_ == AppState::kStandLock) {
        s.kp = stand_gains_kp_;
        s.kd = stand_gains_kd_;
    } else if (state_ == AppState::kSafeShutdown) {
        s.kp = sit_gains_kp_;
        s.kd = sit_gains_kd_;
    } else if (state_ == AppState::kGetUp || state_ == AppState::kLieDown) {
        s.kp = getup_gains_kp_;
        s.kd = getup_gains_kd_;
    }

    for (int i = 0; i < spec::kNumJoints; ++i) {
        const int idl = spec::MotorIdl(i);
        s.q_des[i] = sender_.last_cmd_q_policy(i);
        s.tau_est[i] = low.motor_state()[idl].tau_est();
    }
    telemetry_.Enqueue(s);
}
