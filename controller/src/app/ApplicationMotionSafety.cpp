// Runtime adapter for the same motion guard qualified in simulate_cpp.
#include "Application.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
bool CheckRawSample(motion_safety::Guard& guard,
                    const unitree_hg::msg::dds_::LowState_& low, float& max_dq,
                    std::chrono::steady_clock::time_point progress_time) {
    const auto& imu = low.imu_state();
    const Eigen::Quaternionf root(imu.quaternion()[0], imu.quaternion()[1],
                                   imu.quaternion()[2], imu.quaternion()[3]);
    const Eigen::Vector3f gyro(imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]);
    bool finite = true;
    max_dq = 0;
    auto joint = [&](int idl) {
        const auto& m = low.motor_state()[idl];
        finite = finite && std::isfinite(m.q()) && std::isfinite(m.dq());
        max_dq = std::max(max_dq, std::abs(m.dq()));
    };
    for (int i = 0; i < spec::kNumJoints; ++i) joint(spec::MotorIdl(i));
    joint(spec::kHeadPitchIdl);
    joint(spec::kHeadYawIdl);
    for (float a : imu.accelerometer()) finite = finite && std::isfinite(a);
    const double wall = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const double progress = std::chrono::duration<double>(progress_time.time_since_epoch()).count();
    return !guard.Sensor(low.tick(), wall, root, gyro, finite, max_dq, progress);
}
}

bool Application::CheckMotionSensor(const unitree_hg::msg::dds_::LowState_& low) {
    const bool powered = state_ != AppState::kIdle && state_ != AppState::kZeroTorque &&
        state_ != AppState::kWaitingForState && state_ != AppState::kDisarmed &&
        state_ != AppState::kConflict;
    if (!powered) { motion_guard_monitoring_ = false; return true; }
    if (!motion_guard_monitoring_) {
        if (motion_guard_.Latched()) { StopForMotionFault(motion_guard_.Reason()); return false; }
        motion_guard_ = motion_safety::Guard(tuning_.MotionSafetyConfig());
        motion_guard_monitoring_ = true;
    }
    float max_dq = 0;
    if (!CheckRawSample(motion_guard_, low, max_dq, sampled_state_progress_time_)) {
        StopForMotionFault(motion_guard_.Reason());
        return false;
    }
    return true;
}

bool Application::TryRecoverMotion(const unitree_hg::msg::dds_::LowState_& low,
                                   const InputCommand& cmd, bool state_ok) {
    float max_dq = 0;
    if (!state_ok || !estimator_.sample_valid() || !state_fresh_ || !remote_state_fresh_ ||
        !CheckRawSample(motion_recovery_guard_, low, max_dq, sampled_state_progress_time_)) {
        motion_recovery_guard_ = motion_safety::Guard(tuning_.MotionSafetyConfig());
        motion_recovery_sample_seen_ = motion_recovery_released_ = false;
        return false;
    }
    // One newly received packet (or repeated frozen packets) is not proof that
    // feedback has recovered. Observe physical tick progression as well.
    if (!new_state_sample_ || motion_recovery_guard_.SampleDt() <= 0) return false;
    const bool progressing = motion_recovery_sample_seen_;
    motion_recovery_sample_seen_ = true;
    if (!progressing) return false;
    const bool request = cmd.want_stand_lock || cmd.want_get_up_down ||
        cmd.want_locomotion || cmd.want_teleop_locomotion || cmd.want_mimic_key ||
        cmd.want_zero_torque || cmd.want_safe_shutdown;
    const bool remote_neutral = input_.gamepad().Neutral() ||
        (!input_.gamepad().packet_valid() && input_.keyboard().available());
    const bool neutral = remote_neutral && !request && !cmd.want_emergency_stop &&
        !cmd.want_reset && !cmd.want_teleop_toggle && !cmd.want_gesture &&
        std::abs(cmd.vx) <= .05f && std::abs(cmd.vy) <= .05f && std::abs(cmd.yaw) <= .05f;
    if (neutral) { motion_recovery_released_ = true; return false; }
    if (!motion_recovery_released_ || !request || cmd.want_emergency_stop) return false;
    // Consume rejected requests too; never queue a command to activate later
    // merely because a moving robot subsequently becomes quiet/upright.
    motion_recovery_released_ = false;
    const auto& rs = estimator_.state();
    const bool quiet = max_dq <= tuning_.mimic_entry_dq_max &&
                       rs.gyro_raw.norm() <= tuning_.mimic_entry_gyro_max;
    const bool lying = RobotLying(rs);
    const bool getup_request = lying && (cmd.want_stand_lock || cmd.want_get_up_down) &&
                               getup_motion_;
    const bool upright = motion_safety::TiltErrorDeg(rs.quat, Eigen::Quaternionf::Identity())
                         <= tuning_.fall_tilt_deg;
    if (!quiet || (!cmd.want_zero_torque && !getup_request && !upright)) return false;
    motion_guard_ = motion_safety::Guard(tuning_.MotionSafetyConfig());
    motion_guard_monitoring_ = false;
    std::cout << "[MotionSafety] New operator command accepted from IDLE; normal controls restored.\n";
    return true;
}

bool Application::CheckMotionPose() {
    if (!tuning_.state_motion_guard) return true;
    if (motion_guard_.Latched()) { StopForMotionFault(motion_guard_.Reason()); return false; }
    // Do not invent an upright target for intentionally seated/lying/getup
    // trajectories. Raw data, joint speed and freshness remain guarded there.
    motion_safety::Mode mode = motion_safety::Mode::Posture;
    const auto& rs = estimator_.state();
    bool dance_upright = false;
    Eigen::Quaternionf actual = rs.quat, reference = Eigen::Quaternionf::Identity();
    if (state_ == AppState::kMimic) {
        auto it = mimics_.find(pending_dance_key_);
        if (it == mimics_.end() || !it->second || active_ != it->second.get()) {
            StopForMotionFault("invalid_reference"); return false;
        }
        mode = motion_safety::Mode::Dance;
        actual = it->second->SafetyActualTorsoWorld(rs);
        reference = it->second->TelemetryReferenceTorsoWorld();
        dance_upright = it->second->SafetyUprightExpected();
    } else if (state_ == AppState::kLocomotion || state_ == AppState::kReturningToDefault ||
               state_ == AppState::kStandLock || state_ == AppState::kStandUp) {
        mode = motion_safety::Mode::Upright;
    } else if (state_ != AppState::kGetUp && state_ != AppState::kLieDown &&
               state_ != AppState::kSafeShutdown) return true;
    if (motion_guard_.Pose(mode, rs.quat, rs.gyro_raw, actual, reference, dance_upright)) {
        StopForMotionFault(motion_guard_.Reason()); return false;
    }
    return true;
}

void Application::StopForMotionFault(const char* reason) {
    const bool first = !motion_guard_.Latched();
    motion_guard_.Trip(reason);
    // Sensor/Pose may have latched before calling us. The state is the reliable
    // one-shot log gate, while Damping must be repeated even when already IDLE.
    if (first || state_ != AppState::kIdle) {
        motion_recovery_guard_ = motion_safety::Guard(tuning_.MotionSafetyConfig());
        motion_recovery_sample_seen_ = motion_recovery_released_ = false;
        std::cerr << "[MotionSafety] STOP reason=" << motion_guard_.Reason()
                  << " state=" << StateName() << " dance=" << pending_dance_key_
                  << " error_deg=" << motion_guard_.ErrorDeg()
                  << " bad_s=" << motion_guard_.BadSeconds()
                  << " progress_age_ms=" << motion_guard_.ProgressAgeSeconds()*1000
                  << " observation_gap_ms=" << motion_guard_.ObservationGapSeconds()*1000
                  << " tick_advance=" << motion_guard_.TickAdvance()
                  << "; IDLE/damping, release controls then issue a new command\n";
    }
    EnterIdle(std::string("Motion safety: ") + motion_guard_.Reason());
    active_ = nullptr;
    teleop_runtime_on_ = false;
    teleop_.Update(spec::kLoopDt, false);
    arm_gesture_.Cancel();
    sender_.Damping(estimator_.state().mode_machine);
}
