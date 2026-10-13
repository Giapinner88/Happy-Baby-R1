// Arming handover with the built-in controller: DISARMED, CONFLICT, disarm gate.
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

/**
 * Xử lý trạng thái chờ tiếp quản điều khiển (Disarmed).
 * Trong trạng thái này, ứng dụng KHÔNG publish bất kỳ lệnh nào để tránh ghi đè lệnh của hệ thống built-in.
 */
bool Application::TickDisarmed(const InputCommand& cmd) {
    if (arm_neutral_required_) {
        if (new_state_sample_ && state_fresh_ && input_.gamepad().Neutral())
            arm_neutral_required_ = false;
        return true;
    }
    // Kiểm tra ý định của người dùng: Cần giữ đồng thời nút R1+R2 liên tục trong khoảng thời gian >= `arm_hold_s`.
    // Không cho ARM dựa trên một gói LowState đông cứng: nếu DDS chết đúng lúc
    // người vận hành đang giữ R1+R2 thì bit đó đứng yên và timer vẫn chạy đủ.
    bool hold = remote_state_fresh_ && input_.gamepad().ExactArmCombo();
    if (!hold) arm_hold_timer_ = 0.0f;
    else if (new_state_sample_) arm_hold_timer_ += static_cast<float>(input_sample_dt_s_);
    if (!remote_state_fresh_) arm_intent_latched_ = false;
    if (new_state_sample_ && state_fresh_ && arm_hold_timer_ >= tuning_.arm_hold_s)
        arm_intent_latched_ = true;

    int64_t silence = NowMs() - last_foreign_ms_.load();
    long foreign_seen = foreign_count_.load();

    // Bypass: Nếu quá timeout mà không thấy built-in thì bỏ điều kiện can_hear.
    bool no_builtin_bypass = false;
    if (tuning_.arm_no_builtin_timeout_s > 0.0f && foreign_seen < tuning_.arm_min_foreign_seen) {
        float uptime_s = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - disarmed_start_time_).count() / 1000.0f;
        if (uptime_s >= tuning_.arm_no_builtin_timeout_s) {
            no_builtin_bypass = true;
        }
    }

    bool ok_intent  = !tuning_.arm_require_button || arm_intent_latched_;
    bool can_hear   = no_builtin_bypass
                   || !tuning_.arm_require_seen_builtin
                   || foreign_seen >= tuning_.arm_min_foreign_seen;
    bool ok_silence = (silence >= static_cast<int64_t>(tuning_.arm_silence_ms)) && can_hear;

    if (ok_intent && ok_silence && new_state_sample_ && state_fresh_ &&
        estimator_.valid() && estimator_.sample_valid()) {
        // Bắt đầu quy trình phát hiện xung đột LẠI TỪ ĐẦU ngay khi vừa được tiếp quản điều khiển.
        // Xoá bỏ mọi dữ liệu tích luỹ trước đó vì trong trạng thái `kDisarmed`, việc hệ thống built-in publish là chuyện bình thường.
        // LƯU Ý: Phải reset TRƯỚC khi bật cờ `armed_` để tránh tình trạng `armed_` đã bằng true nhưng state vẫn chứa dữ liệu cũ.
        conflict_flag_.store(false);
        conflict_run_.store(0);
        conflict_first_ms_.store(0);
        conflict_last_ms_.store(0);
        armed_ = true;
        state_ = AppState::kWaitingForState;
        if (no_builtin_bypass) {
            std::cout << "[Application] ARM — bypass Dev Mode (không thấy built-in sau "
                      << tuning_.arm_no_builtin_timeout_s << "s), ý định người OK.\n";
        } else {
            std::cout << "[Application] ARM — built-in đã nhả (im " << silence << "ms, đã thấy "
                      << foreign_seen << " gói built-in), ý định người OK.\n";
        }
        // Không phát voice riêng ở đây: tiếp theo vào kIdle sẽ phát voice_startup.
        return true;
    }

    if (tick_ % 500 == 0) {
        float uptime_s = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - disarmed_start_time_).count() / 1000.0f;
        std::cout << "[kDisarmed] chờ bàn giao | giữ R1+R2: " << arm_hold_timer_ << "/"
                  << tuning_.arm_hold_s << "s (chốt=" << arm_intent_latched_
                  << ") | gói built-in đã thấy: " << foreign_seen
                  << " | im: " << silence << "ms";
        if (tuning_.arm_no_builtin_timeout_s > 0.0f && foreign_seen < tuning_.arm_min_foreign_seen)
            std::cout << " | bypass sau: " << tuning_.arm_no_builtin_timeout_s - uptime_s << "s";
        std::cout << "\n";
    }
    (void)cmd;
    return true;   // KHÔNG publish gì
}

/**
 * Xử lý sự kiện hệ thống built-in giành lại quyền điều khiển (lúc app đang ở trạng thái armed).
 * Ngay lập tức ngừng gửi lệnh và chốt trạng thái ứng dụng về `kConflict`.
 */
void Application::EnterConflict() {
    armed_ = false;
    state_ = AppState::kConflict;
    ClearVoiceCommandGesture();
    arm_gesture_.Cancel();
    conflict_flag_.store(false);
    std::cerr << "\n[FATAL] 🔴 PHÁT HIỆN LOWCMD LẠ khi đang điều khiển — BUILT-IN QUAY LẠI.\n"
                 "        Đã NHẢ QUYỀN (ngừng publish). Bấm E-stop cứng nếu robot loạn.\n";
    if (!tuning_.voice_conflict.empty())
        music_.Say(tuning_.voice_conflict, tuning_.voice_volume, tuning_.voice_speaker_id);
}

bool Application::TryDisarm(const InputCommand& cmd, const LowState_& low) {
    const auto& rs = estimator_.state();
    // Use unfiltered feedback so smoothing cannot hide motion at handover.
    float max_speed = 0.0f;
    for (float dq : rs.dq_raw) max_speed = std::max(max_speed, std::abs(dq));
    bool head_valid = true;
    for (int idl : {spec::kHeadPitchIdl, spec::kHeadYawIdl}) {
        const auto& motor = low.motor_state()[idl];
        head_valid = head_valid && std::isfinite(motor.q()) && std::isfinite(motor.dq());
        max_speed = std::max(max_speed, std::abs(motor.dq()));
    }
    const bool passive = tuning_.arm_gate_enabled && state_ == AppState::kZeroTorque &&
        !cmd.want_emergency_stop && !cmd.want_zero_torque && !teleop_.Active() &&
        !teleop_.FreshEnabledPacket();
    if (!disarm_gate_.Update(passive, state_fresh_ && head_valid && estimator_.valid() && estimator_.sample_valid(), new_state_sample_,
                            input_.gamepad().Neutral(), input_.gamepad().ExactArmCombo(),
                            input_sample_dt_s_, max_speed, rs.gyro_raw.norm(),
                            tuning_.arm_hold_s)) return false;
    EnterDisarmed();
    return true;
}

void Application::EnterDisarmed() {
    // No Send/Damping here: the previous state was already passive, and every
    // subsequent tick must remain silent until a new, neutral-rearmed handover.
    armed_ = false;
    state_ = AppState::kDisarmed;
    active_ = nullptr;
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    sender_.ClearNonTeleopHold();
    teleop_runtime_on_ = false;
    arm_gesture_.RetractSafety();
    ClearVoiceCommandGesture();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    startup_voice_pending_ = false;
    arm_hold_timer_ = 0.0f;
    arm_intent_latched_ = false;
    arm_neutral_required_ = true;
    foreign_count_.store(0);
    last_foreign_ms_.store(NowMs());
    conflict_flag_.store(false);
    conflict_run_.store(0);
    conflict_diag_logged_ = false;
    disarmed_start_time_ = std::chrono::steady_clock::now();
    disarm_gate_.Reset();
    music_.Stop();
    std::cout << "[Application] DISARMED — ngung rt/lowcmd; co the stop-high truoc auto-calib. "
                 "Nha het nut truoc khi arm lai.\n";
    PublishIntegrationStatus(true);
}
