// Policy states (LOCOMOTION, MIMIC): upper-body overlay, inference, dance handover.
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

namespace {

float JointVelocityRms(const RobotState& state) {
    float sum_sq = 0.0f;
    for (const float value : state.dq) sum_sq += value * value;
    return std::sqrt(sum_sq / static_cast<float>(spec::kNumJoints));
}

/**
 * Tính góc Yaw (quay quanh trục Z) từ quaternion của phần hông (pelvis - wxyz).
 * Do robot di chuyển trong trạng thái gần thẳng đứng, giá trị này có thể dùng để 
 * xấp xỉ hướng mặt của robot so với mặt sàn.
 */
float YawFromQuat(const Eigen::Quaternionf& q) {
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();
    return std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

/**
 * Giới hạn (bọc) giá trị góc về khoảng (-pi, pi].
 */
float WrapPi(float a) {
    constexpr float kPi = static_cast<float>(M_PI);
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

} // namespace

void Application::ApplyHeadingHold(const RobotState& rs) {
    const Tuning& t = tuning_;
    // Tắt hẳn / không ở locomotion -> không đụng lệnh, reset cờ.
    if (!t.heading_hold_enabled || state_ != AppState::kLocomotion) {
        heading_hold_active_ = false;
        return;
    }
    // Chỉ giữ hướng khi ĐANG DI CHUYỂN và người lái KHÔNG bẻ yaw.
    const float move = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_);
    const bool steering = std::abs(cmd_yaw_) > 0.05f;
    if (move < t.heading_hold_move_min || steering) {
        heading_hold_active_ = false;
        return;
    }
    const float yaw_now = YawFromQuat(rs.quat);
    // Chốt hướng khi mới vào HOẶC thân còn xoay nhanh (đợi hết quán tính sau cú bẻ lái).
    // Tick chốt: chưa sửa lệnh (để hết xoay rồi mới bắt đầu kéo giữ).
    if (!heading_hold_active_ || std::abs(rs.gyro.z()) > t.heading_hold_relatch_gyro) {
        heading_ref_ = yaw_now;
        heading_hold_active_ = true;
        return;
    }
    const float err = WrapPi(heading_ref_ - yaw_now);
    // Lệch quá lớn (IMU nhảy bậc / bị người xoay tay) -> chốt lại, không kéo giật.
    if (std::abs(err) > 1.0f) {
        heading_ref_ = yaw_now;
        return;
    }
    // Lệnh sửa: kéo mặt về hướng đã chốt. DẤU: err>0 (lệch một phía) -> cmd_yaw kéo
    // ngược lại. Nếu bench-test thấy đẩy CÙNG chiều -> đảo thành (yaw_now - heading_ref_).
    cmd_yaw_ = std::clamp(t.heading_hold_kp * err,
                          -t.heading_hold_max_yaw, t.heading_hold_max_yaw);
}

PolicyController* Application::LocomotionFor(LocoMode mode) const {
    if (mode == LocoMode::kTeleop && teleop_ctrl_) return teleop_ctrl_.get();
    return walk_ctrl_.get();
}

const char* Application::LocoModeName(LocoMode mode) {
    return mode == LocoMode::kTeleop ? "DI BO + TELEOP" : "DI BO";
}

void Application::EnterLocomotion(LocoMode mode) {
    loco_swap_pending_ = false;
    loco_mode_ = mode;
    locomotion_ = LocomotionFor(mode);
    std::cout << "[Application] Activating LOCOMOTION (" << LocoModeName(mode) << ", policy '"
              << (mode == LocoMode::kTeleop ? teleop_policy_label_ : walk_policy_label_)
              << "').\n";
    ActivatePolicy(locomotion_);
    state_ = AppState::kLocomotion;
    const std::string& voice = mode == LocoMode::kTeleop && !tuning_.voice_locomotion_teleop.empty()
        ? tuning_.voice_locomotion_teleop : tuning_.voice_locomotion;
    music_.Say(voice, tuning_.voice_volume, tuning_.voice_speaker_id);
}

void Application::RequestLocoMode(LocoMode mode) {
    if (loco_swap_pending_) {
        if (mode == loco_mode_) CancelLocoModeSwap("nguoi lai giu che do hien tai");
        return;
    }
    if (mode == loco_mode_) {
        std::cout << "[Application] Da o che do " << LocoModeName(mode) << ".\n";
        return;
    }
    if (announcing_ || pending_action_ != PendingAction::kNone) {
        std::cout << "[Application] Doi che do bi chan: dang cho dance/tu the.\n";
        return;
    }
    loco_swap_pending_ = true;
    loco_swap_target_ = mode;
    loco_swap_timer_s_ = 0.0f;
    loco_swap_still_s_ = 0.0f;
    input_.ZeroVelocity();
    // Rời teleop: nhả quyền ngay để tay thu về theo đường handover H4 sẵn có.
    if (loco_mode_ == LocoMode::kTeleop && teleop_runtime_on_)
        SetTeleopArmed(false, "doi sang di bo thuong");
    std::cout << "[Application] Doi che do " << LocoModeName(loco_mode_) << " -> "
              << LocoModeName(mode) << ": dung yen, cho tay thu va gait STAND on dinh...\n";
}

void Application::CancelLocoModeSwap(const char* why) {
    if (!loco_swap_pending_) return;
    loco_swap_pending_ = false;
    loco_swap_still_s_ = 0.0f;
    std::cout << "[Application] Huy doi che do (" << why << "), giu "
              << LocoModeName(loco_mode_) << ".\n";
}

// Đổi nóng chỉ khi: thân trên đã nhả hết (gesture/teleop/history H4), và — nếu hai
// chế độ dùng hai policy khác nhau — robot đứng yên ổn định liên tục
// policy_swap_dwell_s (gait STAND + stable với gait-H4). Policy mới được Reset
// (history sạch) và blend từ target vừa gửi như mọi lần ActivatePolicy.
void Application::AdvanceLocoModeSwap(const RobotState& rs) {
    loco_swap_timer_s_ += spec::kLoopDt;
    if (loco_swap_timer_s_ > tuning_.policy_swap_timeout_s) {
        CancelLocoModeSwap("qua policy_swap_timeout_s, robot chua dung yen");
        return;
    }
    if (loco_mode_ == LocoMode::kTeleop && teleop_runtime_on_)
        SetTeleopArmed(false, "doi sang di bo thuong");

    PolicyController* next = LocomotionFor(loco_swap_target_);
    const bool same_ctrl = next == locomotion_;
    const bool upper_idle = arm_gesture_.Idle() && gesture_history_release_s_ <= 0.0f &&
                            teleop_.Weight() <= 0.0f && h4_history_release_s_ <= 0.0f &&
                            h4_head_release_s_ <= 0.0f;
    bool stable = rs.gyro.norm() <= tuning_.gait_stability_gyro_norm &&
                  JointVelocityRms(rs) <= tuning_.gait_stability_joint_velocity_rms;
    if (const auto* gait = dynamic_cast<const FlatPlusGaitController*>(locomotion_)) {
        stable = stable && gait->gait().mode() == GaitMode::Stand && gait->gait().stable();
    }
    const bool ready = upper_idle && (same_ctrl || stable);
    loco_swap_still_s_ = ready ? loco_swap_still_s_ + spec::kLoopDt : 0.0f;
    const float dwell = same_ctrl ? 0.0f : tuning_.policy_swap_dwell_s;
    if (!ready || loco_swap_still_s_ < dwell) return;

    const LocoMode from = loco_mode_;
    loco_swap_pending_ = false;
    loco_mode_ = loco_swap_target_;
    if (!same_ctrl) {
        locomotion_ = next;
        ActivatePolicy(next);
    }
    std::cout << "[Application] Da doi che do " << LocoModeName(from) << " -> "
              << LocoModeName(loco_mode_) << " sau " << loco_swap_timer_s_ << "s"
              << (same_ctrl ? " (chung policy)" : " (policy moi, history reset)") << ".\n";
    const std::string& voice = loco_mode_ == LocoMode::kTeleop && !tuning_.voice_locomotion_teleop.empty()
        ? tuning_.voice_locomotion_teleop : tuning_.voice_locomotion;
    music_.Say(voice, tuning_.voice_volume, tuning_.voice_speaker_id);
}

void Application::RunPolicy(const InputCommand& cmd) {
    const RobotState& rs = estimator_.state();

    int worst_j = -1;
    float max_dq = MaxJointSpeed(rs, worst_j);
    std::vector<std::string> reasons;
    if (fall_detector_.Check(rs.projected_gravity, rs.gyro, max_dq, worst_j, reasons)) {
        std::cout << "\n[Application] Fall detected!\n";
        for (const auto& r : reasons) std::cout << "  - " << r << "\n";
        EnterIdle("Fall detected - triggering damping");
        return;
    }

    const bool holding = announcing_ || pending_action_ != PendingAction::kNone ||
                         loco_swap_pending_;
    const PolicyOverlayCaps caps = PolicyCaps();
    const float move_intent = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy +
                                        cmd.yaw * cmd.yaw);
    const bool history_gesture_move_requested = caps.history_gesture &&
        move_intent > tuning_.gesture_history_move_threshold;
    const bool history_gesture_ready =
        UpdateHistoryGestureDwell(rs, caps.history_gesture, history_gesture_move_requested);
    UpdateBaseCommand(cmd, caps, holding);

    // Giữ hướng khi đi thẳng: có thể ghi đè cmd_yaw_ bằng lệnh sửa (no-op nếu tắt).
    ApplyHeadingHold(rs);

    const UpperBodyCommand ub = UpdateUpperBody(
        cmd, rs, caps, holding, move_intent,
        history_gesture_ready, history_gesture_move_requested);

    if (tick_ % spec::kPolicyDecimation == 0 && !StepPolicy(rs, caps, ub)) return;

    // Ramp gain khi bàn giao vào MIMIC: ngay lúc soft-start giữ CHÂN cứng như tư thế
    // đứng (hip/knee stand_kp=200, ankle return_kp_ankle=120) rồi nhả dần về gain policy
    // (ankle 40, hip/knee 100) theo tiến độ warmup. Tránh cổ chân quá mềm đúng lúc
    // reference vừa kéo ankle vừa dạng chân ra => hết bập bênh ở bài đứng dạng/nhón (xexe).
    // Xuất phát từ return_gains_kp_ (KHÔNG phải stand_gains_kp_) để nối liền với
    // kReturningToDefault ngay trước đó -> cổ chân không có bậc nhảy ở ranh giới 2 state.
    // Tra cứu MỘT lần bằng find(): map::operator[] sẽ chèn unique_ptr rỗng nếu key
    // biến mất rồi deref null ngay trong vòng 500Hz.
    MimicController* mimic = nullptr;
    if (state_ == AppState::kMimic) {
        auto mit = mimics_.find(pending_dance_key_);
        if (mit == mimics_.end() || !mit->second) {
            EnterIdle("Dance " + std::to_string(pending_dance_key_) +
                      " biến mất giữa chừng");
            return;
        }
        mimic = mit->second.get();
    }
    std::array<float, spec::kNumJoints> kp_eff = active_->kp();
    std::array<float, spec::kNumJoints> kd_eff = active_->kd();
    ShapeMimicTarget(mimic, kp_eff, kd_eff);

    UpdateGestureHandover(rs, caps.gesture, ub.use_teleop);

    std::array<float, spec::kNumJoints> send_q = ai_target_q_;
    // H4 teleop and gestures both mask each 83-D history frame before packing.
    if (ub.active && (caps.gesture || caps.h4)) {
        if (ub.use_teleop) teleop_.BlendInto(send_q);
        else               arm_gesture_.BlendInto(send_q);
    }
    sender_.Send(send_q, kp_eff, kd_eff,
                 tuning_.policy_rate_limit, rs.mode_machine, ub.head);

    UpdateMimicLifecycle(mimic, rs);
}

Application::PolicyOverlayCaps Application::PolicyCaps() const {
    const bool legacy_arm_overlay_supported = locomotion_->SupportsArmOverlay();
    // H4 overlay chỉ thuộc chế độ đi bộ + teleop; ở chế độ thường gói Quest bị bỏ qua.
    const bool h4_overlay_supported = h4_teleop_approved_ &&
        loco_mode_ == LocoMode::kTeleop &&
        locomotion_->SupportsH4TeleopOverlay();
    const bool history_gesture_contract = tuning_.gesture_history_enabled &&
        locomotion_->SupportsHistoryGestureOverlay();
    const bool gesture_overlay_supported = legacy_arm_overlay_supported ||
        history_gesture_contract;
    return {legacy_arm_overlay_supported, h4_overlay_supported,
            history_gesture_contract, gesture_overlay_supported};
}

// H4/gait: đếm thời gian đứng yên ổn định trước khi cho gesture bắt đầu.
bool Application::UpdateHistoryGestureDwell(const RobotState& rs,
                                            bool history_gesture_contract,
                                            bool history_gesture_move_requested) {
    bool history_gesture_ready = !history_gesture_contract;
    if (history_gesture_contract) {
        bool stable = rs.gyro.norm() <= tuning_.gait_stability_gyro_norm &&
                      JointVelocityRms(rs) <= tuning_.gait_stability_joint_velocity_rms;
        if (const auto* gait = dynamic_cast<const FlatPlusGaitController*>(locomotion_)) {
            stable = stable && gait->gait().mode() == GaitMode::Stand && gait->gait().stable();
        }
        const bool still = !history_gesture_move_requested && stable;
        gesture_history_still_s_ = still
            ? std::min(tuning_.gesture_history_stand_dwell_s,
                       gesture_history_still_s_ + spec::kLoopDt)
            : 0.0f;
        history_gesture_ready = gesture_history_still_s_ >=
            tuning_.gesture_history_stand_dwell_s;

        // H4 có bốn frame. Sau khi tay thu xong, giữ base 80 ms để history
        // chỉ còn các frame không mask trước khi nhận lệnh di chuyển trở lại.
        if (!arm_gesture_.Idle()) {
            gesture_history_release_s_ = 4.0f * spec::kPolicyDt;
        } else if (gesture_history_release_s_ > 0.0f) {
            gesture_history_release_s_ = std::max(
                0.0f, gesture_history_release_s_ - spec::kLoopDt);
        }
    } else {
        gesture_history_still_s_ = 0.0f;
        gesture_history_release_s_ = 0.0f;
    }
    return history_gesture_ready;
}

// Lệnh vận tốc cho chân: chỉ theo cần analog khi locomotion không bị giữ.
void Application::UpdateBaseCommand(const InputCommand& cmd, const PolicyOverlayCaps& caps,
                                    bool holding) {
    const bool legacy_arm_overlay_supported = caps.legacy_arm;
    const bool h4_overlay_supported = caps.h4;
    const bool history_gesture_contract = caps.history_gesture;
    // teleop_lock_base: khi teleop đang ARMED thì bỏ qua cần analog -> chân chỉ đứng
    // giữ thăng bằng, teleop đúng nghĩa CHỈ điều khiển tay + đầu.
    const bool history_gesture_holds_base = history_gesture_contract &&
        (!arm_gesture_.Idle() || gesture_history_release_s_ > 0.0f);
    const bool teleop_locks_base = tuning_.teleop_lock_base &&
        ((teleop_runtime_on_ && legacy_arm_overlay_supported) ||
         (h4_overlay_supported &&
          (teleop_.H4Authorized() || teleop_.Weight() > 0.0f ||
           h4_history_release_s_ > 0.0f || h4_head_release_s_ > 0.0f)));
    if (state_ == AppState::kLocomotion && !holding && !teleop_locks_base &&
        !history_gesture_holds_base) {
        cmd_vx_ = cmd.vx;
        cmd_vy_ = cmd.vy;
        cmd_yaw_ = cmd.yaw;
    } else {
        cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    }
}

// Thân trên trong LOCOMOTION/MIMIC: chọn nguồn (teleop > gesture), cập nhật quyền,
// và trả mask/lean/head cho bước inference + gửi lệnh.
Application::UpperBodyCommand Application::UpdateUpperBody(
        const InputCommand& cmd, const RobotState& rs, const PolicyOverlayCaps& caps,
        bool holding, float move_intent, bool history_gesture_ready,
        bool history_gesture_move_requested) {
    const bool legacy_arm_overlay_supported = caps.legacy_arm;
    const bool h4_overlay_supported = caps.h4;
    const bool history_gesture_contract = caps.history_gesture;
    const bool gesture_overlay_supported = caps.gesture;
    // --- Thân trên: legacy 83-D dùng overlay; FlatPlus từ chối bằng capability. ---
    bool  ub_active = false;      // có override tay cho ctx/obs không
    bool  use_teleop = false;     // nguồn active là teleop?
    float ub_mask_keep = 1.0f, ub_lean = 0.0f;
    float ub_slot_grav = 0.0f, ub_slot_vx = 0.0f;   // bù riêng của slot đang phát
    HeadTarget head_tgt{};
    if (h4_overlay_supported)
        head_tgt.max_rate_rad_s = tuning_.teleop_head_rate_limit_rad_s;
    if (tuning_.gesture_enabled || tuning_.teleop_enabled) {
        // !holding: đang announce dance hoặc đang chờ settle để đổi trạng thái thì tay
        // phải thu về. Vung tay đúng cửa sổ HandleTransitions đo settle_gyro_max sẽ dịch
        // CoM ngay lúc đang quyết định có chuyển hay không.
        bool loco = (state_ == AppState::kLocomotion) && !holding;
        float gx = rs.projected_gravity.x(), gy = rs.projected_gravity.y();
        float tilt = std::sqrt(gx * gx + gy * gy);
        bool guard_bad = tilt > tuning_.gesture_guard_tilt ||
                         rs.gyro.norm() > tuning_.gesture_guard_gyro;

        const bool h4_operator_ready = state_fresh_ &&
            input_.gamepad().InputActive() && input_.gamepad().packet_valid();
        bool h4_packet_authorized = h4_overlay_supported && teleop_runtime_on_ &&
            teleop_.H4Authorized();
        if (h4_overlay_supported && (!loco || guard_bad || !h4_operator_ready)) {
            teleop_.BeginH4Session();
            h4_packet_authorized = false;
        }
        if (h4_packet_authorized && !arm_gesture_.Idle())
            arm_gesture_.RetractSafety();

        // Truyền cả điều kiện mất quyền vào Update: weight của teleop tự ramp về 0 theo
        // teleop_retract_s. Nếu chỉ tắt use_teleop mà vẫn báo armed=true thì weight đứng
        // nguyên ~1 và BlendInto ngừng đột ngột -> tay GIẬT về tư thế policy.
        if (tuning_.teleop_enabled && (legacy_arm_overlay_supported || h4_overlay_supported)) {
            const bool legacy_grant = legacy_arm_overlay_supported && teleop_runtime_on_;
            const bool h4_grant = h4_packet_authorized && h4_operator_ready &&
                                  arm_gesture_.Idle();
            teleop_.Update(spec::kLoopDt, (legacy_grant || h4_grant) && loco && !guard_bad,
                           h4_overlay_supported);
        }
        if (h4_overlay_supported) {
            if (teleop_.Weight() > 0.0f)
                h4_history_release_s_ = 4.0f * spec::kPolicyDt;
            else if (h4_history_release_s_ > 0.0f)
                h4_history_release_s_ = std::max(0.0f,
                    h4_history_release_s_ - spec::kLoopDt);
            if (teleop_.HeadValid())
                h4_head_release_s_ =
                    std::max(tuning_.teleop_head_yaw_max,
                             tuning_.teleop_head_pitch_max) /
                    tuning_.teleop_head_rate_limit_rad_s + 0.1f;
            else if (h4_head_release_s_ > 0.0f)
                h4_head_release_s_ = std::max(0.0f,
                    h4_head_release_s_ - spec::kLoopDt);
        }
        // ARMED mà không có nguồn UDP -> cảnh báo MỘT lần, tránh hiểu nhầm là hỏng nút.
        if (teleop_runtime_on_ && !teleop_no_source_warned_ && !teleop_.Active()) {
            const float armed_s = std::chrono::duration<float>(
                std::chrono::steady_clock::now() - teleop_armed_at_).count();
            if (armed_s > kTeleopNoSourceWarnS) {
                teleop_no_source_warned_ = true;
                std::cout << "[Teleop] ĐANG BẬT nhưng chưa nhận gói UDP nào ở cổng "
                          << tuning_.teleop_udp_port << " sau " << kTeleopNoSourceWarnS
                          << "s — kiểm tra bộ phát (scripts/teleop_send_test.py).\n";
            }
        }
        use_teleop = tuning_.teleop_enabled && teleop_.Active() &&
            ((legacy_arm_overlay_supported && teleop_runtime_on_ && loco && !guard_bad) ||
             (h4_overlay_supported && teleop_runtime_on_ && state_ == AppState::kLocomotion));
        const float block_at = std::max(0.0f, tuning_.gesture_voice_move_threshold);
        const float resume_below = std::clamp(
            tuning_.gesture_voice_resume_threshold, 0.0f, block_at);
        if (move_intent > block_at) {
            voice_gesture_movement_blocked_ = true;
            voice_gesture_still_s_ = 0.0f;
        } else if (voice_gesture_movement_blocked_) {
            if (move_intent < resume_below) {
                voice_gesture_still_s_ += spec::kLoopDt;
                if (voice_gesture_still_s_ >=
                    std::max(0.0f, tuning_.gesture_voice_resume_delay_s)) {
                    voice_gesture_movement_blocked_ = false;
                    voice_gesture_still_s_ = 0.0f;
                }
            } else {
                voice_gesture_still_s_ = 0.0f;
            }
        }

        // Gesture chỉ còn quyền overlay khi đang ở locomotion, policy hỗ trợ tay,
        // và teleop chưa chiếm quyền. Các state không gọi BlendInto thì phải xóa
        // gesture, nếu không nó sẽ giữ pose cũ và tái xuất hiện sau khi đổi mode.
        // Khi an toàn, nút tay cầm và voice-auto dùng chung player; nút tay cầm có thể
        // thay gesture trong lúc voice đang nói mà không trigger lại mỗi tick.
        const bool gesture_can_overlay = loco && gesture_overlay_supported &&
            !use_teleop && !h4_packet_authorized;
        ProcessVoiceGestureOwnerRequest(
            gesture_can_overlay, guard_bad,
            !history_gesture_contract || history_gesture_ready,
            history_gesture_move_requested,
            use_teleop || h4_packet_authorized);
        if (!gesture_can_overlay) {
            // Báo có tiếng khi nuốt một cú double-click, tránh bị hiểu là hỏng nút.
            if (cmd.want_gesture)
                std::cout << "[Gesture] bỏ qua slot " << cmd.want_gesture << " — "
                          << (guard_bad      ? "robot đang nghiêng/lắc"
                              : use_teleop   ? "teleop đang giữ tay"
                              : !loco        ? "không ở LOCOMOTION (hoặc đang chuyển trạng thái)"
                              : history_gesture_move_requested ? "có lệnh di chuyển, đang thu tay"
                                             : "nguồn thân trên chưa sẵn sàng")
                          << ".\n";
            arm_gesture_.Cancel();
            voice_gesture_latched_ = false;
        } else if (guard_bad || h4_history_release_s_ > 0.0f ||
                   h4_head_release_s_ > 0.0f || history_gesture_move_requested) {
            // Guard, lệnh di chuyển hoặc H4 handover yêu cầu thu an toàn. Lệnh
            // lặp mỗi tick phải idempotent để không reset tiến độ quỹ đạo return.
            arm_gesture_.RetractSafety();
            voice_gesture_latched_ = false;
        // Sau khi gesture đã bắt đầu, vẫn cho phép nút/voice yêu cầu thu hoặc
        // đổi gesture. Cửa đứng-yên chỉ chặn lần BẮT ĐẦU từ trạng thái idle.
        } else if (!history_gesture_contract || history_gesture_ready || !arm_gesture_.Idle()) {
            HandleVoiceGesture(voice_gesture_movement_blocked_);
            HandleGestureTrigger(cmd);
        } else if (cmd.want_gesture) {
            std::cout << "[Gesture] bỏ qua slot " << cmd.want_gesture
                      << " — H4/gait chưa đứng ổn định đủ "
                      << tuning_.gesture_history_stand_dwell_s << "s.\n";
        }
        // Nếu thân còn đang bị kích thích thì giữ target thu hiện tại. Đây chỉ
        // điều tiết overlay tay; chân vẫn hoàn toàn do policy locomotion quyết định.
        arm_gesture_.SetReturnPaused(
            rs.gyro.norm() > tuning_.gesture_return_pause_gyro_norm);
        arm_gesture_.Update(spec::kLoopDt);

        if (use_teleop) {
            // Head-only packets may own the head but must not mask/override arms.
            if (teleop_.ArmValid()) {
                ub_active = true;
                ub_mask_keep = 1.0f - teleop_.Weight();
                ub_lean = teleop_.LeanPitchProxy();
            }
            if (teleop_.HeadValid())
                head_tgt = {true, teleop_.HeadYaw(), teleop_.HeadPitch(),
                            tuning_.teleop_head_rate_limit_rad_s};
        } else if (loco && gesture_overlay_supported && arm_gesture_.Active()) {
            ub_active = true;
            ub_mask_keep = 1.0f - arm_gesture_.Weight();
            ub_lean = arm_gesture_.LeanPitchProxy();
            // Bù riêng của slot: nhân weight -> lên/xuống cùng nhịp blend-in và
            // retract, không nhảy bậc lúc vừa bấm. Weight=0 -> bias=0 tuyệt đối.
            if (legacy_arm_overlay_supported) {
                const int slot = tuning_.GestureSlotIndex(arm_gesture_.ActiveName());
                if (slot >= 0) {
                    const float w = arm_gesture_.Weight();
                    const float trim_deg = std::clamp(tuning_.gesture_slot_trim_deg[slot],
                                                      -Tuning::kGestureTrimDegMax,
                                                      Tuning::kGestureTrimDegMax);
                    const float vx_bias = std::clamp(tuning_.gesture_slot_vx_bias[slot],
                                                     -Tuning::kGestureVxBiasMax,
                                                     Tuning::kGestureVxBiasMax);
                    ub_slot_grav = std::sin(trim_deg * static_cast<float>(M_PI) / 180.0f) * w;
                    ub_slot_vx   = vx_bias * w;
                }
            }
            if (arm_gesture_.HeadActive()) {
                const auto head = arm_gesture_.ReferenceHead();
                // Các NPZ gesture hiện hữu được record theo thứ tự IDL vật lý
                // [pitch(29), yaw(30)]. Đổi sang HeadTarget ngữ nghĩa
                // [yaw, pitch] tại đúng một biên để giữ nguyên chuyển động cũ.
                head_tgt = {
                    true,
                    std::clamp(head[1], -tuning_.gesture_head_yaw_max,
                               tuning_.gesture_head_yaw_max),
                    std::clamp(head[0], -tuning_.gesture_head_pitch_max,
                               tuning_.gesture_head_pitch_max),
                    tuning_.gesture_head_rate_limit_rad_s,
                };
            }
        }
    }
    return {ub_active, use_teleop, ub_mask_keep, ub_lean, ub_slot_grav, ub_slot_vx, head_tgt};
}

// Một bước policy (mỗi kPolicyDecimation tick). false = đã EnterIdle vì ONNX lỗi.
bool Application::StepPolicy(const RobotState& rs, const PolicyOverlayCaps& caps,
                             const UpperBodyCommand& ub) {
    const bool legacy_arm_overlay_supported = caps.legacy_arm;
    const bool h4_overlay_supported = caps.h4;
    const bool gesture_overlay_supported = caps.gesture;
    const bool  ub_active = ub.active;
    const float ub_mask_keep = ub.mask_keep, ub_lean = ub.lean;
    const float ub_slot_grav = ub.slot_grav, ub_slot_vx = ub.slot_vx;
    float cmd_norm = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_ +
                               cmd_yaw_ * cmd_yaw_);
    ControlContext ctx{rs, cmd_vx_, cmd_vy_, cmd_yaw_, gait_.PhaseObs(cmd_norm)};
    if (ub_active && (gesture_overlay_supported || h4_overlay_supported)) {
        // H4/gait mask frame 83-D before history. Feedforward balance stays
        // legacy-only until it has its own acceptance evidence.
        ctx.arm_override_active = true;
        ctx.arm_mask_keep = ub_mask_keep;
        if (legacy_arm_overlay_supported) {
            ctx.gravity_x_bias = tuning_.gesture_balance_kg * ub_lean + ub_slot_grav;
            ctx.cmd_vx_bias    = tuning_.gesture_balance_kv * ub_lean + ub_slot_vx;
        }
    }
    try {
        ai_target_q_ = active_->Step(ctx);
    } catch (const std::exception& e) {
        std::cerr << "\n[FATAL] ONNX inference failed in " << active_->Name() << ": "
                  << e.what() << "\n";
        EnterIdle("ONNX inference failure - triggering damping");
        return false;
    } catch (...) {
        std::cerr << "\n[FATAL] Unknown ONNX inference failure in " << active_->Name()
                  << "\n";
        EnterIdle("Unknown ONNX inference failure - triggering damping");
        return false;
    }

    if (blend_alpha_ < 1.0f) {
        blend_alpha_ = std::min(1.0f, blend_alpha_ + spec::kPolicyDt / tuning_.blend_time_s);
        for (int i = 0; i < spec::kNumJoints; ++i) {
            ai_target_q_[i] = (1.0f - blend_alpha_) * blend_start_q_[i] +
                               blend_alpha_ * ai_target_q_[i];
        }
    }
    return true;
}

// MIMIC: ramp gain từ return gains, transition v2 và recovery cooldown trên ai_target_q_.
void Application::ShapeMimicTarget(MimicController* mimic,
                                   std::array<float, spec::kNumJoints>& kp_eff,
                                   std::array<float, spec::kNumJoints>& kd_eff) {
    if (mimic) {
        float wp = mimic->WarmupProgress();
        float g  = wp * wp * (3.0f - 2.0f * wp);   // smoothstep 0->1 theo warmup
        // Ankle (khớp yếu nhất) tụt gain CHẬM hơn: giữ ~return_kp_ankle gần hết warmup,
        // chỉ nhả về 40 ở đoạn cuối => hết bập bênh trong lúc soft-start.
        float g_ank = g * g * g;
        for (int i = 0; i < spec::kNumJoints; ++i) {
            bool is_ankle = (i == 4 || i == 5 || i == 10 || i == 11);
            float gi = is_ankle ? g_ank : g;
            kp_eff[i] = (1.0f - gi) * return_gains_kp_[i] + gi * kp_eff[i];
            kd_eff[i] = (1.0f - gi) * stand_gains_kd_[i] + gi * kd_eff[i];
        }
    }
    if (mimic && !dance_stopping_ && mimic->TransitionV2Enabled()) {
        // Keep the joint-space reference trajectory as the command during soft-start.
        // The actor is mixed in only as the reference reaches the selected
        // entry frame, so ONNX output cannot create a first-frame target jump.
        const float u = mimic->WarmupProgress();
        const float actor_w = u * u * (3.0f - 2.0f * u);
        if (actor_w < 1.0f) {
            const auto& entry_ref = mimic->TelemetryReferenceQ();
            for (int i = 0; i < spec::kNumJoints; ++i) {
                ai_target_q_[i] = (1.0f - actor_w) * entry_ref[i]
                    + actor_w * ai_target_q_[i];
            }
        }
    }
    if (mimic && dance_stopping_) {
        // Keep the actor's stabilizing target during the normal cooldown. If
        // it remains outside the q/dq gate after max_s, blend it gradually
        // toward the measured-to-stand reference instead of locking or making
        // a target step.
        const float recovery_s = std::max(0.5f, tuning_.mimic_cooldown_s);
        const float recovery_u = std::clamp(
            (mimic->CooldownElapsedS() - tuning_.mimic_handover_max_s)
                / recovery_s,
            0.0f, 1.0f);
        const float recovery_w = recovery_u * recovery_u
            * (3.0f - 2.0f * recovery_u);
        if (recovery_w > 0.0f) {
            const auto& stand_ref = mimic->CooldownReferencePosition();
            for (int i = 0; i < spec::kNumJoints; ++i) {
                ai_target_q_[i] = (1.0f - recovery_w) * ai_target_q_[i]
                    + recovery_w * stand_ref[i];
            }
        }
    }
}

void Application::UpdateGestureHandover(const RobotState& rs, bool gesture_overlay_supported,
                                        bool use_teleop) {
    // Ghi đè góc khớp tay từ nguồn active (teleop/gesture) trên BẢN SAO mỗi tick
    // (tránh trộn chồng tích luỹ); ai_target_q_ thô giữ nguyên cho lần blend kế.
    // Head teleop ưu tiên; nếu không có teleop, gesture head_pos truyền xuống sender.
    // invalid -> giữ 0. NO-OP với NPZ không có head_pos.
    // Sau pha return, tiếp tục giữ overlay ở weight=0 cho tới khi target policy,
    // tay đo thực và IMU cùng ổn định đủ dwell. Nhờ vậy việc mở lại history/H4
    // không tạo một bước target_q ở frame nhả quyền.
    if (gesture_overlay_supported && !use_teleop && arm_gesture_.HandoverPending()) {
        float arm_dq_sq = 0.0f;
        float max_arm_target_error = 0.0f;
        for (int joint = ArmGesturePlayer::kArmBegin;
             joint < ArmGesturePlayer::kArmBegin + ArmGesturePlayer::kNumArm; ++joint) {
            const float measured_q = rs.q[joint];
            const float measured_dq = rs.dq[joint];
            arm_dq_sq += measured_dq * measured_dq;
            max_arm_target_error = std::max(max_arm_target_error,
                                            std::fabs(measured_q - ai_target_q_[joint]));
        }
        const float gx = rs.projected_gravity.x(), gy = rs.projected_gravity.y();
        const float tilt = std::sqrt(gx * gx + gy * gy);
        const float gyro_norm = rs.gyro.norm();
        const float arm_dq_rms = std::sqrt(
            arm_dq_sq / static_cast<float>(ArmGesturePlayer::kNumArm));
        const bool policy_target_matches = arm_gesture_.HandoverMatches(
            ai_target_q_, tuning_.gesture_handover_position_tol_rad,
            tuning_.gesture_handover_arm_dq_rms);
        const bool measured_position_matches =
            max_arm_target_error <= tuning_.gesture_handover_measured_position_tol_rad;
        gesture_handover_log_s_ += spec::kLoopDt;
        if (gesture_handover_log_s_ >= 1.0f) {
            gesture_handover_log_s_ = 0.0f;
            std::cout << "[Gesture] handover wait policy=" << (policy_target_matches ? 1 : 0)
                      << " measured=" << (measured_position_matches ? 1 : 0)
                      << " q_err=" << max_arm_target_error
                      << " arm_dq_rms=" << arm_dq_rms
                      << " tilt=" << tilt << " gyro=" << gyro_norm << "\n";
        }
        if (gesture_handover_guard_.Advance(
                policy_target_matches && measured_position_matches, arm_dq_rms, tilt, gyro_norm,
                tuning_.gesture_handover_arm_dq_rms, tuning_.gesture_handover_tilt_rad,
                tuning_.gesture_handover_gyro_norm, spec::kLoopDt)) {
            arm_gesture_.ReleaseHandover();
            std::cout << "[Gesture] Handover an toan sau "
                      << tuning_.gesture_handover_dwell_s << "s on dinh.\n";
        }
    } else {
        gesture_handover_guard_.Reset();
        gesture_handover_log_s_ = 0.0f;
    }
}

// MIMIC: bật nhạc sau soft-start, soft-stop khi hết bài, bàn giao về LOCOMOTION.
void Application::UpdateMimicLifecycle(MimicController* mimic, const RobotState& rs) {
    // Bật nhạc khi soft-start hoàn thành.
    // !dance_stopping_: không bật nhạc khi đang soft-stop (điệu đã kết thúc / bị huỷ).
    if (mimic && !music_started_ && !dance_stopping_ && mimic->WarmupDone()) {
        music_started_ = true;
        auto it = music_files_.find(pending_dance_key_);
        if (it != music_files_.end())
            music_.Play(it->second, tuning_.dance_volume[pending_dance_key_ - 2]);
        std::cout << "[Application] Soft-start done - DANCE " << pending_dance_key_ << " running.\n";
    }

    // Soft-stop khi kết thúc điệu nhảy.
    if (mimic && mimic->IsFinished())
        BeginSoftStop("Dance finished");

    // Bàn giao cho locomotion khi robot ổn định sau cooldown.
    if (mimic && dance_stopping_) {
        float gx = rs.projected_gravity.x(), gy = rs.projected_gravity.y();
        float tilt = std::sqrt(gx * gx + gy * gy);
        float gyro_n = rs.gyro.norm();
        bool settled = tilt < tuning_.mimic_handover_tilt && gyro_n < tuning_.mimic_handover_gyro;
        const bool pose_ready = mimic->CooldownPoseReady(
            rs, tuning_.mimic_transition_exit_pos_tol,
            tuning_.mimic_transition_exit_dq_tol);
        const float cooldown_pos_err = mimic->CooldownMaxPositionError(rs);
        const float cooldown_max_dq = mimic->CooldownMaxJointSpeed(rs);
        const float cooldown_recovery_s = std::max(0.5f, tuning_.mimic_cooldown_s);
        const float cooldown_recovery_deadline =
            tuning_.mimic_handover_max_s + cooldown_recovery_s;
        const bool bounded_fallback =
            mimic->TransitionV2Enabled() && !pose_ready && settled && mimic->CooldownDone() &&
            mimic->CooldownReady(tuning_.mimic_handover_max_s) &&
            cooldown_pos_err <= tuning_.mimic_transition_fallback_pos_tol &&
            cooldown_max_dq <= tuning_.mimic_transition_fallback_dq_tol;
        // Legacy keeps its historical max-time fallback. Transition v2 is
        // fail-closed: a quiet torso is not enough if q/dq still show a squat
        // or a moving leg. A bounded fallback is allowed only after max_s and
        // is explicitly logged as degraded handover.
        const bool handover_ready =
            (settled && mimic->CooldownDone() &&
             mimic->CooldownReady(tuning_.mimic_handover_min_s) && pose_ready) ||
            (!mimic->TransitionV2Enabled() &&
             mimic->CooldownReady(tuning_.mimic_handover_max_s)) ||
            bounded_fallback;
        if (handover_ready) {
            std::cout << "[Application] Robot standing (tilt=" << tilt << ", gyro=" << gyro_n
                      << (settled ? ") settled" : ") CAP max_s reached");
            if (bounded_fallback) {
                std::cout << " [bounded fallback max_pos_err=" << cooldown_pos_err
                          << ", max_dq=" << cooldown_max_dq << "]";
            }
            std::cout << " - handing over to LOCOMOTION.\n";
            dance_stopping_ = false;
            ActivatePolicy(locomotion_);   // ActivatePolicy trả trim locomotion
            state_ = AppState::kLocomotion;
            lock_block_timer_ = std::max(0.0f, tuning_.dance_abort_lock_block_s);
        } else if (mimic->TransitionV2Enabled() &&
                   mimic->CooldownReady(cooldown_recovery_deadline) &&
                   settled && !pose_ready && !bounded_fallback &&
                   !mimic_handover_block_logged_) {
            std::cerr << "[Application] Transition v2 holds mimic cooldown past recovery window: "
                      << "pose gate not ready (tilt=" << tilt << ", gyro=" << gyro_n
                      << ", max_pos_err=" << cooldown_pos_err
                      << ", max_dq=" << cooldown_max_dq
                      << "). Entering STAND LOCK recovery instead of dumping to locomotion.\n";
            mimic_handover_block_logged_ = true;
            BeginStandUp(rs, /*warn_first=*/false);
            return;
        }
    }
}
