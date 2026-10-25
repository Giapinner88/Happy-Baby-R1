// Operator-driven state transitions and the Begin* entry points.
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

void Application::RequestFromPolicy(PendingAction act, bool has_chair) {
    if (pending_action_ == act) return;
    CancelLocoModeSwap("co lenh tu the");

    // Nếu đang nhảy, thực hiện soft-stop trước khi chuyển trạng thái để tránh mất thăng bằng.
    if (state_ == AppState::kMimic) BeginSoftStop("Stopping dance");
    announcing_ = false;
    pending_action_ = act;
    if (act == PendingAction::kToSit) pending_sit_has_chair_ = has_chair;
    settle_timer_ = 0.0f;
    input_.ZeroVelocity();
    // Phát âm thanh cảnh báo trước khi kích hoạt stand lock.
    if (act == PendingAction::kToStandLock)
        music_.Say(tuning_.voice_stand_lock, tuning_.voice_volume, tuning_.voice_speaker_id);
    const char* act_name = act == PendingAction::kToStandLock ? "STAND LOCK"
                          : act == PendingAction::kToSit       ? "SIT"
                                                                 : "LIE DOWN";
    std::cout << "[Application] Requesting transitions: " << act_name
              << " - settling robot under policy first.\n";
}

// Dừng mềm (soft-stop): Giữ policy chạy và trượt reference về tư thế đứng thẳng để giữ thăng bằng.
void Application::BeginSoftStop(const char* why) {
    auto it = mimics_.find(pending_dance_key_);
    if (it == mimics_.end() || dance_stopping_) return;
    std::cout << "[Application] " << why << " - soft-stop: mimic policy standing robot up first.\n";
    music_.Stop();
    // Giữ music_started_ để không bật lại nhạc khi đang cooldown.
    announcing_ = false;
    dance_stopping_ = true;
    mimic_handover_block_logged_ = false;
    it->second->BeginCooldown(locomotion_->default_q(), tuning_.mimic_cooldown_s,
                              estimator_.state());
}

// Huỷ STAND LOCK đang chờ settle dưới policy (chưa rời LOCOMOTION): ở lại policy
// đang chạy, cần điều khiển dùng lại được ngay.
void Application::CancelPendingStandLock(const char* why) {
    std::cout << "[Application] " << why << ": huy STAND LOCK dang cho -> o lai LOCOMOTION ("
              << LocoModeName(loco_mode_) << ").\n";
    pending_action_ = PendingAction::kNone;
    settle_timer_ = 0.0f;
    music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
}

// Huỷ vào dance khi đang RETURNING (gồng PD về default, policy đã tắt): bật lại
// policy đi bộ đang chọn. Không blend: sim (results/returning_cancel_20261024)
// cho thấy blend 0.5 s giữ target cũ trong lúc thân đã trôi, nghiêng gấp ~2 lần
// so với trả quyền ngay cho policy vừa reset.
void Application::CancelReturningToLocomotion(const char* why) {
    std::cout << "[Application] " << why << ": huy vao DANCE " << pending_dance_key_
              << " khi RETURNING -> LOCOMOTION.\n";
    music_.Stop();
    ActivatePolicy(locomotion_);
    blend_alpha_ = 1.0f;
    state_ = AppState::kLocomotion;
    // L2+Lên còn giữ sau khi huỷ không được thành "xin STAND LOCK".
    lock_block_timer_ = std::max(0.0f, tuning_.dance_abort_lock_block_s);
}

void Application::AbortDance() {
    if (dance_stopping_) return;
    BeginSoftStop("Dance aborted");
    pending_action_ = PendingAction::kNone;
    music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
}

void Application::BeginStandUp(const RobotState& rs, bool warn_first) {
    std::cout << "[Application] Transition to STAND LOCK.\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    lying_ = false;
    // Điều khiển âm thanh và tư thế khi chuyển sang stand lock.
    if (warn_first) {
        music_.Say(tuning_.voice_stand_lock, tuning_.voice_volume, tuning_.voice_speaker_id);
        stand_warn_timer_ = 0.0f;
    } else {
        stand_warn_timer_ = std::max(0.0f, tuning_.stand_lock_warn_s);
    }
    stand_start_q_ = rs.q;
    stand_timer_ = 0.0f;
    SyncSenderToLatestState();
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kStandUp;
}

void Application::BeginSit(const RobotState& rs, bool has_chair) {
    std::cout << "[Application] Sit down sequence... (" << (has_chair ? "co ghe" : "TU DONG - KHONG ghe, khong ai do") << ")\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    lying_ = false;
    music_.Say(tuning_.voice_sit_down, tuning_.voice_volume, tuning_.voice_speaker_id);
    sit_start_q_ = rs.q;
    sit_timer_ = 0.0f;
    sit_phase_ = 0;
    sit_has_chair_ = has_chair;
    SyncSenderToLatestState();
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kSafeShutdown;
}

// Robot đang nằm? Cờ lying_ HOẶC IMU nghiêng quá ngưỡng (xem khai báo ở Application.hpp).
bool Application::RobotLying(const RobotState& rs) const {
    // Đứng thẳng: projected_gravity.z ~ -1. Nằm: ~ -cos(82°) ~ -0.14.
    // Ngưỡng: nghiêng hơn lying_tilt_deg so với phương đứng -> coi là nằm.
    const float thresh = -std::cos(tuning_.lying_tilt_deg * static_cast<float>(M_PI) / 180.0f);
    return lying_ || rs.projected_gravity.z() > thresh;
}

// Gồng cứng robot từ tư thế nằm rũ mềm về tư thế nằm chuẩn, chuẩn bị đứng dậy.
void Application::BeginGetUp(const RobotState& rs) {
    if (!getup_motion_) {
        std::cerr << "[Application] GET UP: chưa nạp getup.npz - bỏ qua lệnh.\n";
        return;
    }
    std::cout << "[Application] Chuẩn bị đứng dậy: gồng cứng về tư thế nằm-chuẩn, "
                 "giữ chờ L2+X.\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    traj_start_q_ = rs.q;
    traj_timer_ = 0.0f;
    traj_phase_ = 0;   // 0=blend về tư thế nằm chuẩn, sau đó 1=giữ chờ L2+X
    SyncSenderToLatestState();
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kGetUp;
}

// Bắt đầu chuỗi nằm xuống (lie down).
void Application::BeginLieDown(const RobotState& rs) {
    if (!liedown_motion_) {
        std::cerr << "[Application] LIE DOWN: chưa nạp liedown.npz - bỏ qua lệnh.\n";
        return;
    }
    std::cout << "[Application] Bắt đầu NẰM XUỐNG (lie-down)...\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    if (!tuning_.voice_lie_down.empty())
        music_.Say(tuning_.voice_lie_down, tuning_.voice_volume, tuning_.voice_speaker_id);
    traj_start_q_ = rs.q;
    traj_timer_ = 0.0f;
    traj_phase_ = 0;
    SyncSenderToLatestState();
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kLieDown;
}

void Application::HandleTransitions(const InputCommand& cmd) {
    const RobotState& rs = estimator_.state();

    if (lock_block_timer_ > 0.0f) lock_block_timer_ -= spec::kLoopDt;
    if (sit_block_timer_  > 0.0f) sit_block_timer_  -= spec::kLoopDt;
    if (getup_liedown_block_timer_ > 0.0f) getup_liedown_block_timer_ -= spec::kLoopDt;

    if (cmd.want_stand_lock && HandleStandLockRequest(rs)) return;
    if (cmd.want_safe_shutdown && HandleSitRequest(rs)) return;
    if (cmd.want_get_up_down && HandleGetUpDownRequest(rs)) return;

    if (pending_action_ == PendingAction::kToStandLock && state_ == AppState::kLocomotion &&
        (cmd.want_locomotion || cmd.want_teleop_locomotion)) {
        // R2+A/R2+X chỉ huỷ, không đổi policy; muốn đổi thì bấm thêm lần nữa.
        CancelPendingStandLock(cmd.want_teleop_locomotion ? "R2+X" : "R2+A");
        return;
    }
    if (pending_action_ != PendingAction::kNone && state_ == AppState::kLocomotion) {
        AdvancePendingAction(rs);
        return;
    }

    // R2+A/R2+X chọn hai policy locomotion. Từ STAND LOCK: vào thẳng chế độ
    // đó. Đang LOCOMOTION: xin đổi chế độ, đổi thật khi robot đứng yên.
    if (cmd.want_locomotion || cmd.want_teleop_locomotion) {
        const LocoMode mode = cmd.want_teleop_locomotion ? LocoMode::kTeleop : LocoMode::kWalk;
        if (state_ == AppState::kStandLock) {
            EnterLocomotion(mode);
        } else if (state_ == AppState::kLocomotion) {
            RequestLocoMode(mode);
        } else {
            std::cout << "[Application] Blocked: Must be in STAND_LOCK to activate policy.\n";
        }
    }
    if (loco_swap_pending_) {
        if (state_ == AppState::kLocomotion) AdvanceLocoModeSwap(rs);
        else CancelLocoModeSwap("roi LOCOMOTION");
    }

    if (cmd.want_mimic_key && state_ == AppState::kLocomotion) StartDanceAnnounce(cmd);
    if (announcing_ && state_ == AppState::kLocomotion) AdvanceDanceAnnounce(rs);

    if (cmd.want_reset &&
        (state_ == AppState::kLocomotion || state_ == AppState::kMimic)) {
        std::cout << "[Application] Resetting controller: " << active_->Name() << "\n";
        active_->Reset(rs);
    }
}

// L2+Lên. true = đã chuyển state, HandleTransitions dừng ở đây.
bool Application::HandleStandLockRequest(const RobotState& rs) {
    if (state_ == AppState::kMimic) {
        // Đang nhảy: Chỉ hủy điệu nhảy, không gồng cứng ngay.
        AbortDance();
    } else if (state_ == AppState::kReturningToDefault) {
        // Robot đang gồng về default chờ vào dance: huỷ bằng tay, trả về
        // LOCOMOTION thay vì đợi return_timeout_s hoặc bấm E-stop.
        CancelReturningToLocomotion("L2+Len");
        return true;
    } else if (state_ == AppState::kLocomotion) {
        if (pending_action_ == PendingAction::kToStandLock) {
            // Bấm lại trong lúc chờ: người lái đổi ý / bấm nhầm.
            CancelPendingStandLock("L2+Len lan nua");
        } else if (lock_block_timer_ > 0.0f) {
            std::cout << "[Application] Stand-lock blocked for another "
                      << lock_block_timer_ << "s after dance abort.\n";
        } else {
            RequestFromPolicy(PendingAction::kToStandLock);
        }
    } else if ((state_ == AppState::kIdle || state_ == AppState::kZeroTorque) &&
               RobotLying(rs)) {
        // Chuẩn bị đứng dậy từ trạng thái nằm (gồng cứng về tư thế nằm chuẩn).
        // RobotLying dùng cả IMU nên bắt được cả khi robot nằm mà cờ lying_ chưa đặt
        // (đặt nằm bằng built-in/bằng tay, hoặc app restart lúc đang nằm).
        BeginGetUp(rs);
        return true;
    } else if (state_ == AppState::kIdle || state_ == AppState::kSafeShutdown ||
               state_ == AppState::kZeroTorque) {
        // Từ Zero Torque, L2+Lên thoát limp và chuyển sang stand lock.
        // (Nhánh nằm đã bắt ở trên -> tới đây IMU xác nhận robot đang dựng đứng,
        // không còn nguy cơ stand-lock trong lúc nằm.)
        BeginStandUp(rs, /*warn_first=*/true);
        return true;
    }
    return false;
}

// L2+Trái (giữ). true = đã vào SAFE SHUTDOWN.
bool Application::HandleSitRequest(const RobotState& rs) {
    if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
        RequestFromPolicy(PendingAction::kToSit, /*has_chair=*/true);   // L2+Trái: có ghế
    } else if (state_ == AppState::kStandLock) {
        if (sit_block_timer_ > 0.0f) {
            std::cout << "[Application] Sit blocked for another "
                      << sit_block_timer_ << "s after entering STAND LOCK.\n";
        } else {
            BeginSit(rs, /*has_chair=*/true);   // L2+Trái: có ghế
            return true;
        }
    }
    return false;
}

// L2+X (giữ): nằm xuống khi đứng, đứng dậy khi nằm. true = đã chuyển state.
bool Application::HandleGetUpDownRequest(const RobotState& rs) {
    if (state_ == AppState::kGetUp && traj_phase_ == 1) {
        // Bước 2 đứng dậy: Bắt đầu phát getup.npz.
        std::cout << "[Application] ĐỨNG DẬY: phát chuỗi getup.npz -> Locomotion khi xong.\n";
        if (!tuning_.voice_get_up.empty())
            music_.Say(tuning_.voice_get_up, tuning_.voice_volume, tuning_.voice_speaker_id);
        traj_phase_ = 2;
        traj_timer_ = 0.0f;
        return true;
    } else if (state_ == AppState::kStandLock) {
        if (getup_liedown_block_timer_ > 0.0f) {
            std::cout << "[Application] Lie-down blocked for another "
                      << getup_liedown_block_timer_ << "s.\n";
        } else {
            BeginLieDown(rs);   // đang ĐỨNG -> nằm xuống
            return true;
        }
    } else if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
        RequestFromPolicy(PendingAction::kToLieDown);   // dừng dưới policy rồi nằm
    } else if ((state_ == AppState::kIdle || state_ == AppState::kZeroTorque) &&
               RobotLying(rs)) {
        // L2+X chung: IMU nói robot đang NẰM -> lần 1 chuẩn bị (gồng về nằm-chuẩn),
        // lần 2 (khi kGetUp phase 1, nhánh đầu ở trên) mới thực sự đứng dậy.
        BeginGetUp(rs);
        return true;
    } else {
        std::cout << "[Application] Lie-down/Get-up blocked in state " << StateName() << ".\n";
    }
    return false;
}

// Đang chờ robot đứng yên dưới policy trước khi STAND LOCK / SIT / LIE DOWN.
void Application::AdvancePendingAction(const RobotState& rs) {
    // Normal handover waits for arm retract and four unmasked H4 frames.
    // Emergency/fall paths above remain immediate and never wait here.
    if (h4_teleop_approved_ &&
        (teleop_.Weight() > 0.0f || h4_history_release_s_ > 0.0f ||
         h4_head_release_s_ > 0.0f))
        return;
    settle_timer_ += spec::kLoopDt;
    float min_t  = std::max(0.0f, tuning_.settle_time_s);
    // Chờ hết thời gian cảnh báo khi chuyển sang stand lock.
    if (pending_action_ == PendingAction::kToStandLock)
        min_t = std::max(min_t, std::max(0.0f, tuning_.stand_lock_warn_s));
    bool  quiet  = rs.gyro.norm() <= tuning_.settle_gyro_max;
    bool  capped = settle_timer_ >= min_t * 3.0f + 0.5f;
    if (settle_timer_ >= min_t && (quiet || capped)) {
        if (!quiet) {
            std::cout << "[Application] Settle timeout reached, proceeding with transition.\n";
        }
        PendingAction act = pending_action_;
        pending_action_ = PendingAction::kNone;
        if (act == PendingAction::kToStandLock)    BeginStandUp(rs, /*warn_first=*/false);
        else if (act == PendingAction::kToSit)     BeginSit(rs, pending_sit_has_chair_);
        else                                        BeginLieDown(rs);
    }
}

// R1+nút (hoặc phím 2-8): đọc tên điệu, chờ voice rồi mới vào RETURNING.
void Application::StartDanceAnnounce(const InputCommand& cmd) {
    if (loco_swap_pending_) {
        std::cout << "[Application] Dance " << cmd.want_mimic_key
                  << " blocked: dang doi che do di bo.\n";
        return;
    }
    if (mimics_.find(cmd.want_mimic_key) != mimics_.end()) {
        std::cout << "[Application] Announcing dance " << cmd.want_mimic_key << "\n";
        pending_dance_key_ = cmd.want_mimic_key;
        StartMimicTelemetry(pending_dance_key_);
        announcing_ = true;
        announce_timer_ = 0.0f;
        announce_saw_busy_ = false;
        mimic_entry_quiet_s_ = 0.0f;
        mimic_entry_wait_logged_ = false;
        input_.ZeroVelocity();
        int vidx = cmd.want_mimic_key - 1;
        announce_has_voice_ = (vidx >= 0 && vidx < Tuning::kMaxDances
                               && !tuning_.voice_mimic[vidx].empty()
                               && music_.ready());
        if (announce_has_voice_)
            music_.Say(tuning_.voice_mimic[vidx], tuning_.voice_volume, tuning_.voice_speaker_id);
    } else {
        const auto dit = disabled_dance_reasons_.find(cmd.want_mimic_key);
        if (dit != disabled_dance_reasons_.end()) {
            std::cout << "[Application] Dance " << cmd.want_mimic_key
                      << " is disabled: " << dit->second << "\n";
        } else {
            std::cout << "[Application] Dance " << cmd.want_mimic_key
                      << " not configured.\n";
        }
    }
}

void Application::AdvanceDanceAnnounce(const RobotState& rs) {
    announce_timer_ += spec::kLoopDt;
    if (music_.busy()) announce_saw_busy_ = true;

    bool proceed;
    if (announce_has_voice_) {
        // Chờ voice phát HẾT (busy đã true rồi về false), dài hay ngắn đều trọn.
        const float min_s = std::max(0.0f, tuning_.mimic_announce_min_s);
        const float max_s = std::max(min_s, tuning_.mimic_announce_timeout_s);
        const bool voice_done = announce_saw_busy_ && !music_.busy();
        proceed = (announce_timer_ >= min_s) &&
                  (voice_done || announce_timer_ >= max_s);
    } else {
        // Không có voice để chờ: giữ hành vi cũ theo delay cố định.
        proceed = announce_timer_ >= std::max(0.0f, tuning_.mimic_announce_delay_s);
    }

    int quiet_worst_joint = -1;
    const float max_dq = MaxJointSpeed(rs, quiet_worst_joint);
    const bool quiet = rs.gyro.norm() <= tuning_.mimic_entry_gyro_max &&
                       max_dq <= tuning_.mimic_entry_dq_max;
    mimic_entry_quiet_s_ = mimic_transition::AdvanceQuietDwell(
        mimic_entry_quiet_s_, spec::kLoopDt, quiet, tuning_.mimic_entry_settle_s);
    const bool entry_settled = mimic_transition::QuietDwellReady(
        mimic_entry_quiet_s_, tuning_.mimic_entry_settle_s);
    if (proceed && !entry_settled && !mimic_entry_wait_logged_) {
        std::cout << "[Application] Dance entry waiting for locomotion settle "
                  << "(gyro=" << rs.gyro.norm() << ", max_dq=" << max_dq
                  << ", limits=" << tuning_.mimic_entry_gyro_max << "/"
                  << tuning_.mimic_entry_dq_max << ").\n";
        mimic_entry_wait_logged_ = true;
    }
    proceed = proceed && entry_settled &&
        (!h4_teleop_approved_ ||
         (teleop_.Weight() <= 0.0f && h4_history_release_s_ <= 0.0f &&
          h4_head_release_s_ <= 0.0f));

    if (proceed) {
        announcing_ = false;
        announce_saw_busy_ = false;
        announce_has_voice_ = false;
        mimic_entry_quiet_s_ = 0.0f;
        mimic_entry_wait_logged_ = false;
        std::cout << "[Application] Announce done (t=" << announce_timer_
                  << "s) - returning to default pose for DANCE "
                  << pending_dance_key_ << "\n";
        return_timer_ = 0.0f;
        state_ = AppState::kReturningToDefault;
    }
}
