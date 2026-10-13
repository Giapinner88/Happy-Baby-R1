// Upper-body sources: gestures, voice gestures, UDP teleop (incl. Mode Z).
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
#include <sys/stat.h>
#include <time.h>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;
using namespace app_internal;

/**
 * Nạp thư viện gesture theo tên cấu hình trong tuning.yaml (`gesture_slot_N: vaytay` ->
 * `vaytay.loop.npz` lặp hoặc `vaytay.npz` dừng ở frame cuối; slot trống -> tự tìm
 * `<N>.loop.npz`/`<N>.npz`).
 * Dùng `JointTrajectory` (khớp format `record_motion`: chỉ cần joint_pos+fps, không cần
 * joint_vel/body_quat_w như MotionData của dance). Luôn trích 10 cột tay (policy 14-23);
 * có head_pos thì phát qua kênh LowCmd head độc lập. Thiếu file gesture cho 1 slot chỉ
 * vô hiệu slot đó, không ảnh hưởng locomotion.
 */
void Application::InitGestures() {
    if (!tuning_.gesture_enabled) return;

    std::array<float, 10> def_arm;
    const auto& dq = locomotion_->default_q();
    for (int j = 0; j < 10; ++j) def_arm[j] = dq[14 + j];
    arm_gesture_.Init(def_arm, tuning_.gesture_blend_in_s,
                      tuning_.gesture_retract_s,
                      tuning_.gesture_safety_retract_s,
                      tuning_.gesture_safety_max_vel_rad_s);
    arm_gesture_.SetReturnLimits({tuning_.gesture_return_min_duration_s,
                                  tuning_.gesture_return_max_velocity_rad_s,
                                  tuning_.gesture_return_max_acceleration_rad_s2});
    gesture_handover_guard_.Configure(tuning_.gesture_handover_dwell_s);
    voice_gesture_auto_enabled_ = tuning_.gesture_voice_auto_start;

    struct Cand { const char* suffix; bool loop; };
    const Cand cands[] = {{".loop.npz", true}, {".npz", false}};

    int loaded = 0;
    auto load_named = [&](const std::string& base, const std::string& label,
                          float speed = 1.0f, float blend_ovr = 0.0f,
                          float retract_ovr = 0.0f) {
        if (base.empty()) return false;
        if (arm_gesture_.Has(base)) return true;
        for (const Cand& c : cands) {
            std::string path = proj_dir_ + "/" + tuning_.gesture_folder + "/" + base + c.suffix;
            std::ifstream probe(path);
            if (!probe.good()) continue;
            try {
                JointTrajectory traj;
                traj.Load(path);
                std::vector<std::array<float, 10>> frames;
                std::vector<std::array<float, ArmGesturePlayer::kNumHead>> head_frames;
                frames.reserve(traj.num_frames());
                if (traj.has_head()) head_frames.reserve(traj.num_frames());
                for (int f = 0; f < traj.num_frames(); ++f) {
                    const auto& jp = traj.frame(f);
                    std::array<float, 10> a;
                    for (int j = 0; j < 10; ++j) a[j] = jp[14 + j];
                    frames.push_back(a);
                    if (traj.has_head()) head_frames.push_back(traj.head_frame(f));
                }
                const float playback_fps =
                    traj.fps() * std::clamp(speed, 0.5f, 2.0f);

                // Clip LẶP: SampleActive() nối frame cuối -> frame đầu trong ĐÚNG MỘT
                // frame, nên vòng hở bao nhiêu là tay giật bấy nhiêu, lặp lại mỗi vòng.
                // Cùng ngưỡng 0.15 rad với tools/make_gesture.py; vá bằng --loop-bridge-s.
                if (c.loop && frames.size() >= 2) {
                    float seam = 0.0f;
                    for (int j = 0; j < 10; ++j)
                        seam = std::max(seam, std::abs(frames.back()[j] - frames.front()[j]));
                    if (seam > 0.15f) {
                        std::cerr << "[Gesture] ⚠ '" << base << "' VONG LAP HO " << seam
                                  << " rad -> moi vong giat " << seam * playback_fps
                                  << " rad/s. Cat lai bang tools/make_gesture.py --loop-bridge-s.\n";
                    }
                }

                arm_gesture_.AddGesture(base, std::move(frames), playback_fps, c.loop,
                                        blend_ovr, retract_ovr, std::move(head_frames));
                ++loaded;
                std::cout << "[Gesture] " << label << " = '" << base << "' <- " << path
                          << " (" << traj.num_frames() << " frames, loop=" << (c.loop ? 1 : 0)
                          << ", fps=" << playback_fps
                          << ", head=" << (traj.has_head() ? 1 : 0)
                          << ", blend=" << (blend_ovr > 1e-3f ? blend_ovr : tuning_.gesture_blend_in_s)
                          << ", retract=" << (retract_ovr > 1e-3f ? retract_ovr
                                                                   : tuning_.gesture_retract_s)
                          << ")\n";
            } catch (const std::exception& e) {
                std::cerr << "[Gesture] " << label << " loi nap " << path << ": "
                          << e.what() << "\n";
                return false;
            }
            return true;
        }
        return false;
    };

    for (int slot = 1; slot <= 8; ++slot) {
        if (slot == 5 && !tuning_.gesture_voice_name.empty())
            continue;   // A dành riêng bật/tắt voice-auto
        // Tên khai báo trong yaml; trống -> fallback theo số slot.
        std::string base = tuning_.gesture_slot[slot - 1];
        if (base.empty()) base = std::to_string(slot);
        if (load_named(base, "slot " + std::to_string(slot),
                       tuning_.gesture_slot_speed[slot - 1],
                       tuning_.gesture_slot_blend[slot - 1],
                       tuning_.gesture_slot_retract[slot - 1]))
            gesture_slot_[slot] = base;
    }

    if (!tuning_.gesture_voice_name.empty() &&
        load_named(tuning_.gesture_voice_name, "voice-auto",
                   tuning_.gesture_voice_speed)) {
        std::cout << "[Gesture] voice-auto marker: " << tuning_.gesture_voice_marker
                  << " (stale=" << tuning_.gesture_voice_stale_s << "s, startup="
                  << (voice_gesture_auto_enabled_ ? "ON" : "OFF")
                  << ", speed=" << std::clamp(tuning_.gesture_voice_speed, 0.5f, 2.0f)
                  << ", double-click A de bat/tat)\n";
    }

    if (loaded == 0 && tuning_.gesture_demo_wave) {
        arm_gesture_.AddGesture("demo", ArmGesturePlayer::MakeDemoWave(def_arm), 50.0f, true);
        gesture_slot_[1] = "demo";
        std::cout << "[Gesture] Khong co npz -> nap gesture MAU vao slot 1 (double-click Up).\n";
    }
    std::cout << "[Gesture] Da nap " << loaded << " gesture. "
                 "Double-click nut R3: 1=Up 2=Down 3=Left 4=Right "
                 "5=A(voice-auto) 6=B 7=X 8=Y.\n";
}

// Mở socket UDP teleop (tay + đầu). Chỉ khi teleop_enabled. Gọi từ Run() sau InitControllers.
void Application::InitTeleop() {
    if (!tuning_.teleop_enabled) return;
    std::array<float, 10> def_arm;
    const auto& dq = locomotion_->default_q();
    for (int j = 0; j < 10; ++j) def_arm[j] = dq[14 + j];
    teleop_.Init(def_arm, /*enabled=*/true, tuning_.teleop_udp_port,
                 tuning_.teleop_timeout_ms, tuning_.teleop_blend_in_s,
                 tuning_.teleop_retract_s, tuning_.teleop_smooth_hz,
                 tuning_.teleop_head_yaw_max, tuning_.teleop_head_pitch_max);
    std::cout << "[Teleop] Lang nghe LOOPBACK 127.0.0.1:" << tuning_.teleop_udp_port
              << " (tay+dau). Mode Z san sang trong ZERO TORQUE; co Quest + R3 "
                 "la hai lop deadman. ";
    if (h4_teleop_approved_) {
        std::cout << "H4: R2+A nha can ve 0 (base khoa), R2+X cho phep di chuyen; "
                     "nhan START de arm/disarm; "
                     "ZERO TORQUE: START truc tiep.\n";
    } else if (locomotion_->SupportsArmOverlay()) {
        std::cout << "Teleop legacy: nhan START mot lan trong LOCOMOTION.\n";
    } else {
        std::cout << "Teleop legacy khong ho tro contract locomotion nay; H4 "
                     "chi duoc phep khi da qua artifact gate.\n";
    }
}

// Bật/tắt cờ ARMED của teleop từ MỘT chỗ: nút START, hoặc tự nhả khi rời LOCOMOTION.
// Reset luôn mốc thời gian + cờ cảnh báo "chưa có nguồn" để lần bật sau lại cảnh báo được.
void Application::SetTeleopArmed(bool on, const char* reason) {
    if (teleop_runtime_on_ == on) return;
    teleop_runtime_on_ = on;
    teleop_armed_at_ = std::chrono::steady_clock::now();
    teleop_no_source_warned_ = false;
    std::cout << "\n[Teleop] " << (on ? "BẬT" : "TẮT") << " (" << reason << ") — "
              << (on ? "nhận điều khiển tay/đầu qua UDP; chân giữ thăng bằng"
                     : "nhả tay/đầu về policy") << "\n";
    const std::string& v = on ? tuning_.voice_teleop_on : tuning_.voice_teleop_off;
    if (!v.empty()) music_.Say(v, tuning_.voice_volume, tuning_.voice_speaker_id);
}

// Double-click nút R3 -> chơi/thu gesture ở slot tương ứng (chỉ gọi khi đang Locomotion).
void Application::HandleGestureTrigger(const InputCommand& cmd) {
    int slot = cmd.want_gesture;
    if (slot < 1 || slot > 8) return;
    if (slot == 5 && !tuning_.gesture_voice_name.empty()) {
        voice_gesture_auto_enabled_ = !voice_gesture_auto_enabled_;
        if (!voice_gesture_auto_enabled_) {
            if (arm_gesture_.ActiveName() == tuning_.gesture_voice_name &&
                arm_gesture_.PendingName().empty())
                arm_gesture_.Retract();
            voice_gesture_latched_ = false;
        }
        std::cout << "[Gesture] double-click A -> voice-auto "
                  << (voice_gesture_auto_enabled_ ? "BAT" : "TAT") << ".\n";
        return;
    }
    const std::string& name = gesture_slot_[slot];
    if (name.empty()) {
        std::cout << "[Gesture] slot " << slot << " chua gan gesture nao.\n";
        return;
    }
    // Gesture bấm tay cầm luôn thắng voice-auto. Player sẽ thu gesture hiện tại
    // về default hoàn toàn rồi mới chạy yêu cầu thủ công này.
    voice_gesture_latched_ = false;
    if (voice_command_gesture_active_) {
        // A physical gesture request takes ownership; the player keeps the
        // established return/queue behavior for the newly selected slot.
        ClearVoiceCommandGesture();
    }
    arm_gesture_.Trigger(name);
}

bool Application::VoiceGestureSpeaking() {
    if (tuning_.gesture_voice_name.empty() || tuning_.gesture_voice_marker.empty())
        return false;

    voice_gesture_poll_accum_s_ += spec::kLoopDt;
    if (voice_gesture_poll_accum_s_ < 0.1f)
        return voice_gesture_marker_fresh_;
    voice_gesture_poll_accum_s_ = 0.0f;

    struct stat st {};
    if (::stat(tuning_.gesture_voice_marker.c_str(), &st) != 0) {
        voice_gesture_marker_fresh_ = false;
        return false;
    }

    struct timespec now {};
    if (::clock_gettime(CLOCK_REALTIME, &now) != 0) {
        voice_gesture_marker_fresh_ = false;
        return false;
    }
    const double age_s = static_cast<double>(now.tv_sec - st.st_mtim.tv_sec) +
                         static_cast<double>(now.tv_nsec - st.st_mtim.tv_nsec) / 1e9;
    voice_gesture_marker_fresh_ =
        age_s >= -0.25 && age_s <= std::max(0.1f, tuning_.gesture_voice_stale_s);
    return voice_gesture_marker_fresh_;
}

void Application::HandleVoiceGesture(bool movement_blocked) {
    const std::string& name = tuning_.gesture_voice_name;
    const bool speaking = voice_gesture_auto_enabled_ && !movement_blocked &&
                          VoiceGestureSpeaking();
    if (speaking && !voice_gesture_latched_) {
        // Không chen ngang gesture thủ công. Chỉ tự bắt đầu khi player đã về default.
        if (arm_gesture_.Has(name) && arm_gesture_.Idle()) {
            arm_gesture_.Trigger(name);
            voice_gesture_latched_ = true;
            std::cout << "[Gesture] voice bat dau -> '" << name << "'.\n";
        }
    } else if (!speaking && voice_gesture_latched_) {
        if (arm_gesture_.ActiveName() == name && arm_gesture_.PendingName().empty())
            arm_gesture_.Retract();
        voice_gesture_latched_ = false;
        std::cout << (!voice_gesture_auto_enabled_
                          ? "[Gesture] voice-auto tat -> thu tay.\n"
                          : movement_blocked
                          ? "[Gesture] co lenh locomotion -> ngung tay voice.\n"
                          : "[Gesture] voice dung -> thu tay.\n");
    }
}

void Application::ProcessVoiceGestureOwnerRequest(
        bool gesture_can_overlay, bool guard_bad, bool history_ready,
        bool movement_requested, bool teleop_owns_upper_body) {
    if (voice_command_gesture_active_ && arm_gesture_.Idle()) {
        ClearVoiceCommandGesture();
    }

    const bool release_for_safety = !gesture_can_overlay || guard_bad ||
        movement_requested || voice_gesture_movement_blocked_ || teleop_owns_upper_body;
    if (voice_command_gesture_active_ && !arm_gesture_.Idle()) {
        if (release_for_safety) {
            arm_gesture_.RetractSafety();
            voice_command_retract_requested_ = true;
        } else if (!voice_command_retract_requested_ && arm_gesture_.ActiveClipFinished()) {
            // Use the loaded clip's registered frame count/fps and slot speed;
            // no asset duration table is duplicated in the command dispatcher.
            arm_gesture_.Retract();
            voice_command_retract_requested_ = true;
        }
    }

    if (!voice_gesture_owner_.HasPending()) return;
    const VoiceGestureOwner::Request request = voice_gesture_owner_.Pending();
    if (request.command != "play") return;
    auto reject = [&](const char* reason) {
        voice_gesture_owner_.Reply(false, "rejected", reason);
    };

    if (!tuning_.gesture_enabled || !armed_ || state_ != AppState::kLocomotion) {
        reject("not_in_locomotion");
        return;
    }
    if (teleop_owns_upper_body) {
        reject("teleop_owns_upper_body");
        return;
    }
    if (guard_bad) {
        reject("stability_guard");
        return;
    }
    if (!gesture_can_overlay) {
        reject("overlay_unavailable");
        return;
    }
    if (movement_requested || voice_gesture_movement_blocked_) {
        reject("movement_requested");
        return;
    }
    if (!history_ready) {
        reject("stand_dwell_incomplete");
        return;
    }
    if (!arm_gesture_.Idle()) {
        reject("gesture_busy");
        return;
    }

    const int slot = tuning_.GestureVoiceSlot(request.gesture_id);
    if (slot == 0) {
        reject("gesture_disabled");
        return;
    }
    if (slot < 1 || slot > 8 || slot == 5) {
        reject("unknown_gesture");
        return;
    }
    const std::string& name = gesture_slot_[slot];
    if (name.empty() || !arm_gesture_.Has(name)) {
        reject("gesture_asset_unavailable");
        return;
    }
    if (arm_gesture_.Loops(name)) {
        reject("looping_asset_unsupported");
        return;
    }

    arm_gesture_.Trigger(name);
    voice_gesture_latched_ = false;
    voice_command_gesture_active_ = true;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_ = request.gesture_id;
    std::cout << "[GestureOwner] accepted " << request.gesture_id
              << " -> configured slot " << slot << " ('" << name << "').\n";
    voice_gesture_owner_.Reply(true, "accepted", "", request.gesture_id);
}

void Application::RunZeroTorqueTeleop() {
    if (!tuning_.teleop_enabled) {
        sender_.ClearNonTeleopHold();
        sender_.ZeroTorque(estimator_.state().mode_machine);
        return;
    }

    // R3 phải vừa ở trạng thái hoạt động VÀ gói remote hiện tại phải hợp lệ.
    // Không cho cửa sổ SUSPECT 3 giây của locomotion duy trì PD tay khi người
    // giữ E-stop đã thực sự mất liên lạc. Quest enable/watchdog là deadman thứ hai.
    const bool operator_ready = state_fresh_ &&
                                input_.gamepad().InputActive() &&
                                input_.gamepad().packet_valid();
    const bool h4_start_gate = !h4_teleop_approved_ || teleop_runtime_on_;
    teleop_.Update(spec::kLoopDt, operator_ready && h4_start_gate,
                   h4_teleop_approved_);
    if (!teleop_.Active()) {
        // Khi thụ động, đồng bộ slew origin theo encoder để lần bóp cò kế tiếp
        // không kéo tay/đầu về một target cũ.
        SyncSenderToLatestState();
        // Nhả deadman là nhả PD chân+eo ngay; không để robot còn được cấp dòng
        // sau khi teleop đã mất authority.
        sender_.ClearNonTeleopHold();
        sender_.ZeroTorque(estimator_.state().mode_machine);
        return;
    }

    // Chốt encoder ở đúng cạnh inactive -> active. Chốt sớm hơn có thể giữ một
    // tư thế cũ và tạo bước target khi người vận hành bóp cò.
    if (tuning_.teleop_lock_others_enabled &&
        !sender_.NonTeleopHoldLatched()) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.LatchNonTeleopHold(shared_low_state_);
    }

    // Encoder hiện tại là gốc. BlendInto chỉ chạm policy index 14..23 và chỉ
    // khi kênh tay có authority; packet head-only không được energize tay.
    std::array<float, spec::kNumJoints> target = estimator_.state().q;
    teleop_.BlendInto(target);
    HeadTarget head;
    if (teleop_.HeadValid()) {
        head = {true, teleop_.HeadYaw(), teleop_.HeadPitch(),
                tuning_.teleop_max_rate_rad_s};
    }
    sender_.SendUpperBodyZeroTorque(target, teleop_.ArmValid(),
                                    tuning_.teleop_arm_kp,
                                    tuning_.teleop_arm_kd,
                                    tuning_.teleop_max_rate_rad_s,
                                    estimator_.state().mode_machine, head,
                                    tuning_.teleop_lock_others_enabled
                                        ? tuning_.teleop_lock_kp : 0.0f,
                                    tuning_.teleop_lock_kd,
                                    tuning_.teleop_lock_max_rate_rad_s);
}
