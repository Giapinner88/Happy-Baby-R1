// Core loop: lifecycle, DDS callbacks, Tick dispatch, safety exits, HUD.
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
#include <unitree/robot/channel/channel_factory.hpp>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;
using namespace app_internal;

/**
 * Cờ yêu cầu dừng ứng dụng, được kích hoạt bởi signal handler (SIGTERM/SIGINT).
 */
std::atomic<bool> Application::s_stop_requested_{false};

void Application::RequestStop(int /*sig*/) {
    s_stop_requested_.store(true, std::memory_order_relaxed);
}

Application::Application(std::string proj_dir, std::string interface_override)
    : proj_dir_(std::move(proj_dir)) {
    if (!tuning_.LoadFromFile(proj_dir_ + "/config/tuning.yaml")) {
        throw std::runtime_error("Khong the nap day du config/tuning.yaml (hoac mot file include)");
    }
    if (!interface_override.empty()) tuning_.network_interface = interface_override;
    music_.SetProjectRoot(proj_dir_);
    music_.SetStateCallback([this](bool busy) {
        audio_busy_.store(busy);
        integration_notifier_.Send(busy, armed_.load());
    });

    for (int i = 0; i < spec::kNumJoints; ++i) {
        bool is_arm = (i >= 14);
        bool is_waist = (i == 12 || i == 13);
        bool is_ankle = (i == 4 || i == 5 || i == 10 || i == 11);
        stand_gains_kp_[i] = is_arm ? tuning_.stand_kp_arm
                            : is_waist ? tuning_.stand_kp_waist
                                       : tuning_.stand_kp_leg;
        stand_gains_kd_[i] = tuning_.stand_kd;

        // Lưu ý: Chỉ giảm độ cứng (mềm hơn) ở cổ chân khi bắt đầu chuyển sang chế độ nhảy (dance).
        // Các khớp hông, gối, eo và tay vẫn giữ nguyên hệ số như `stand_gains_kp_` để duy trì tư thế mặc định vững chắc.
        return_gains_kp_[i] = is_ankle ? tuning_.return_kp_ankle : stand_gains_kp_[i];

        sit_gains_kp_[i] = is_arm ? tuning_.stand_kp_arm : tuning_.sit_kp_leg;
        sit_gains_kd_[i] = tuning_.sit_kd;

        getup_gains_kp_[i] = is_arm ? tuning_.getup_kp_arm
                            : is_waist ? tuning_.getup_kp_waist
                                       : tuning_.getup_kp_leg;
        getup_gains_kd_[i] = tuning_.getup_kd;
    }

    ort_opts_.SetIntraOpNumThreads(1);
    ort_opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
}

void Application::InitDds() {
    std::string uri = std::string(getenv("HOME") ? getenv("HOME") : "") +
                      "/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml";
    if (std::ifstream(uri).good()) {
        setenv("CYCLONEDDS_URI", ("file://" + uri).c_str(), 1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, tuning_.network_interface);
    sender_.Init(tuning_);

    low_state_sub_ = std::make_unique<
        unitree::robot::ChannelSubscriber<LowState_>>("rt/lowstate");
    low_state_sub_->InitChannel([this](const void* msg) { OnLowState(msg); }, 1);

    // Lắng nghe topic rt/lowcmd để phát hiện các lệnh điều khiển từ hệ thống built-in thông qua kiểm tra mã CRC.
    if (tuning_.arm_gate_enabled) {
        // LƯU Ý QUAN TRỌNG: Cần phải thực hiện disarm TRƯỚC KHI bật bộ lắng nghe.
        // Mặc định `armed_` = true, nếu bật bộ lắng nghe lúc này thì các gói tin từ built-in lúc khởi động (~621Hz) 
        // sẽ bị hệ thống hiểu nhầm là "xung đột" và chốt cứng cờ `conflict_flag_`, dẫn đến báo động giả ngay khi vừa arm.
        armed_ = false;
        last_foreign_ms_.store(NowMs());   // có cửa sổ quan sát tối thiểu, không arm tức thì
        low_cmd_sub_ = std::make_unique<
            unitree::robot::ChannelSubscriber<LowCmd_>>("rt/lowcmd");
        low_cmd_sub_->InitChannel([this](const void* msg) { OnLowCmdSeen(msg); }, 1);
    }

    // Giám sát pin qua topic.
    if (tuning_.battery_monitor_enabled) {
        battery_.Init(tuning_.battery_topic);
        std::cout << "[Application] Battery monitor on " << tuning_.battery_topic << "\n";
    }

    if (tuning_.voice_enabled || !music_files_.empty()) {
        music_.InitAudio();
        std::cout << "[Application] AudioClient: "
                  << (music_.ready() ? "ready" : "not ready")
                  << " (voice=" << (tuning_.voice_enabled ? "on" : "off")
                  << ", " << music_files_.size() << " dance tunes loaded)\n";
    }
}

// Slew origin của sender = encoder mới nhất (gói DDS gần nhất), đọc dưới state_mutex_.
void Application::SyncSenderToLatestState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sender_.SyncToState(shared_low_state_);
}

void Application::ClearVoiceCommandGesture() {
    voice_command_gesture_active_ = false;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_.clear();
}

void Application::OnLowState(const void* msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    shared_low_state_ = *static_cast<const LowState_*>(msg);
    got_state_ = true;
    ++state_sequence_;
    last_state_time_ = std::chrono::steady_clock::now();
}

/**
 * Chạy trên luồng DDS để phát hiện xung đột điều khiển.
 * Hàm này đếm số lượng các gói lệnh `lowcmd` chứa mã CRC không hợp lệ (không phải của app) 
 * xuất hiện trong một khoảng thời gian nhất định (cửa sổ thời gian).
 */
void Application::OnLowCmdSeen(const void* msg) {
    const auto& c = *static_cast<const LowCmd_*>(msg);
    if (sender_.IsOurCrc(c.crc())) return;

    int64_t now = NowMs();
    last_foreign_ms_.store(now);
    foreign_count_.fetch_add(1);
    last_foreign_crc_.store(c.crc());
    if (!armed_) return;

    // Reset chuỗi đếm nếu gói lạ quá xa gói trước.
    int64_t prev = conflict_first_ms_.load();
    if (prev == 0 || now - conflict_last_ms_.load() > tuning_.arm_conflict_window_ms) {
        conflict_first_ms_.store(now);
        conflict_run_ = 1;
    } else {
        conflict_run_++;
    }
    conflict_last_ms_.store(now);
    if (conflict_run_ >= tuning_.arm_conflict_min) conflict_flag_.store(true);
}

int Application::Run() {
    std::cout << "====================================\n"
              << " HB R1 High-Level Runner\n"
              << " Interface : " << tuning_.network_interface << "\n"
              << "====================================\n";

    InitControllers();
    InitDds();
    InitTeleop();

    estimator_.Configure(tuning_, spec::kLoopDt);
    fall_detector_.Configure(tuning_, spec::kLoopDt);
    input_.Configure(tuning_);

    // Khởi động ở kDisarmed (không gửi lệnh).
    if (tuning_.arm_gate_enabled) {
        armed_ = false;
        state_ = AppState::kDisarmed;
        disarmed_start_time_ = std::chrono::steady_clock::now();
        std::cout << "[Application] Cổng an toàn BẬT - chờ bàn giao: L2+R2 (built-in dev mode) "
                     "rồi giữ R1+R2 " << tuning_.arm_hold_s << "s để run_r1 tiếp quản.\n";
    }

    KeyboardX11::SetEmergencyDampFn([this]() {
        if (armed_) sender_.Damping(estimator_.state().mode_machine);   // disarmed/conflict -> KHÔNG publish (đè built-in)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });
    input_.Start();
    if (tuning_.gesture_enabled)
        voice_gesture_owner_.Init(tuning_.gesture_voice_command_socket);

    std::cout << "[Application] Waiting for DDS connection...\n";

    auto next_wake = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(2000);

    while (running_) {
        if (s_stop_requested_.load(std::memory_order_relaxed)) {
            std::cout << "\n[Application] Nhận tín hiệu dừng (SIGTERM/SIGINT).\n";
            break;
        }
        if (!Tick()) break;
        next_wake += period;
        auto now = std::chrono::steady_clock::now();
        if (next_wake < now) next_wake = now;
        std::this_thread::sleep_until(next_wake);
        ++tick_;
    }

    // Chỉ gửi Damping khi đang giữ quyền điều khiển hợp lệ.
    if (armed_) {
        std::cout << "\n[Application] Exiting - setting motor damping...\n";
        sender_.Damping(estimator_.state().mode_machine);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else {
        std::cout << "\n[Application] Exiting - chưa arm, thoát IM LẶNG (không đè built-in).\n";
    }
    voice_gesture_owner_.Close();
    input_.Stop();
    telemetry_.Shutdown();
    std::cout << "[Application] Exited safely.\n";
    return 0;
}

bool Application::Tick() {
    ServiceVoiceGestureOwner();
    PublishIntegrationStatus();
    LowState_ low;
    bool got;
    long state_age_ms = 0;
    {
        // last_state_time_ được DDS callback ghi TRONG mutex này; đọc nó ngoài
        // mutex (watchdog cũ) là data race thật sự — tính tuổi ngay tại đây.
        std::lock_guard<std::mutex> lock(state_mutex_);
        low = shared_low_state_;
        got = got_state_;
        new_state_sample_ = got && state_sequence_ != consumed_state_sequence_;
        consumed_state_sequence_ = state_sequence_;
        input_sample_dt_s_ = sample_progress_.Update(new_state_sample_, last_state_time_);
        // Chưa có gói nào = cũ vô hạn, KHÔNG phải 0 (0 sẽ đọc thành "vừa mới").
        state_age_ms = got
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - last_state_time_).count()
            : std::numeric_limits<long>::max();
    }
    // Gói cũ hơn watchdog DDS thì mọi thứ đọc từ nó (kể cả byte tay cầm) đều
    // không còn giá trị. Cờ này đi kèm `low` xuống mọi consumer.
    const bool previous_state_fresh = state_fresh_;
    state_fresh_ = got && state_age_ms <= static_cast<long>(tuning_.state_timeout_ms);
    remote_state_fresh_ = got && state_age_ms <= static_cast<long>(tuning_.remote_state_timeout_ms);
    // Bất biến cuối: cảm biến chết thì KHÔNG đường nào cấp PD được, kể cả các
    // đường tự gate riêng (Mode Z) hay state chưa có tên trong watchdog.
    sender_.SetSensorFresh(state_fresh_);

    // Kiểm tra xung đột built-in trước khi publish.
    if (conflict_flag_.load() && armed_) {
        if (!conflict_diag_logged_) {
            conflict_diag_logged_ = true;
            uint32_t fc = last_foreign_crc_.load();
            // Bỏ qua gói tự phản hồi (self-echo).
            std::cerr << "[CONFLICT-DIAG] foreign_crc=0x" << std::hex << fc << std::dec
                      << " IsOurCrc_recheck=" << sender_.IsOurCrc(fc)
                      << " sent_count=" << sender_.SentCount()
                      << " foreign_count=" << foreign_count_.load() << "\n";
        }
        if (tuning_.arm_conflict_release) {
            EnterConflict();
            return true;
        }
        // arm_conflict_release=false: BỎ QUA, tiếp tục lái (dùng khi chắc built-in đã tắt hẳn).
        conflict_flag_.store(false);
        conflict_run_ = 0;
    }
    if (state_ == AppState::kConflict) return true;   // im tuyệt đối, latch
    if (state_ == AppState::kDisarmed) {
        if (new_state_sample_) estimator_.Update(low);
        input_.Update(low, remote_state_fresh_, new_state_sample_, input_sample_dt_s_);
        return TickDisarmed(input_.GetMergedCommand());
    }
    if (state_fresh_ && !previous_state_fresh) sender_.SyncToState(low);

    if (state_ == AppState::kWaitingForState) {
        if (!got) {
            if (input_.keyboard().WantExit()) {
                running_ = false;
                return false;
            }
            return true;
        }
        std::cout << "[Application] Connected - system ready (IDLE).\n";
        sender_.SyncToState(low);
        estimator_.Reset();
        state_ = AppState::kIdle;
        // Hoãn voice chào mừng để tránh đè âm thanh của built-in.
        if (tuning_.startup_voice_delay_s > 0.0f) {
            startup_voice_pending_ = true;
            startup_voice_at_ = std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<int>(tuning_.startup_voice_delay_s * 1000));
        } else {
            music_.Say(tuning_.voice_startup, tuning_.voice_volume, tuning_.voice_speaker_id);
        }
    }

    // Voice khởi động đã tới hẹn -> phát (sau khi built-in nói xong "Development Mode").
    if (startup_voice_pending_ && std::chrono::steady_clock::now() >= startup_voice_at_) {
        startup_voice_pending_ = false;
        music_.Say(tuning_.voice_startup, tuning_.voice_volume, tuning_.voice_speaker_id);
    }

    const bool state_ok = estimator_.Update(low);

    input_.Update(low, remote_state_fresh_, new_state_sample_, input_sample_dt_s_);
    InputCommand cmd = input_.GetMergedCommand();

    if (input_.keyboard().WantExit()) {
        EnterIdle("ESC pressed - Emergency stop");
        running_ = false;
        return false;
    }

    // LowState hỏng: không có tư thế đáng tin thì không policy nào chạy đúng và
    // fall detector cũng mất cơ sở -> damping. Đặt SAU phím thoát để ESC vẫn
    // dùng được khi IMU/encoder đang lỗi.
    if (!state_ok) {
        // Nhắc lại mỗi giây: nếu đã ở IDLE thì EnterIdle im lặng, người vận hành
        // chỉ thấy "robot không nhận lệnh" mà không biết vì sao.
        if (state_fault_log_tick_ == 0) std::cerr << "[Application] " << estimator_.FaultText() << "\n";
        state_fault_log_tick_ = (state_fault_log_tick_ + 1) % 500;
        EnterIdle(estimator_.FaultText());
        return true;
    }
    state_fault_log_tick_ = 0;
    if (!state_fresh_) {
        // Latch every armed state (including Mode Z) into IDLE. Reconnection
        // must not automatically re-energize a stale teleop/policy target.
        if (state_ == AppState::kIdle) sender_.Damping(estimator_.state().mode_machine);
        else EnterIdle("LowState PD watchdog expired");
        return true;
    }
    if (TryDisarm(cmd, low)) return true;

    HandleStopCommands(cmd);
    HandleTeleopToggle(cmd);
    UpdateSafeStop(cmd);

    AnnounceSpeedMode();

    UpdateBattery();

    HandleTransitions(cmd);

    SyncUpperBodyOwnership();

    gait_.Update(spec::kLoopDt);

    RunActiveState(cmd);

    CheckOutputFaults();

    CaptureMimicTelemetry(low);
    UpdateHud();
    if (tick_ % 500 == 0) PrintStatus();
    return true;
}

// Lệnh status/cancel từ voice (socket) được trả lời mọi tick; play xử lý trong RunPolicy.
void Application::ServiceVoiceGestureOwner() {
    voice_gesture_owner_.Poll();
    if (voice_gesture_owner_.HasPending()) {
        const VoiceGestureOwner::Request request = voice_gesture_owner_.Pending();
        if (request.command == "status") {
            const char* status = voice_command_gesture_active_ ? "active"
                : (arm_gesture_.Idle() ? "idle" : "busy");
            voice_gesture_owner_.Reply(true, status, "", voice_command_gesture_id_);
        } else if (request.command == "cancel") {
            if (voice_command_gesture_active_ && !arm_gesture_.Idle()) {
                arm_gesture_.RetractSafety();
                voice_command_retract_requested_ = true;
                voice_gesture_owner_.Reply(true, "cancelling", "",
                                           voice_command_gesture_id_);
            } else {
                ClearVoiceCommandGesture();
                voice_gesture_owner_.Reply(true, "idle");
            }
        } else if (request.command == "play" &&
                   (!tuning_.gesture_enabled || !armed_ ||
                    state_ != AppState::kLocomotion)) {
            voice_gesture_owner_.Reply(false, "rejected", "not_in_locomotion");
        }
    }
}

// E-stop và L2+Y (Zero Torque) — sau watchdog/disarm, trước mọi chuyển state khác.
void Application::HandleStopCommands(const InputCommand& cmd) {
    // Emergency stop (the PD watchdog above covers every armed state).
    if (state_ != AppState::kIdle && state_ != AppState::kWaitingForState &&
        state_ != AppState::kZeroTorque) {
        if (cmd.want_emergency_stop) {
            EnterIdle("EMERGENCY STOP (L2+B / ESC)");
        }
    }

    // Cho phép E-stop thoát Zero Torque về Damping.
    if (state_ == AppState::kZeroTorque && cmd.want_emergency_stop) {
        EnterIdle("EMERGENCY STOP (L2+B) — thoát Zero Torque");
    }

    // L2+Y: Toggle giữa kIdle và kZeroTorque (xả lực hoàn toàn).
    if (cmd.want_zero_torque) {
        if (state_ == AppState::kIdle) {
            EnterZeroTorque("ZERO TORQUE (L2+Y)");
        } else if (state_ == AppState::kZeroTorque) {
            // Thoát Zero Torque về Damping.
            std::cout << "\n[Application] Thoát Zero Torque -> Damping (IDLE).\n";
            state_ = AppState::kIdle;
            input_lost_ = false;
            input_lost_count_ = 0;
            sender_.Damping(estimator_.state().mode_machine);
        }
    }
}

void Application::HandleTeleopToggle(const InputCommand& cmd) {
    // Cạnh nhấn START: bật/tắt UDP overlay tay+đầu.
    if (cmd.want_teleop_toggle) {
        if (!tuning_.teleop_enabled) {
            std::cout << "[Teleop] START bị bỏ qua — teleop_enabled=false trong gestures.yaml.\n";
        } else if (locomotion_ && !locomotion_->SupportsArmOverlay() && !h4_teleop_approved_) {
            std::cout << "[Teleop] START bi tu choi — contract hien tai "
                         "khong ho tro arm overlay va H4 chua qua artifact gate.\n";
        } else if (teleop_runtime_on_) {
            SetTeleopArmed(false, "START");
        } else if (state_ == AppState::kLocomotion && loco_swap_pending_) {
            std::cout << "[Teleop] START bi tu choi — dang doi policy locomotion; "
                         "cho robot dung yen va doi xong.\n";
        } else if (state_ == AppState::kLocomotion && loco_mode_ == LocoMode::kWalk &&
                   (std::fabs(cmd.vx) > 1e-4f || std::fabs(cmd.vy) > 1e-4f ||
                    std::fabs(cmd.yaw) > 1e-4f)) {
            // R2+A dùng walk_policy: chỉ cấp quyền từ trạng thái dừng hẳn và
            // UpdateBaseCommand khóa base suốt phiên teleop.
            std::cout << "[Teleop] START bi tu choi — R2+A yeu cau nha can dieu khien "
                         "ve 0, cho robot dung yen roi nhan START.\n";
        } else if (state_ != AppState::kLocomotion &&
                   !(h4_teleop_approved_ && state_ == AppState::kZeroTorque)) {
            // Teleop là overlay TRÊN policy đi bộ: chân phải đang được policy giữ.
            // Từ chối có tiếng chứ không im lặng, để không bị hiểu là hỏng nút.
            std::cout << "[Teleop] START bi tu choi — phai dang LOCOMOTION (R2+A) "
                         "hoac H4 ZERO TORQUE. Trang thai hien tai: " << StateName() << "\n";
        } else {
            SetTeleopArmed(!teleop_runtime_on_, "START");
        }
    }
}

void Application::AnnounceSpeedMode() {
    // Phát âm thanh khi đổi chế độ tốc độ nhanh/chậm.
    bool now_fast = input_.IsFastMode();
    if (speed_voice_inited_ && now_fast != prev_fast_) {
        const std::string& v = now_fast ? tuning_.voice_fast_speed : tuning_.voice_slow_speed;
        if (!v.empty()) music_.Say(v, tuning_.voice_volume, tuning_.voice_speaker_id);
    }
    prev_fast_ = now_fast;
    speed_voice_inited_ = true;
}

// Sau HandleTransitions: nhả teleop/gesture khi state mới không còn sở hữu thân trên.
void Application::SyncUpperBodyOwnership() {
    // A new LOCOMOTION entry must see a fresh explicit STOP before any H4
    // enable packet can own the upper body, including returns from dance.
    if (h4_teleop_approved_) {
        // Phiên H4 mới mỗi lần vào LOCOMOTION (R2+A hoặc R2+X), từ STAND LOCK
        // hay sau dance: luôn cần STOP tường minh trước gói enable đầu tiên.
        const bool in_locomotion = state_ == AppState::kLocomotion;
        if (in_locomotion && !h4_session_in_locomotion_) {
            teleop_.BeginH4Session();
            h4_history_release_s_ = 0.0f;
            h4_head_release_s_ = 0.0f;
        }
        h4_session_in_locomotion_ = in_locomotion;
    }

    // START arm/disarm H4 ở LOCOMOTION và ZERO TORQUE. Legacy vẫn chỉ thuộc
    // LOCOMOTION; mọi state khác tự nhả quyền để không giữ target cũ.
    if (teleop_runtime_on_ && state_ != AppState::kLocomotion &&
        !(h4_teleop_approved_ && state_ == AppState::kZeroTorque))
        SetTeleopArmed(false, "rời LOCOMOTION");
    // Ngoài locomotion, target tay thuộc về state controller (stand/sit/getup...)
    // và không có policy BlendInto để hoàn tất gesture handover. Hủy quyền overlay
    // để player không giữ pose cũ rồi phát lại khi locomotion được bật lần nữa.
    if (tuning_.gesture_enabled && state_ != AppState::kLocomotion &&
        state_ != AppState::kMimic) {
        arm_gesture_.Cancel();
        voice_gesture_latched_ = false;
        ClearVoiceCommandGesture();
    }
}

void Application::RunActiveState(const InputCommand& cmd) {
    switch (state_) {
        case AppState::kIdle:
            sender_.Damping(estimator_.state().mode_machine);
            break;
        case AppState::kZeroTorque:
            RunZeroTorqueTeleop();
            break;
        case AppState::kStandUp:
            RunStandUp();
            break;
        case AppState::kStandLock:
            RunStandLock();
            break;
        case AppState::kReturningToDefault:
            RunReturning();
            break;
        case AppState::kLocomotion:
        case AppState::kMimic:
            RunPolicy(cmd);
            break;
        case AppState::kSafeShutdown:
            RunSafeShutdown();
            break;
        case AppState::kGetUp:
            RunGetUp();
            break;
        case AppState::kLieDown:
            RunLieDown();
            break;
        default:
            break;
    }
}

void Application::CheckOutputFaults() {
    // Cổng an toàn của LowCmdSender đã phải can thiệp trong tick này: frame vừa
    // gửi đã được làm sạch, nhưng nguồn sinh ra NaN thì vẫn còn -> dừng an toàn.
    if (sender_.HasFault()) {
        const std::string reason = "Cong an toan lowcmd: " + sender_.FaultText();
        sender_.ClearFault();
        EnterIdle(reason);
    }

    // Policy đang chạy đã trả NaN/Inf: target giữ nguyên frame trước nên robot
    // chưa giật, nhưng model đã phân kỳ -> không chạy tiếp.
    if (active_ && active_->infer_fault()) {
        EnterIdle("Policy " + active_->Name() + " tra output khong huu han");
    }
}

void Application::PublishIntegrationStatus(bool force) {
    if (!force && tick_ - integration_last_tick_ < 100) return;
    integration_last_tick_ = tick_;
    integration_notifier_.Send(audio_busy_.load(), armed_.load(), StateName());
}

void Application::UpdateSafeStop(InputCommand& cmd) {
    // Không áp dụng safe-stop cho trạng thái kZeroTorque.
    bool eligible = tuning_.safe_stop_enabled &&
                    state_ != AppState::kIdle && state_ != AppState::kWaitingForState &&
                    state_ != AppState::kZeroTorque;
    if (eligible && !cmd.is_active) input_lost_count_++;
    else                           input_lost_count_ = 0;

    int need = std::max(1, static_cast<int>(
                   tuning_.safe_stop_debounce_ms / 1000.0f / spec::kLoopDt));
    bool lost = input_lost_count_ >= need;

    if (lost && !input_lost_) {
        std::cout << "\n[Application] CONTROL_LOST source=all_input remote_state="
                  << input_.gamepad().LinkStateName()
                  << " action=ZERO_VELOCITY state=" << StateName() << "\n";
        music_.Say(tuning_.voice_safe_stop, tuning_.voice_volume, tuning_.voice_speaker_id);
        input_.ZeroVelocity();
    } else if (!lost && input_lost_) {
        std::cout << "[Application] CONTROL_RECOVERED remote_state="
                  << input_.gamepad().LinkStateName()
                  << " state=" << StateName() << "\n";
    }
    input_lost_ = lost;

    if (input_lost_) {
        cmd.vx = cmd.vy = cmd.yaw = 0.0f;
    }
}

// Đọc thông tin pin và xử lý cảnh báo pin yếu/cạn.
void Application::UpdateBattery() {
    if (!tuning_.battery_monitor_enabled) return;

    // Chỉ hoạt động ở trạng thái đứng/di chuyển/nhảy.
    bool active = (state_ == AppState::kLocomotion || state_ == AppState::kMimic ||
                   state_ == AppState::kStandLock);
    if (!active) { battery_active_ = false; return; }

    // Cho phép cảnh báo ngay khi kích hoạt nếu pin yếu.
    if (!battery_active_) { battery_active_ = true; battery_warn_timer_ = 0.0f; }
    if (battery_warn_timer_ > 0.0f) battery_warn_timer_ -= spec::kLoopDt;

    // Cảnh báo nếu mất tín hiệu pin.
    if (!battery_.Fresh(tuning_.battery_stale_s)) {
        if (battery_warn_timer_ <= 0.0f) {
            std::cout << "[Application] ⚠ Mất tín hiệu pin (rt/lf/bmsstate) — kiểm tra topic!\n";
            battery_warn_timer_ = tuning_.battery_announce_period_s;
        }
        return;
    }

    int soc = battery_.Soc();

    // Tự động ngồi xuống khi pin cạn.
    if (soc <= tuning_.battery_critical_pct && !battery_critical_done_) {
        battery_critical_done_ = true;
        std::cout << "[Application] 🔴 PIN CẠN " << soc << "% -> hành động: "
                  << tuning_.battery_critical_action << "\n";
        if (!tuning_.voice_battery_critical.empty())
            music_.Say(tuning_.voice_battery_critical, tuning_.voice_volume, tuning_.voice_speaker_id);
        if (tuning_.battery_critical_action == "sit") {
            // Ngồi xuống không dùng ghế khi pin cạn.
            if (state_ == AppState::kLocomotion || state_ == AppState::kMimic)
                RequestFromPolicy(PendingAction::kToSit, /*has_chair=*/false);
            else if (state_ == AppState::kStandLock)
                BeginSit(estimator_.state(), /*has_chair=*/false);
        } else if (tuning_.battery_critical_action == "damp") {
            EnterIdle("Pin cạn - xả lực");
        }
        return;
    }

    // Cảnh báo (throttle).
    if (soc <= tuning_.battery_warn_pct && battery_warn_timer_ <= 0.0f) {
        std::cout << "[Application] ⚠ Pin yếu: " << soc << "%\n";
        if (!tuning_.voice_battery_low.empty())
            music_.Say(tuning_.voice_battery_low, tuning_.voice_volume, tuning_.voice_speaker_id);
        battery_warn_timer_ = tuning_.battery_announce_period_s;
    }
}

void Application::ActivatePolicy(PolicyController* ctrl) {
    // Trả trim locomotion về ĐÂY, không rải ở từng call-site: có 4 đường vào
    // locomotion (stand-lock, dance biến mất, đứng dậy sau ngã, bàn giao hết bài)
    // và trước đây chỉ đường bàn giao khôi phục trim -> ngã giữa dance rồi đứng
    // dậy là locomotion chạy tiếp bằng trim của bài dance đó.
    // Đặt TRƯỚC Reset() để policy nhận đúng quaternion ngay frame đầu.
    // Mimic tự nạp dance_trim_deg_N của nó ngay trước khi gọi hàm này.
    if (ctrl == walk_ctrl_.get() || (teleop_ctrl_ && ctrl == teleop_ctrl_.get()))
        estimator_.SetPitchTrimDeg(tuning_.LocomotionTrimDeg());
    ctrl->Reset(estimator_.state());
    active_ = ctrl;
    lying_ = false;
    fall_detector_.Reset();
    blend_alpha_ = 0.0f;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        blend_start_q_[i] = sender_.last_cmd_q_policy(i);
        ai_target_q_[i] = blend_start_q_[i];
    }
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
}

void Application::EnterIdle(const std::string& reason) {
    if (state_ == AppState::kIdle) return;
    std::cout << "\n[Application] " << reason << " -> Damping (IDLE).\n";
    music_.Stop();
    music_.Say(tuning_.voice_safe_stop, tuning_.voice_volume, tuning_.voice_speaker_id);
    state_ = AppState::kIdle;
    ClearVoiceCommandGesture();
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    dance_stopping_ = false;
    // Sau IDLE (E-stop, ngã, mất input) luôn quay về chế độ đi bộ thường;
    // teleop chỉ được cấp lại bằng START trong mode locomotion phù hợp.
    loco_swap_pending_ = false;
    loco_mode_ = LocoMode::kWalk;
    locomotion_ = walk_ctrl_.get();
    input_lost_ = false;
    input_lost_count_ = 0;
    sender_.ClearNonTeleopHold();
    sender_.Damping(estimator_.state().mode_machine);
}

void Application::EnterZeroTorque(const std::string& reason) {
    std::cout << "\n[Application] " << reason << " -> ZERO TORQUE (limp).\n";
    music_.Stop();
    if (!tuning_.voice_zero_torque.empty())
        music_.Say(tuning_.voice_zero_torque, tuning_.voice_volume, tuning_.voice_speaker_id);
    state_ = AppState::kZeroTorque;
    ClearVoiceCommandGesture();
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    dance_stopping_ = false;
    input_lost_ = false;
    input_lost_count_ = 0;
    sender_.ClearNonTeleopHold();
    // Chốt encoder làm gốc trước khi Mode Z có thể thấy một gói UDP đã chờ sẵn.
    // Nếu bỏ bước này, slew có thể bắt đầu từ lệnh policy cũ thay vì tư thế thật.
    SyncSenderToLatestState();
    sender_.ZeroTorque(estimator_.state().mode_machine);
}

std::string Application::StateName() const {
    switch (state_) {
        case AppState::kDisarmed: return "DISARMED (chờ bàn giao)";
        case AppState::kConflict: return "CONFLICT (đã nhả quyền)";
        case AppState::kWaitingForState: return "WAITING FOR STATE";
        case AppState::kIdle: return "IDLE (damping)";
        case AppState::kZeroTorque: return "ZERO TORQUE (limp)";
        case AppState::kStandUp: return "STANDING UP";
        case AppState::kStandLock: return "STAND LOCK";
        case AppState::kLocomotion: return "LOCOMOTION";
        case AppState::kReturningToDefault: return "RETURNING";
        case AppState::kMimic: return "MIMIC (dance)";
        case AppState::kSafeShutdown: return "SAFE SHUTDOWN";
        case AppState::kGetUp:
            return traj_phase_ == 0 ? "GET UP (vào tư thế nằm-chuẩn)"
                 : traj_phase_ == 1 ? "GET UP (sẵn sàng - bấm L2+X)"
                                    : "GET UP (đang đứng dậy)";
        case AppState::kLieDown:
            return "LIE DOWN (đang nằm xuống)";
    }
    return "?";
}

void Application::UpdateHud() {
    std::vector<std::string> hints;
    switch (state_) {
        case AppState::kDisarmed:
            hints = {"L2+R2: built-in dev mode", "rồi giữ R1+R2 để run_r1 tiếp quản"};
            break;
        case AppState::kConflict:
            hints = {"Built-in quay lại - đã nhả quyền", "Bấm E-stop cứng nếu robot loạn"};
            break;
        case AppState::kIdle:
            hints = RobotLying(estimator_.state())
                ? std::vector<std::string>{"Robot đang NẰM (IMU)", "L2+X: Chuẩn bị đứng dậy (bấm 2 lần)"}
                : std::vector<std::string>{"L2+Lên: STAND LOCK", "L2+Y: Zero Torque (limp)"};
            break;
        case AppState::kZeroTorque:
            hints = {"Motor XA LUC HOAN TOAN (limp)", "L2+Y: Thoat ve Damping",
                     "Disarm: treo/do robot, nha Quest + nut, giu R1+R2",
                     RobotLying(estimator_.state()) ? "L2+X: Chuẩn bị đứng dậy"
                                                    : "L2+Lên: Đứng dậy gồng cứng"};
            break;
        case AppState::kStandLock:
            hints = {"1 / R2+A : ĐI BỘ A | T / R2+X : ĐI BỘ X",
                     "L2+Trái : Ngồi ghế", "L2+X : Nằm xuống"};
            break;
        case AppState::kLocomotion: {
            std::string dance_keys = mimics_.empty() ? "" : "2-8 : DANCE | ";
            hints = {std::string("Che do: ") + LocoModeName(loco_mode_) +
                         (loco_swap_pending_ ? std::string(" -> ") + LocoModeName(loco_swap_target_) +
                                                   " (dung yen...)"
                                             : std::string()) +
                         (loco_mode_ == LocoMode::kTeleop
                              ? " | START: tay teleop + di chuyen"
                              : " | START: tay teleop (can dieu khien = 0)"),
                     "W/S/A/D : Move | Q/E : Yaw | 1/T : doi che do",
                     "Tab : speed | R : Reset policy",
                     dance_keys + "0 : STAND LOCK | 9 : Sit down",
                     input_.IsFastMode() ? "Mode: FAST" : "Mode: SLOW"};
            break;
        }
        case AppState::kMimic:
            hints = {"Dancing - returns to LOCOMOTION when done",
                     "0 : STAND LOCK | 9 : Sit down"};
            break;
        case AppState::kSafeShutdown:
            hints = {"Sitting pose active",
                     "0 / L2+Up: Stand up | ESC / L2+B: Damp"};
            break;
        case AppState::kGetUp:
            hints = traj_phase_ == 1
                ? std::vector<std::string>{"Sẵn sàng đứng dậy - bấm L2+X", "L2+B: Khẩn cấp"}
                : traj_phase_ == 0
                    ? std::vector<std::string>{"Đang vào tư thế nằm-chuẩn...", "L2+B: Khẩn cấp"}
                    : std::vector<std::string>{"Đang đứng dậy (getup.npz) -> Locomotion", "L2+B: Khẩn cấp"};
            break;
        case AppState::kLieDown:
            hints = {"Đang nằm xuống (liedown.npz)...", "L2+B: Khẩn cấp"};
            break;
        default:
            break;
    }
    input_.keyboard().SetHud("[ " + StateName() + " ]", hints);
}

void Application::PrintStatus() {
    if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
        printf("[%s|%s%s] vx=%.2f vy=%.2f yaw=%.2f %s\n", StateName().c_str(),
               LocoModeName(loco_mode_), loco_swap_pending_ ? " swap..." : "", cmd_vx_,
               cmd_vy_, cmd_yaw_, input_.IsFastMode() ? "(FAST)" : "(SLOW)");
        if (state_ == AppState::kLocomotion) {
            const Eigen::Vector3f& g = estimator_.state().projected_gravity;
            const float rad2deg = 180.0f / static_cast<float>(M_PI);
            float pitch_deg = std::atan2(-g.x(), -g.z()) * rad2deg;
            float roll_deg  = std::atan2(-g.y(), -g.z()) * rad2deg;
            printf("  [tilt] pitch=%+.2f deg roll=%+.2f deg grav=(%+.3f,%+.3f,%+.3f) |gyro|=%.2f\n",
                   pitch_deg, roll_deg, g.x(), g.y(), g.z(),
                   estimator_.state().gyro.norm());
        }
    } else if (state_ == AppState::kZeroTorque) {
        // Hiển thị góc đo thực tế khi robot ở trạng thái limp (Zero Torque).
        const auto& q = estimator_.state().q;
        const float r2d = 180.0f / static_cast<float>(M_PI);
        printf("[ZERO TORQUE] goc chan (do):\n");
        printf("  L: hipP=%+.1f hipR=%+.1f hipY=%+.1f knee=%+.1f ankP=%+.1f ankR=%+.1f\n",
               q[0]*r2d, q[1]*r2d, q[2]*r2d, q[3]*r2d, q[4]*r2d, q[5]*r2d);
        printf("  R: hipP=%+.1f hipR=%+.1f hipY=%+.1f knee=%+.1f ankP=%+.1f ankR=%+.1f\n",
               q[6]*r2d, q[7]*r2d, q[8]*r2d, q[9]*r2d, q[10]*r2d, q[11]*r2d);
        // Hiển thị góc tay (rad).
        printf("  L tay(rad): shP=%+.2f shR=%+.2f shY=%+.2f elb=%+.2f wr=%+.2f\n",
               q[14], q[15], q[16], q[17], q[18]);
        printf("  R tay(rad): shP=%+.2f shR=%+.2f shY=%+.2f elb=%+.2f wr=%+.2f\n",
               q[19], q[20], q[21], q[22], q[23]);
    } else {
        printf("[%s]\n", StateName().c_str());
    }
    fflush(stdout);
}
