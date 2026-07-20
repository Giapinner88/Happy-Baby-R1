#include "Application.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

#include <dirent.h>

#include <unitree/robot/channel/channel_factory.hpp>

using unitree_hg::msg::dds_::LowState_;

namespace {

// Đuôi file có đuôi khớp `ext` (không phân biệt hoa thường)
bool HasExt(const std::string& name, const std::string& ext) {
    if (name.size() < ext.size()) return false;
    std::string tail = name.substr(name.size() - ext.size());
    std::transform(tail.begin(), tail.end(), tail.begin(), ::tolower);
    return tail == ext;
}

// Quét folder tìm file .onnx, .npz và nhạc. Trả về file đầu tiên khớp.
void ScanDanceFolder(const std::string& dir, const std::string& label,
                     std::string& onnx, std::string& npz, std::string& music) {
    static const char* kAudioExt[] = {".mp3", ".wav", ".ogg", ".flac", ".m4a"};
    onnx.clear(); npz.clear(); music.clear();
    DIR* d = opendir(dir.c_str());
    if (!d) {
        std::cerr << "[Application] Không mở được folder điệu " << label << ": " << dir << "\n";
        return;
    }
    for (dirent* e = readdir(d); e; e = readdir(d)) {
        std::string name = e->d_name;
        if (name == "." || name == "..") continue;
        auto pick = [&](std::string& slot) {
            if (slot.empty()) slot = name;
            else std::cerr << "[Application] Điệu " << label << ": nhiều file cùng loại, "
                           << "giữ '" << slot << "', bỏ qua '" << name << "'.\n";
        };
        if (HasExt(name, ".onnx")) pick(onnx);
        else if (HasExt(name, ".npz")) pick(npz);
        else {
            for (const char* ax : kAudioExt) {
                if (HasExt(name, ax)) { pick(music); break; }
            }
        }
    }
    closedir(d);
}

} // namespace

Application::Application(std::string proj_dir, std::string interface_override)
    : proj_dir_(std::move(proj_dir)) {
    tuning_.LoadFromFile(proj_dir_ + "/config/tuning.yaml");
    if (!interface_override.empty()) tuning_.network_interface = interface_override;

    // Gains khi không chạy policy
    for (int i = 0; i < spec::kNumJoints; ++i) {
        bool is_arm = (i >= 14);
        bool is_waist = (i == 12 || i == 13);
        stand_gains_kp_[i] = is_arm ? tuning_.stand_kp_arm
                            : is_waist ? tuning_.stand_kp_waist
                                       : tuning_.stand_kp_leg;
        stand_gains_kd_[i] = tuning_.stand_kd;

        // Gains riêng lúc ngồi
        sit_gains_kp_[i] = is_arm ? tuning_.stand_kp_arm : tuning_.sit_kp_leg;
        sit_gains_kd_[i] = tuning_.sit_kd;
    }

    ort_opts_.SetIntraOpNumThreads(1);
    ort_opts_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
}

void Application::InitDds() {
    // CycloneDDS config
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

    // Loa robot qua Unitree "voice" service
    if (tuning_.voice_enabled || !music_files_.empty()) {
        music_.InitAudio();
        std::cout << "[Application] AudioClient (loa robot): "
                  << (music_.ready() ? "sẵn sàng" : "KHÔNG sẵn sàng -> tắt audio")
                  << " (voice=" << (tuning_.voice_enabled ? "bật" : "tắt")
                  << ", " << music_files_.size() << " điệu có nhạc)\n";
    }
}

void Application::OnLowState(const void* msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    shared_low_state_ = *static_cast<const LowState_*>(msg);
    got_state_ = true;
    last_state_time_ = std::chrono::steady_clock::now();
}

void Application::InitControllers() {
    locomotion_ = std::make_unique<LocomotionController>();
    locomotion_->Init(proj_dir_ + "/policies/flat/" + tuning_.flat_model,
                      ort_env_, ort_opts_, tuning_);

    for (int i = 0; i < Tuning::kMaxDances; ++i) {
        if (tuning_.dance_folder[i].empty()) continue;
        int key = i + 2; // key 2..8
        std::string dir = proj_dir_ + "/policies/dance/" + tuning_.dance_folder[i];

        std::string onnx, npz, music;
        ScanDanceFolder(dir, std::to_string(key), onnx, npz, music);
        if (onnx.empty() || npz.empty()) {
            std::cerr << "[Application] Điệu " << key << " (" << tuning_.dance_folder[i]
                      << "): thiếu " << (onnx.empty() ? ".onnx " : "")
                      << (npz.empty() ? ".npz " : "") << "-> BỎ QUA.\n";
            continue;
        }

        motions_[key] = std::make_unique<MotionData>();
        motions_[key]->Load(dir + "/" + npz);

        int start_frame = tuning_.dance_start_frame;
        if (start_frame < 0) {
            start_frame = motions_[key]->FindSmoothStartFrame(tuning_.dance_start_search_frames);
            std::cout << "[Application] Dance " << key << " start_frame=-1 -> chọn frame "
                      << start_frame << "\n";
        }

        mimics_[key] = std::make_unique<MimicController>(*motions_[key], start_frame, tuning_.dance_speed[i]);
        mimics_[key]->Init(dir + "/" + onnx, ort_env_, ort_opts_, tuning_);

        if (!music.empty()) music_files_[key] = dir + "/" + music;
        std::cout << "[Application] Đã tải Mimic " << key << " (" << tuning_.dance_folder[i] << ") "
                  << "onnx=" << onnx << " npz=" << npz
                  << (music.empty() ? " (không nhạc)" : " nhạc=" + music)
                  << " speed=" << tuning_.dance_speed[i]
                  << " volume=" << tuning_.dance_volume[i] << "\n";
    }
    active_ = locomotion_.get();
}

int Application::Run() {
    std::cout << "====================================\n"
              << " HB R1 High-Level Runner\n"
              << " Interface : " << tuning_.network_interface << "\n"
              << "====================================\n";

    InitControllers();
    InitDds();

    estimator_.Configure(tuning_, spec::kLoopDt);
    fall_detector_.Configure(tuning_, spec::kLoopDt);
    input_.Configure(tuning_);

    // Đứt SSH -> dập motor ngay
    KeyboardX11::SetEmergencyDampFn([this]() {
        sender_.Damping();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });
    input_.Start();

    std::cout << "[Application] Chờ kết nối DDS (rt/lowstate)...\n";

    auto next_wake = std::chrono::steady_clock::now();
    const auto period = std::chrono::microseconds(2000); // 500Hz

    while (running_) {
        if (!Tick()) break;
        next_wake += period;
        auto now = std::chrono::steady_clock::now();
        if (next_wake < now) next_wake = now;
        std::this_thread::sleep_until(next_wake);
        ++tick_;
    }

    std::cout << "\n[Application] Đang thoát — xả motor...\n";
    sender_.Damping();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    input_.Stop();
    std::cout << "[Application] Đã thoát an toàn.\n";
    return 0;
}

bool Application::Tick() {
    // ── 1. Đọc state ─────────────────────────────────────
    LowState_ low;
    bool got;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        low = shared_low_state_;
        got = got_state_;
    }

    if (state_ == AppState::kWaitingForState) {
        if (!got) {
            if (input_.keyboard().WantExit()) {
                running_ = false;
                return false;
            }
            return true;
        }
        std::cout << "[Application] Đã kết nối robot — đang thả lỏng (IDLE).\n"
                  << "  Bàn phím : 0=Stand Lock, 1=Đi bộ, 2-8=Dance (mimic), Tab=Tốc độ,\n"
                  << "             R=Reset, 9=Ngồi xuống an toàn\n"
                  << "  R3-1     : L2+Lên=Stand Lock, R2+A=Đi bộ, R1+D-pad/A=Dance,\n"
                  << "             R2+Lên/Xuống=Nhanh/Chậm, L2+X=Ngồi xuống,\n"
                  << "             L2+B=KHẨN CẤP\n";
        sender_.SyncToState(low);
        estimator_.Reset();
        state_ = AppState::kIdle;
        // Voice: khởi động thành công
        music_.Say(tuning_.voice_startup, tuning_.voice_volume, tuning_.voice_speaker_id);
    }

    estimator_.Update(low);

    // ── 2. Input ─────────────────────────────────────────
    input_.Update(low);
    InputCommand cmd = input_.GetMergedCommand();

    if (input_.keyboard().WantExit()) {
        EnterIdle("ESC — dừng khẩn cấp và thoát");
        running_ = false;
        return false;
    }

    // ── 3. Watchdog ──────────────────────────────────────
    if (state_ != AppState::kIdle && state_ != AppState::kWaitingForState) {
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - last_state_time_).count();
        if (ms > static_cast<long>(tuning_.state_timeout_ms)) {
            EnterIdle("Mất tín hiệu lowstate > " +
                      std::to_string(static_cast<int>(tuning_.state_timeout_ms)) + "ms");
        } else if (cmd.want_emergency_stop) {
            EnterIdle("DỪNG KHẨN CẤP (L2+B / ESC)");
        }
    }

    // ── 3b. Safe-stop khi mất input ──────────────────────
    UpdateSafeStop(cmd);

    // ── 4. Chuyển trạng thái ─────────────────────────────
    HandleTransitions(cmd);

    // ── 5. Gait clock ────────────────────────────────────
    gait_.Update(spec::kLoopDt);

    // ── 6. Hành động theo trạng thái ─────────────────────
    switch (state_) {
        case AppState::kIdle:
            sender_.Damping();
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

        default:
            break;
    }

    UpdateHud();
    if (tick_ % 500 == 0) PrintStatus();
    return true;
}

void Application::RequestFromPolicy(PendingAction act) {
    if (pending_action_ == act) return;

    if (state_ == AppState::kMimic) {
        std::cout << "[Application] Dừng điệu nhảy — về policy đi bộ để dừng an toàn.\n";
        music_.Stop();
        ActivatePolicy(locomotion_.get());
        state_ = AppState::kLocomotion;
    }
    announcing_ = false;
    pending_action_ = act;
    settle_timer_ = 0.0f;
    input_.ZeroVelocity();
    std::cout << "[Application] "
              << (act == PendingAction::kToStandLock ? "KHÓA ĐỨNG" : "NGỒI GHẾ")
              << " — cho robot DỪNG LẠI dưới policy trước (vẫn cân bằng)...\n";
}

void Application::BeginStandUp(const RobotState& rs) {
    std::cout << "[Application] Bắt đầu đứng dậy về STAND LOCK.\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    stand_start_q_ = rs.q;
    stand_timer_ = 0.0f;
    // Re-anchor rate limiter
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kStandUp;
}

void Application::BeginSit(const RobotState& rs) {
    std::cout << "[Application] Ngồi xuống ghế (feet-flat)...\n";
    music_.Stop();
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    music_.Say(tuning_.voice_sit_down, tuning_.voice_volume, tuning_.voice_speaker_id);
    sit_start_q_ = rs.q;
    sit_timer_ = 0.0f;
    sit_phase_ = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kSafeShutdown;
}

void Application::HandleTransitions(const InputCommand& cmd) {
    const RobotState& rs = estimator_.state();

    // ── Bấm KHÓA ĐỨNG ────────────────────────────────────
    if (cmd.want_stand_lock) {
        if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
            RequestFromPolicy(PendingAction::kToStandLock);
        } else if (state_ == AppState::kIdle || state_ == AppState::kSafeShutdown) {
            BeginStandUp(rs);
            return;
        }
    }

    // ── Bấm NGỒI GHẾ ─────────────────────────────────────
    if (cmd.want_safe_shutdown) {
        if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
            RequestFromPolicy(PendingAction::kToSit);
        } else if (state_ == AppState::kStandLock) {
            BeginSit(rs);
            return;
        }
    }

    // ── Đang chờ robot dừng ──────────────────────────────
    if (pending_action_ != PendingAction::kNone && state_ == AppState::kLocomotion) {
        settle_timer_ += spec::kLoopDt;
        float min_t  = std::max(0.0f, tuning_.settle_time_s);
        bool  quiet  = rs.gyro.norm() <= tuning_.settle_gyro_max;
        bool  capped = settle_timer_ >= min_t * 3.0f + 0.5f;
        if (settle_timer_ >= min_t && (quiet || capped)) {
            if (!quiet)
                std::cout << "[Application] ⚠ Robot chưa thật yên (|gyro|="
                          << rs.gyro.norm() << ") nhưng hết giờ chờ -> vẫn bàn giao.\n";
            PendingAction act = pending_action_;
            pending_action_ = PendingAction::kNone;
            if (act == PendingAction::kToStandLock) BeginStandUp(rs);
            else                                    BeginSit(rs);
        }
        return;
    }

    // -> LOCOMOTION (chỉ từ STAND_LOCK)
    if (cmd.want_locomotion) {
        if (state_ == AppState::kStandLock) {
            std::cout << "[Application] Bật LOCOMOTION — sẵn sàng đi bộ.\n";
            ActivatePolicy(locomotion_.get());
            state_ = AppState::kLocomotion;
            music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
        } else if (state_ != AppState::kLocomotion) {
            std::cout << "[Application] TỪ CHỐI: phải ở STAND_LOCK mới bật được policy!\n";
        }
    }

    // -> MIMIC
    if (cmd.want_mimic_key && state_ == AppState::kLocomotion) {
        if (mimics_.find(cmd.want_mimic_key) != mimics_.end()) {
            std::cout << "[Application] Đọc tên điệu " << cmd.want_mimic_key
                      << " — GIỮ LOCOMOTION (đứng yên tại chỗ, vẫn cân bằng)...\n";
            pending_dance_key_ = cmd.want_mimic_key;
            announcing_ = true;
            announce_timer_ = 0.0f;
            input_.ZeroVelocity();
            int vidx = cmd.want_mimic_key - 2;
                music_.Say(tuning_.voice_mimic[vidx], tuning_.voice_volume, tuning_.voice_speaker_id);
        } else {
            std::cout << "[Application] Mimic " << cmd.want_mimic_key << " chưa được cấu hình.\n";
        }
    }

    // Đọc xong -> nhảy
    if (announcing_ && state_ == AppState::kLocomotion) {
        announce_timer_ += spec::kLoopDt;
        if (announce_timer_ >= std::max(0.0f, tuning_.mimic_announce_delay_s)) {
            announcing_ = false;
            std::cout << "[Application] Đọc xong — về default rồi nhảy DANCE "
                      << pending_dance_key_ << "...\n";
            state_ = AppState::kReturningToDefault;
        }
    }

    // Reset policy
    if (cmd.want_reset &&
        (state_ == AppState::kLocomotion || state_ == AppState::kMimic)) {
        std::cout << "[Application] Reset controller: " << active_->Name() << "\n";
        active_->Reset(rs);
    }
}

void Application::UpdateSafeStop(InputCommand& cmd) {
    bool eligible = tuning_.safe_stop_enabled &&
                    state_ != AppState::kIdle && state_ != AppState::kWaitingForState;
    if (eligible && !cmd.is_active) input_lost_count_++;
    else                           input_lost_count_ = 0;

    int need = std::max(1, static_cast<int>(
                   tuning_.safe_stop_debounce_ms / 1000.0f / spec::kLoopDt));
    bool lost = input_lost_count_ >= need;

    if (lost && !input_lost_) {
        std::cout << "\n[Application] MẤT TOÀN BỘ ĐIỀU KHIỂN (gamepad + bàn phím) "
                     "-> GIỮ TRẠNG THÁI (safe-stop, KHÔNG damp).\n";
        music_.Say(tuning_.voice_safe_stop, tuning_.voice_volume, tuning_.voice_speaker_id);
        input_.ZeroVelocity();
    } else if (!lost && input_lost_) {
        std::cout << "[Application] Có lại tín hiệu điều khiển -> tiếp tục bình thường.\n";
    }
    input_lost_ = lost;

    if (input_lost_) {
        cmd.vx = cmd.vy = cmd.yaw = 0.0f;
    }
}

void Application::RunStandUp() {
    stand_timer_ += spec::kLoopDt;
    float progress = std::min(1.0f, stand_timer_ / tuning_.stand_up_time_s);

    const auto& default_q = locomotion_->default_q();
    std::array<float, spec::kNumJoints> target;
    for (int i = 0; i < spec::kNumJoints; ++i)
        target[i] = stand_start_q_[i] + progress * (default_q[i] - stand_start_q_[i]);

    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.stand_rate_limit, estimator_.state().mode_machine);

    if (progress >= 1.0f) {
        std::cout << "[Application] Đã đứng vững — STAND LOCK (chưa chạy policy).\n";
        state_ = AppState::kStandLock;
        music_.Say(tuning_.voice_stand_lock, tuning_.voice_volume, tuning_.voice_speaker_id);
    }
}

void Application::RunStandLock() {
    std::array<float, spec::kNumJoints> target = locomotion_->default_q();
    // Dang 2 hip_roll ra một chút cho vững; rate rất chậm -> dang từ từ
    target[1] += tuning_.stand_lock_spread;
    target[7] -= tuning_.stand_lock_spread;
    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.lock_rate_limit, estimator_.state().mode_machine);
}

void Application::RunReturning() {
    const RobotState& rs = estimator_.state();
    const auto& default_q = locomotion_->default_q();

    // ── Bảo vệ NGÃ ───────────────────────────────────────
    std::vector<std::string> reasons;
    if (fall_detector_.Check(rs.projected_gravity, rs.gyro, reasons)) {
        std::cout << "\n[Application] !!! PHÁT HIỆN NGÃ (lúc về tư thế chuẩn) !!!\n";
        for (const auto& r : reasons) std::cout << "  - " << r << "\n";
        EnterIdle("Ngã khi về tư thế chuẩn — xả lực bảo vệ robot");
        return;
    }

    sender_.Send(default_q, stand_gains_kp_, stand_gains_kd_,
                 tuning_.return_rate_limit, rs.mode_machine);

    bool near = true;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        if (std::abs(rs.q[i] - default_q[i]) > tuning_.return_pos_tol) {
            near = false;
            break;
        }
    }
    if (!near) return;

    std::cout << "[Application] Đã về default — bật MIMIC " << pending_dance_key_ << ".\n";
    ActivatePolicy(mimics_[pending_dance_key_].get());
    // Phát nhạc của bài (nếu có) đúng lúc điệu bắt đầu
    auto it = music_files_.find(pending_dance_key_);
    if (it != music_files_.end())
        music_.Play(it->second, tuning_.dance_volume[pending_dance_key_ - 2]);
    state_ = AppState::kMimic;
}

void Application::RunSafeShutdown() {
    const auto& default_q = locomotion_->default_q();
    float knee_t = std::clamp(tuning_.sit_knee_deg * static_cast<float>(M_PI) / 180.0f,
                              0.3f, 2.42f);
    float stance = std::max(0.0f, tuning_.sit_stance); // chỉ cho khép (≥0), không xòe

    std::array<float, spec::kNumJoints> gather = default_q;
    gather[1] = -stance;
    gather[7] =  stance;

    std::array<float, spec::kNumJoints> seated = gather;
    seated[0] = seated[6] = -knee_t;
    seated[3] = seated[9] =  knee_t;

    sit_timer_ += spec::kLoopDt;
    float Tg = std::max(0.1f, tuning_.sit_gather_time_s);
    float Td = std::max(0.1f, tuning_.sit_descent_time_s);

    std::array<float, spec::kNumJoints> target;
    if (sit_phase_ == 0) {                      // THU CHÂN: start -> gather
        float s = std::min(1.0f, sit_timer_ / Tg);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = sit_start_q_[i] + s * (gather[i] - sit_start_q_[i]);
        if (sit_timer_ >= Tg) { sit_phase_ = 1; sit_timer_ = 0.0f; }
    } else if (sit_phase_ == 1) {               // HẠ XUỐNG: gather -> seated
        float s = std::min(1.0f, sit_timer_ / Td);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = gather[i] + s * (seated[i] - gather[i]);
        if (sit_timer_ >= Td) { sit_phase_ = 2; sit_timer_ = 0.0f; }
    } else {                                    // GIỮ tư thế ngồi (pha 2/3)
        target = seated;
    }

    target[4]  = -(target[0] + target[3]);
    target[10] = -(target[6] + target[9]);
    if (tuning_.sit_lateral_flat) {
        target[5]  = -target[1];
        target[11] = -target[7];
    }

    sender_.Send(target, sit_gains_kp_, sit_gains_kd_,
                 tuning_.sit_rate_limit, estimator_.state().mode_machine);

    if (tick_ % 250 == 0) {
        const char* pha = sit_phase_ == 0 ? "THU CHAN"
                        : sit_phase_ == 1 ? "HA XUONG" : "GIU";
        float Tref = sit_phase_ == 0 ? Tg : sit_phase_ == 1 ? Td : tuning_.sit_hold_s;
        printf("  [ngoi ghe] pha=%s t=%.1f/%.1fs knee=%.0f° stance=%.2f feet-flat\n",
               pha, sit_timer_, Tref, tuning_.sit_knee_deg, stance);
        fflush(stdout);
    }

    if (sit_phase_ == 2 && sit_timer_ >= std::max(0.0f, tuning_.sit_hold_s)) {
        if (tuning_.sit_release_after) {
            std::cout << "[Application] Ngồi ghế xong — xả lực.\n";
            EnterIdle("Ngồi ghế hoàn tất (xả lực)");
        } else {
            sit_phase_ = 3;
            std::cout << "[Application] Ngồi ghế hoàn tất — GIỮ tư thế\n";
        }
    }
}

void Application::RunPolicy(const InputCommand& cmd) {
    const RobotState& rs = estimator_.state();

    // ── Fall detector (mỗi tick 500Hz, có debounce) ──────
    std::vector<std::string> reasons;
    if (fall_detector_.Check(rs.projected_gravity, rs.gyro, reasons)) {
        std::cout << "\n[Application] !!! PHÁT HIỆN NGÃ !!!\n";
        for (const auto& r : reasons) std::cout << "  - " << r << "\n";
        EnterIdle("Ngã — xả lực bảo vệ robot");
        return;
    }

    // ── Lệnh vận tốc (chỉ locomotion) ────────────────────
    bool holding = announcing_ || pending_action_ != PendingAction::kNone;
    if (state_ == AppState::kLocomotion && !holding) {
        cmd_vx_ = cmd.vx;
        cmd_vy_ = cmd.vy;
        cmd_yaw_ = cmd.yaw;
    } else {
        cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    }

    // ── Inference 50Hz ───────────────────────────────────
    if (tick_ % spec::kPolicyDecimation == 0) {
        float cmd_norm = std::sqrt(cmd_vx_ * cmd_vx_ + cmd_vy_ * cmd_vy_ +
                                   cmd_yaw_ * cmd_yaw_);
        ControlContext ctx{rs, cmd_vx_, cmd_vy_, cmd_yaw_, gait_.PhaseObs(cmd_norm)};
        ai_target_q_ = active_->Step(ctx);

        // Blend tư thế khi mới bật policy (chống nhảy bậc target)
        if (blend_alpha_ < 1.0f) {
            blend_alpha_ = std::min(1.0f, blend_alpha_ + spec::kPolicyDt / tuning_.blend_time_s);
            for (int i = 0; i < spec::kNumJoints; ++i) {
                ai_target_q_[i] = (1.0f - blend_alpha_) * blend_start_q_[i] +
                                  blend_alpha_ * ai_target_q_[i];
            }
        }
    }

    // ── Gửi lệnh 500Hz (ZOH giữa các tick inference) ─────
    sender_.Send(ai_target_q_, active_->kp(), active_->kd(),
                 tuning_.policy_rate_limit, rs.mode_machine);

    // ── Mimic xong -> tự về Locomotion ───────────────────
    if (state_ == AppState::kMimic && mimics_[pending_dance_key_]->IsFinished()) {
        std::cout << "[Application] Dance xong — tự động về LOCOMOTION.\n";
        music_.Stop(); // tắt nhạc ngay khi điệu kết thúc
        ActivatePolicy(locomotion_.get());
        state_ = AppState::kLocomotion;
    }
}

void Application::ActivatePolicy(PolicyController* ctrl) {
    ctrl->Reset(estimator_.state());
    active_ = ctrl;
    fall_detector_.Reset();
    blend_alpha_ = 0.0f;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        blend_start_q_[i] = sender_.last_cmd_q_policy(i);
        ai_target_q_[i] = blend_start_q_[i]; // trước tick inference đầu tiên
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
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    input_lost_ = false;
    input_lost_count_ = 0;
    sender_.Damping();
}

std::string Application::StateName() const {
    switch (state_) {
        case AppState::kWaitingForState: return "CHO KET NOI";
        case AppState::kIdle: return "IDLE (tha long)";
        case AppState::kStandUp: return "DANG DUNG DAY";
        case AppState::kStandLock: return "STAND LOCK";
        case AppState::kLocomotion: return "LOCOMOTION (di bo)";
        case AppState::kReturningToDefault: return "RETURNING";
        case AppState::kMimic: return "MIMIC (dance)";
        case AppState::kSafeShutdown: return "NGOI XUONG AN TOAN";

    }
    return "?";
}

void Application::UpdateHud() {
    std::vector<std::string> hints;
    switch (state_) {
        case AppState::kIdle:
            hints = {"0 : Gong cung robot (STAND LOCK)"};
            break;
        case AppState::kStandLock:
            hints = {"1 : Bat policy di bo (LOCOMOTION)",
                     "9 : Ngoi xuong an toan"};
            break;
        case AppState::kLocomotion: {
            std::string dance_keys = mimics_.empty() ? "" : "2-8 : Nhay DANCE | ";
            hints = {"W/S/A/D : Di chuyen | Q/E : Xoay",
                     "Tab : Doi toc do | R : Reset policy",
                     dance_keys + "0 : Ve STAND LOCK | 9 : Ngoi xuong",
                     input_.IsFastMode() ? "Toc do: NHANH" : "Toc do: CHAM"};
            break;
        }
        case AppState::kMimic:
            hints = {"Dang nhay — xong tu ve LOCOMOTION",
                     "0 : Ve STAND LOCK | 9 : Ngoi xuong an toan"};
            break;
        case AppState::kSafeShutdown:
            hints = {"Ngoi ghe: ha thang xuong (feet-flat) roi GIU tu the",
                     "0 / L2+Len: dung day lai | ESC / L2+B: xa luc"};
            break;

        default:
            break;
    }
    input_.keyboard().SetHud("[ " + StateName() + " ]", hints);
}

void Application::PrintStatus() {
    if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
        printf("[%s] vx=%.2f vy=%.2f yaw=%.2f %s\n", StateName().c_str(), cmd_vx_,
               cmd_vy_, cmd_yaw_, input_.IsFastMode() ? "(NHANH)" : "(CHAM)");
        // Log chẩn đoán tilt (1Hz)
        if (state_ == AppState::kLocomotion) {
            const Eigen::Vector3f& g = estimator_.state().projected_gravity;
            const float rad2deg = 180.0f / static_cast<float>(M_PI);
            float pitch_deg = std::atan2(-g.x(), -g.z()) * rad2deg;
            float roll_deg  = std::atan2(-g.y(), -g.z()) * rad2deg;
            printf("  [tilt] pitch=%+.2f° roll=%+.2f° grav=(%+.3f,%+.3f,%+.3f) |gyro|=%.2f\n",
                   pitch_deg, roll_deg, g.x(), g.y(), g.z(),
                   estimator_.state().gyro.norm());
        }
    } else {
        printf("[%s]\n", StateName().c_str());
    }
    fflush(stdout);
}
