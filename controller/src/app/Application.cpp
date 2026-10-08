#include "Application.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <time.h>

#include <unitree/robot/channel/channel_factory.hpp>

using unitree_hg::msg::dds_::LowState_;
using unitree_hg::msg::dds_::LowCmd_;

static float MaxJointSpeed(const RobotState& state, int& worst_joint);

/**
 * Cờ yêu cầu dừng ứng dụng, được kích hoạt bởi signal handler (SIGTERM/SIGINT).
 */
std::atomic<bool> Application::s_stop_requested_{false};

void Application::RequestStop(int /*sig*/) {
    s_stop_requested_.store(true, std::memory_order_relaxed);
}

namespace {

int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

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

bool HasExt(const std::string& name, const std::string& ext) {
    if (name.size() < ext.size()) return false;
    std::string tail = name.substr(name.size() - ext.size());
    std::transform(tail.begin(), tail.end(), tail.begin(), ::tolower);
    return tail == ext;
}

void ScanDanceFolder(const std::string& dir, const std::string& label,
                     std::string& onnx, std::string& npz, std::string& music) {
    static const char* kAudioExt[] = {".mp3", ".wav", ".ogg", ".flac", ".m4a"};
    onnx.clear(); npz.clear(); music.clear();
    DIR* d = opendir(dir.c_str());
    if (!d) {
        std::cerr << "[Application] Failed to open dance folder " << label << ": " << dir << "\n";
        return;
    }
    for (dirent* e = readdir(d); e; e = readdir(d)) {
        std::string name = e->d_name;
        if (name == "." || name == "..") continue;
        auto pick = [&](std::string& slot) {
            if (slot.empty()) slot = name;
            else std::cerr << "[Application] Dance " << label << ": multiple files of same type, using '"
                           << slot << "', ignoring '" << name << "'.\n";
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
    voice_command_gesture_active_ = false;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_.clear();
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
    voice_command_gesture_active_ = false;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_.clear();
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

void Application::InitControllers() {
    const FlatPolicyProfile profile = ParseFlatPolicyProfile(tuning_.flat_policy_contract);
    switch (profile) {
        case FlatPolicyProfile::kLegacy83:
            locomotion_ = std::make_unique<LocomotionController>();
            break;
        case FlatPolicyProfile::kFlatPlusH4V1:
            locomotion_ = std::make_unique<FlatPlusController>();
            break;
        case FlatPolicyProfile::kFlatPlusH5V1:
            locomotion_ = std::make_unique<FlatPlusH5Controller>();
            break;
        case FlatPolicyProfile::kFlatPlusGaitH4V1: {
            auto gait_controller = std::make_unique<FlatPlusGaitController>();
            gait_controller->ConfigureGait({
                tuning_.gait_stop_request_lin,
                tuning_.gait_stop_request_ang,
                tuning_.gait_move_request_lin,
                tuning_.gait_move_request_ang,
                tuning_.gait_stability_gyro_norm,
                tuning_.gait_stability_joint_velocity_rms,
                tuning_.gait_settle_dwell_s,
                tuning_.gait_w2s_timeout_s,
                tuning_.gait_stability_filter_tau_s},
                tuning_.gait_period_s);
            locomotion_ = std::move(gait_controller);
            break;
        }
    }

    // Legacy assets stay in policies/flat. New single-policy contracts mirror
    // simulate_cpp under policies/locomotion/flat_plus and are opt-in only.
    const std::string policy_dir =
        profile == FlatPolicyProfile::kLegacy83
            ? "/policies/flat/"
            : "/policies/locomotion/flat_plus/";
    const std::string policy_path = proj_dir_ + policy_dir + tuning_.flat_model;
    if (tuning_.teleop_h4_enabled) {
        h4_teleop_approved_ = locomotion_->SupportsH4TeleopOverlay() &&
            teleop_h4::Approved(true, tuning_.flat_policy_contract, policy_path,
                                tuning_.teleop_h4_model_sha256);
        if (!h4_teleop_approved_)
            throw std::runtime_error("H4 teleop denied: selected policy contract or SHA-256 differs");
    }
    locomotion_->Init(policy_path,
                      ort_env_, ort_opts_, tuning_);

    // Hợp đồng máy-đọc cho chuỗi deploy. preflight.sh và update_model_manifest.sh
    // đọc DÒNG NÀY thay vì tự suy ra đường dẫn + số chiều theo contract: hai script
    // đó từng có bảng contract riêng, lệch với runtime cả hai chiều (nhận
    // r1_unified_v10 mà runtime từ chối; chặn flat_plus_h5_v1 mà runtime chạy được)
    // và ép mọi model nằm dưới policies/flat trong khi non-legacy nạp từ
    // policies/locomotion/flat_plus. Thêm contract mới thì KHÔNG phải sửa script.
    std::cout << "[Preflight] MODEL contract=" << tuning_.flat_policy_contract
              << " rel=" << policy_dir.substr(1) << tuning_.flat_model
              << " input=" << locomotion_->ObsSize()
              << " output=" << spec::kNumJoints << "\n";

    // Đặt ở đây (chứ không ở Run) để --preflight cũng in ra: chu kỳ gait là một phần
    // hợp đồng train giống Kp/Kd, phải đọc được mà không cần cắm DDS.
    gait_.Configure(tuning_.gait_period_s);
    std::cout << "[" << locomotion_->Name() << "] gait_period=" << gait_.period_s()
              << "s (phai khop 'period' cua phase obs luc train " << tuning_.flat_model << ")\n";
    if (profile == FlatPolicyProfile::kFlatPlusGaitH4V1) {
        std::cout << "[FlatPlusGaitH4] gait_settle_dwell_s="
                  << tuning_.gait_settle_dwell_s << "s (runtime config)\n";
    }

    InitGestures();

    for (int i = 0; i < Tuning::kMaxDances; ++i) {
        if (tuning_.dance_folder[i].empty()) continue;
        int key = i + 2; // keys 2-8
        std::string dir = proj_dir_ + "/policies/dance/" + tuning_.dance_folder[i];

        std::string onnx, npz, music;
        ScanDanceFolder(dir, std::to_string(key), onnx, npz, music);
        if (onnx.empty() || npz.empty()) {
            std::cerr << "[Application] Dance " << key << " (" << tuning_.dance_folder[i]
                      << ") missing required files, skipping.\n";
            continue;
        }

        try {
            auto motion = std::make_unique<MotionData>();
            motion->Load(dir + "/" + npz);

            int start_frame = tuning_.dance_start_frame;
            if (start_frame < 0) {
                const int legacy_frame =
                    motion->FindSmoothStartFrame(tuning_.dance_start_search_frames);
                start_frame = legacy_frame;
                if (tuning_.mimic_transition_v2) {
                    std::array<float, spec::kNumJoints> zero_dq{};
                    start_frame = motion->FindTransitionStartFrame(
                        tuning_.mimic_transition_search_frames,
                        locomotion_->default_q(), zero_dq);
                    std::cout << "[Application] Dance " << key
                              << " start_frame=-1 -> legacy=" << legacy_frame
                              << ", transition_v2 preview=" << start_frame << "\n";
                } else {
                    std::cout << "[Application] Dance " << key
                              << " start_frame=-1 -> selected frame " << start_frame << "\n";
                }
            }

            auto mimic = std::make_unique<MimicController>(
                *motion, start_frame, tuning_.dance_speed[i], tuning_.mimic_warmup_s);
            mimic->ConfigureTransitionV2(
                tuning_.mimic_transition_v2,
                tuning_.dance_start_frame < 0,
                tuning_.mimic_transition_search_frames,
                tuning_.mimic_transition_clip_ramp_s);
            mimic->Init(dir + "/" + onnx, ort_env_, ort_opts_, tuning_);

            // Tính toán độ lệch giữa tư thế mở màn của bài nhảy và tư thế hiện tại.
            // Mục đích: Kiểm tra xem có cần tăng thời gian chuyển động (khởi động mềm) để đảm bảo an toàn hay không.
            const auto& dq = locomotion_->default_q();
            const auto& sp = mimic->start_pose();
            float leg_gap = 0.0f, other_gap = 0.0f;
            for (int j = 0; j < spec::kNumJoints; ++j) {
                float e = std::abs(sp[j] - dq[j]);
                if (j < 12) leg_gap   = std::max(leg_gap, e);
                else        other_gap = std::max(other_gap, e);
            }
            std::cout << "[Application] Dance " << key << " clip-vs-default gap: legs "
                      << leg_gap << " rad, arms/waist " << other_gap << " rad"
                      << (leg_gap > 0.35f ? "  <-- CHAN LECH NHIEU, kiem tra do vung!" : "") << "\n";

            // default_joint_pos của dance policy phải cùng "thế hệ" với locomotion policy:
            // RunReturning đưa robot về locomotion default và BeginCooldown cũng nhắm về
            // đó, nên lệch bao nhiêu là dance vào/ra ở tư thế lệch bấy nhiêu so với lúc
            // train nó. (vd pokemon_v9/doremon_v9 có hip_roll 0.035 - cặp với policy_v3,
            // không phải policy_goc.)
            float def_gap = 0.0f;
            int   def_gap_j = -1;
            for (int j = 0; j < spec::kNumJoints; ++j) {
                float e = std::abs(mimic->default_q()[j] - dq[j]);
                if (e > def_gap) { def_gap = e; def_gap_j = j; }
            }
            if (def_gap > 0.01f) {
                std::cerr << "[Application] ⚠ Dance " << key << " default_joint_pos LECH "
                          << def_gap << " rad (khop " << def_gap_j << ") so voi "
                          << tuning_.flat_model
                          << " -- dance nay train o the he policy khac.\n";
            }

            motions_[key] = std::move(motion);
            mimics_[key] = std::move(mimic);
            if (!music.empty()) music_files_[key] = dir + "/" + music;
            std::cout << "[Application] Loaded Mimic " << key << " ("
                      << tuning_.dance_folder[i] << ")\n";
        } catch (const std::exception& e) {
            disabled_dance_reasons_[key] = e.what();
            std::cerr << "[Application] Dance " << key << " (" << tuning_.dance_folder[i]
                      << ") DISABLED: " << e.what() << "\n";
        }
    }
    active_ = locomotion_.get();

    // Nạp quỹ đạo chuyển động đứng dậy (Get up) và nằm xuống (Lie down).
    // Hệ thống sẽ in ra cảnh báo nếu không tìm thấy file ghi quỹ đạo.
    auto load_traj = [&](const std::string& file, const char* label,
                         std::unique_ptr<JointTrajectory>& out) {
        std::string path = proj_dir_ + "/" + file;
        std::ifstream probe(path);
        if (!probe.good()) {
            std::cerr << "[Application] " << label << ": chưa có file " << path
                      << " -> L2+X (" << label << ") sẽ bị chặn tới khi bạn ghi bằng "
                         "tools/record_motion.cpp.\n";
            return;
        }
        try {
            out = std::make_unique<JointTrajectory>();
            out->Load(path);
        } catch (const std::exception& e) {
            std::cerr << "[Application] Lỗi nạp " << label << " (" << path << "): " << e.what() << "\n";
            out.reset();
        }
    };
    load_traj(tuning_.getup_motion_file, "GET UP", getup_motion_);
    load_traj(tuning_.liedown_motion_file, "LIE DOWN", liedown_motion_);
}

int Application::Preflight() {
    std::cout << "[Preflight] Loading models and assets without DDS/motor output...\n";
    InitControllers();
    std::cout << "[Preflight] controller: OK\n";
    return 0;
}

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
        std::cout << "H4: nhan START mot lan de arm/disarm trong LOCOMOTION hoac ZERO TORQUE.\n";
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
        voice_command_gesture_active_ = false;
        voice_command_retract_requested_ = false;
        voice_command_gesture_id_.clear();
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
        voice_command_gesture_active_ = false;
        voice_command_retract_requested_ = false;
        voice_command_gesture_id_.clear();
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
                voice_command_gesture_active_ = false;
                voice_command_retract_requested_ = false;
                voice_command_gesture_id_.clear();
                voice_gesture_owner_.Reply(true, "idle");
            }
        } else if (request.command == "play" &&
                   (!tuning_.gesture_enabled || !armed_ ||
                    state_ != AppState::kLocomotion)) {
            voice_gesture_owner_.Reply(false, "rejected", "not_in_locomotion");
        }
    }
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

    // Cạnh nhấn START: bật/tắt UDP overlay tay+đầu.
    if (cmd.want_teleop_toggle) {
        if (!tuning_.teleop_enabled) {
            std::cout << "[Teleop] START bị bỏ qua — teleop_enabled=false trong gestures.yaml.\n";
        } else if (locomotion_ && !locomotion_->SupportsArmOverlay() && !h4_teleop_approved_) {
            std::cout << "[Teleop] START bi tu choi — contract hien tai "
                         "khong ho tro arm overlay va H4 chua qua artifact gate.\n";
        } else if (!teleop_runtime_on_ && state_ != AppState::kLocomotion &&
                   !(h4_teleop_approved_ && state_ == AppState::kZeroTorque)) {
            // Teleop là overlay TRÊN policy đi bộ: chân phải đang được policy giữ.
            // Từ chối có tiếng chứ không im lặng, để không bị hiểu là hỏng nút.
            std::cout << "[Teleop] START bi tu choi — phai dang LOCOMOTION (R2+A) "
                         "hoac H4 ZERO TORQUE. Trang thai hien tai: " << StateName() << "\n";
        } else {
            SetTeleopArmed(!teleop_runtime_on_, "START");
        }
    }

    UpdateSafeStop(cmd);

    // Phát âm thanh khi đổi chế độ tốc độ nhanh/chậm.
    bool now_fast = input_.IsFastMode();
    if (speed_voice_inited_ && now_fast != prev_fast_) {
        const std::string& v = now_fast ? tuning_.voice_fast_speed : tuning_.voice_slow_speed;
        if (!v.empty()) music_.Say(v, tuning_.voice_volume, tuning_.voice_speaker_id);
    }
    prev_fast_ = now_fast;
    speed_voice_inited_ = true;

    UpdateBattery();

    HandleTransitions(cmd);

    // A new LOCOMOTION entry must see a fresh explicit STOP before any H4
    // enable packet can own the upper body, including returns from dance.
    if (h4_teleop_approved_) {
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
        voice_command_gesture_active_ = false;
        voice_command_retract_requested_ = false;
        voice_command_gesture_id_.clear();
    }

    gait_.Update(spec::kLoopDt);

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

    CaptureMimicTelemetry(low);
    UpdateHud();
    if (tick_ % 500 == 0) PrintStatus();
    return true;
}

void Application::PublishIntegrationStatus(bool force) {
    if (!force && tick_ - integration_last_tick_ < 100) return;
    integration_last_tick_ = tick_;
    integration_notifier_.Send(audio_busy_.load(), armed_.load(), StateName());
}

void Application::RequestFromPolicy(PendingAction act, bool has_chair) {
    if (pending_action_ == act) return;

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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
    input_.ZeroVelocity();
    fall_detector_.Reset();
    state_ = AppState::kLieDown;
}

void Application::HandleTransitions(const InputCommand& cmd) {
    const RobotState& rs = estimator_.state();

    if (lock_block_timer_ > 0.0f) lock_block_timer_ -= spec::kLoopDt;
    if (sit_block_timer_  > 0.0f) sit_block_timer_  -= spec::kLoopDt;
    if (getup_liedown_block_timer_ > 0.0f) getup_liedown_block_timer_ -= spec::kLoopDt;

    if (cmd.want_stand_lock) {
        if (state_ == AppState::kMimic) {
            // Đang nhảy: Chỉ hủy điệu nhảy, không gồng cứng ngay.
            AbortDance();
        } else if (state_ == AppState::kReturningToDefault) {
            // Robot đang gồng về default chờ vào dance: cho phép huỷ bằng tay thay vì
            // phải đợi hết return_timeout_s hoặc bấm E-stop.
            std::cout << "[Application] Huỷ vào dance " << pending_dance_key_
                      << " khi đang RETURNING -> STAND LOCK.\n";
            BeginStandUp(rs, /*warn_first=*/false);
            return;
        } else if (state_ == AppState::kLocomotion) {
            if (lock_block_timer_ > 0.0f) {
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
            return;
        } else if (state_ == AppState::kIdle || state_ == AppState::kSafeShutdown ||
                   state_ == AppState::kZeroTorque) {
            // Từ Zero Torque, L2+Lên thoát limp và chuyển sang stand lock.
            // (Nhánh nằm đã bắt ở trên -> tới đây IMU xác nhận robot đang dựng đứng,
            // không còn nguy cơ stand-lock trong lúc nằm.)
            BeginStandUp(rs, /*warn_first=*/true);
            return;
        }
    }

    if (cmd.want_safe_shutdown) {
        if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
            RequestFromPolicy(PendingAction::kToSit, /*has_chair=*/true);   // L2+Trái: có ghế
        } else if (state_ == AppState::kStandLock) {
            if (sit_block_timer_ > 0.0f) {
                std::cout << "[Application] Sit blocked for another "
                          << sit_block_timer_ << "s after entering STAND LOCK.\n";
            } else {
                BeginSit(rs, /*has_chair=*/true);   // L2+Trái: có ghế
                return;
            }
        }
    }

    if (cmd.want_get_up_down) {
        if (state_ == AppState::kGetUp && traj_phase_ == 1) {
            // Bước 2 đứng dậy: Bắt đầu phát getup.npz.
            std::cout << "[Application] ĐỨNG DẬY: phát chuỗi getup.npz -> Locomotion khi xong.\n";
            if (!tuning_.voice_get_up.empty())
                music_.Say(tuning_.voice_get_up, tuning_.voice_volume, tuning_.voice_speaker_id);
            traj_phase_ = 2;
            traj_timer_ = 0.0f;
            return;
        } else if (state_ == AppState::kStandLock) {
            if (getup_liedown_block_timer_ > 0.0f) {
                std::cout << "[Application] Lie-down blocked for another "
                          << getup_liedown_block_timer_ << "s.\n";
            } else {
                BeginLieDown(rs);   // đang ĐỨNG -> nằm xuống
                return;
            }
        } else if (state_ == AppState::kLocomotion || state_ == AppState::kMimic) {
            RequestFromPolicy(PendingAction::kToLieDown);   // dừng dưới policy rồi nằm
        } else if ((state_ == AppState::kIdle || state_ == AppState::kZeroTorque) &&
                   RobotLying(rs)) {
            // L2+X chung: IMU nói robot đang NẰM -> lần 1 chuẩn bị (gồng về nằm-chuẩn),
            // lần 2 (khi kGetUp phase 1, nhánh đầu ở trên) mới thực sự đứng dậy.
            BeginGetUp(rs);
            return;
        } else {
            std::cout << "[Application] Lie-down/Get-up blocked in state " << StateName() << ".\n";
        }
    }

    if (pending_action_ != PendingAction::kNone && state_ == AppState::kLocomotion) {
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
        return;
    }

    if (cmd.want_locomotion) {
        if (state_ == AppState::kStandLock) {
            std::cout << "[Application] Activating LOCOMOTION.\n";
            ActivatePolicy(locomotion_.get());
            state_ = AppState::kLocomotion;
            music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
        } else if (state_ != AppState::kLocomotion) {
            std::cout << "[Application] Blocked: Must be in STAND_LOCK to activate policy.\n";
        }
    }

    if (cmd.want_mimic_key && state_ == AppState::kLocomotion) {
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
            int vidx = cmd.want_mimic_key - 2;
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

    if (announcing_ && state_ == AppState::kLocomotion) {
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

    if (cmd.want_reset &&
        (state_ == AppState::kLocomotion || state_ == AppState::kMimic)) {
        std::cout << "[Application] Resetting controller: " << active_->Name() << "\n";
        active_->Reset(rs);
    }
}

void Application::StartMimicTelemetry(int dance_key) {
    if (!tuning_.mimic_telemetry_enabled) return;

    const int idx = dance_key - 2;
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

void Application::RunStandUp() {
    const auto& default_q = locomotion_->default_q();
    std::array<float, spec::kNumJoints> target;

    // Pha cảnh báo khi chuyển sang stand lock.
    // Đường từ đi bộ đã chờ xong dưới policy nên stand_warn_timer_ vào đây đã đầy.
    if (stand_warn_timer_ < tuning_.stand_lock_warn_s) {
        stand_warn_timer_ += spec::kLoopDt;
        sender_.Send(stand_start_q_, stand_gains_kp_, stand_gains_kd_,
                     tuning_.stand_rate_limit, estimator_.state().mode_machine);
        return;
    }

    stand_timer_ += spec::kLoopDt;
    float progress = std::min(1.0f, stand_timer_ / tuning_.stand_up_time_s);

    for (int i = 0; i < spec::kNumJoints; ++i)
        target[i] = stand_start_q_[i] + progress * (default_q[i] - stand_start_q_[i]);

    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.stand_rate_limit, estimator_.state().mode_machine);

    if (progress >= 1.0f) {
        std::cout << "[Application] STAND LOCK active.\n";
        state_ = AppState::kStandLock;
        sit_block_timer_ = std::max(0.0f, tuning_.stand_lock_sit_block_s);
    }
}

void Application::RunStandLock() {
    std::array<float, spec::kNumJoints> target = locomotion_->default_q();
    target[1] += tuning_.stand_lock_spread;
    target[7] -= tuning_.stand_lock_spread;
    sender_.Send(target, stand_gains_kp_, stand_gains_kd_,
                 tuning_.lock_rate_limit, estimator_.state().mode_machine);
}

// max|dq| trên toàn bộ khớp + chỉ số khớp nhanh nhất (cho lớp an toàn tốc độ khớp).
static float MaxJointSpeed(const RobotState& rs, int& worst_joint) {
    float m = 0.0f;
    worst_joint = -1;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        float a = std::fabs(rs.dq[i]);
        if (a > m) { m = a; worst_joint = i; }
    }
    return m;
}

void Application::RunReturning() {
    const RobotState& rs = estimator_.state();

    int worst_j = -1;
    float max_dq = MaxJointSpeed(rs, worst_j);
    std::vector<std::string> reasons;
    if (fall_detector_.Check(rs.projected_gravity, rs.gyro, max_dq, worst_j, reasons)) {
        std::cout << "\n[Application] Fall detected while returning to default pose!\n";
        for (const auto& r : reasons) std::cout << "  - " << r << "\n";
        EnterIdle("Fall detected - triggering damping");
        return;
    }

    auto mit = mimics_.find(pending_dance_key_);
    if (mit == mimics_.end()) {
        std::cout << "[Application] Dance " << pending_dance_key_
                  << " disappeared - back to LOCOMOTION.\n";
        ActivatePolicy(locomotion_.get());
        state_ = AppState::kLocomotion;
        return;
    }

    // Đưa robot về tư thế đứng tĩnh ổn định, để soft-start xử lý chuyển sang tư thế mở màn.
    // Cổ chân dùng return_kp_ankle (mềm hơn stand_kp_leg) cho đỡ khựng khi vừa rời
    // locomotion; các khớp còn lại vẫn stand gains. Xem return_gains_kp_ ở Application.hpp.
    const auto& target = locomotion_->default_q();
    sender_.Send(target, return_gains_kp_, stand_gains_kd_,
                 tuning_.return_rate_limit, rs.mode_machine);

    float max_err = 0.0f;
    int   worst_err_j = -1;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        float e = std::abs(rs.q[i] - target[i]);
        if (e > max_err) { max_err = e; worst_err_j = i; }
    }
    const bool return_pose_ready =
        max_err <= tuning_.return_pos_tol &&
        (!tuning_.mimic_transition_v2 ||
         max_dq <= tuning_.mimic_transition_exit_dq_tol);
    if (!return_pose_ready) {
        // Trần thời gian: khớp kẹt hoặc lệch calib thì max_err không bao giờ xuống dưới
        // tol, mà state này KHÔNG nhận nút nào (chỉ E-stop) -> treo vô hạn với stand gains.
        return_timer_ += spec::kLoopDt;
        if (return_timer_ >= std::max(0.5f, tuning_.return_timeout_s)) {
            std::cout << "\n[Application] RETURNING quá " << tuning_.return_timeout_s
                      << "s mà khớp " << worst_err_j << " vẫn lệch " << max_err
                      << " rad (tol " << tuning_.return_pos_tol << ", max dq "
                      << max_dq << ", dq tol " << tuning_.mimic_transition_exit_dq_tol
                      << ") - HUỶ dance " << pending_dance_key_
                      << ", về STAND LOCK.\n";
            BeginStandUp(rs, /*warn_first=*/false);
        }
        return;
    }

    std::cout << "[Application] Default pose reached (max err " << max_err
              << " rad) - MIMIC " << pending_dance_key_ << " soft-start ("
              << tuning_.mimic_warmup_s << "s).\n";
    const float dance_trim_deg = tuning_.DanceTrimDeg(pending_dance_key_);
    estimator_.SetPitchTrimDeg(dance_trim_deg);
    std::cout << "[Application] Dance " << pending_dance_key_
              << " IMU pitch trim: " << dance_trim_deg << " deg.\n";
    ActivatePolicy(mit->second.get());
    std::cout << "[Application] Dance " << pending_dance_key_
              << " runtime entry frame=" << mit->second->StartFrame()
              << (mit->second->TransitionV2Enabled() ? " (transition_v2)" : " (legacy)")
              << "\n";
    music_started_ = false;   // nhạc chỉ bật khi soft-start xong
    dance_stopping_ = false;  // Reset() đã xoá cờ cooldown trong controller
    state_ = AppState::kMimic;
}

void Application::RunSafeShutdown() {
    const RobotState& rs = estimator_.state();
    const auto& dq = locomotion_->default_q();
    const float D2R = static_cast<float>(M_PI) / 180.0f;

    // Trần cổ chân: xem spec::kAnklePitch*/kAnkleRollLimit (có nguồn gốc ở đó).
    constexpr float kAnkPMin = spec::kAnklePitchMin, kAnkPMax = spec::kAnklePitchMax;

    // "desc" là tư thế hạ tự cân bằng. "rest" là tư thế ngồi cuối khi đã có ghế đỡ.
    float spread_d = std::max(0.0f, tuning_.sit_spread);          // dạng chân lúc hạ (nhỏ, an toàn)
    float hip_d    = -std::fabs(tuning_.sit_hip_deg)  * D2R;      // hông lúc hạ (sâu, tự cân bằng)
    float knee_d   = std::clamp(tuning_.sit_knee_deg  * D2R, 0.3f, 2.42f);
    float spread_r = std::max(0.0f, tuning_.sit_rest_spread);     // dạng chân CUỐI (đo thực)
    float yaw_r_v  = tuning_.sit_rest_hip_yaw;                    // xoay mũi chân CUỐI (đo thực)
    float hip_r    = -std::fabs(tuning_.sit_rest_hip_deg) * D2R;  // hông CUỐI (đo thực, nông hơn)
    float knee_r   = std::clamp(tuning_.sit_rest_knee_deg * D2R, 0.3f, 2.42f);
    float lean_d   = std::max(0.0f, tuning_.sit_lean_deg) * D2R;         // đổ người khi hạ
    float lean_s   = std::max(0.0f, tuning_.sit_seated_lean_deg) * D2R;  // sau khi ngồi hẳn

    sit_timer_ += spec::kLoopDt;
    float Tg = std::max(0.1f, tuning_.sit_gather_time_s);
    float Td = std::max(0.1f, tuning_.sit_descent_time_s);
    float Ts = std::max(0.1f, tuning_.sit_settle_time_s);

    std::array<float, spec::kNumJoints> target = dq;
    float lean = 0.0f;

    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    if (sit_phase_ == 0) {
        // Thu chân về tư thế đứng thẳng chuẩn bị ngồi.
        float e = Ease(sit_timer_ / Tg);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = sit_start_q_[i] + e * (dq[i] - sit_start_q_[i]);
        if (sit_timer_ >= Tg) { sit_phase_ = 1; sit_timer_ = 0.0f; }
    } else {
        float hip, knee, roll_l, roll_r, yaw_l, yaw_r, arm, elb;

        if (sit_phase_ == 1) {
            // Hạ người có cân bằng (chưa chắc có ghế đỡ): Gập hông, gối, vươn tay.
            float e = Ease(sit_timer_ / Td);
            hip  = dq[0]  + e * (hip_d  - dq[0]);
            knee = dq[3]  + e * (knee_d - dq[3]);
            roll_l = e * (-spread_d); roll_r = e * spread_d;
            yaw_l = 0.0f; yaw_r = 0.0f;
            lean = e * lean_d;
            arm  = dq[14] + e * (tuning_.sit_arm_forward - dq[14]);
            elb  = dq[17] + e * (tuning_.sit_arm_elbow   - dq[17]);
            if (sit_timer_ >= Td) {
                // Có ghế (L2+Trái): sang pha 2, nới sang tư thế CUỐI (ghế đỡ rồi, an toàn).
                // KHÔNG ghế (tự động, không ai đỡ): GIỮ NGUYÊN tư thế "desc" tự cân bằng —
                // KHÔNG relax sang rest (rest chỉ an toàn khi có ghế đỡ trọng lượng).
                sit_phase_ = sit_has_chair_ ? 2 : 3;
                sit_timer_ = 0.0f;
            }
        } else if (sit_phase_ == 2) {
            // Chuyển sang tư thế ngồi cuối khi ghế đã đỡ trọng lượng.
            float e = Ease(sit_timer_ / Ts);
            hip  = hip_d  + e * (hip_r  - hip_d);
            knee = knee_d + e * (knee_r - knee_d);
            roll_l = -spread_d + e * (-spread_r - (-spread_d));
            roll_r =  spread_d + e * ( spread_r -  spread_d);
            yaw_l  = e * (-yaw_r_v); yaw_r = e * yaw_r_v;
            lean = lean_d + e * (lean_s - lean_d);
            arm  = tuning_.sit_arm_forward + e * (tuning_.sit_seated_arm_pitch - tuning_.sit_arm_forward);
            elb  = tuning_.sit_arm_elbow   + e * (tuning_.sit_seated_arm_elbow - tuning_.sit_arm_elbow);
            if (sit_timer_ >= Ts) { sit_phase_ = 3; sit_timer_ = 0.0f; }
        } else {
            // Pha 3: Giữ tư thế ngồi (rest nếu có ghế, desc nếu tự cân bằng không ghế).
            if (sit_has_chair_) {
                hip = hip_r; knee = knee_r;
                roll_l = -spread_r; roll_r = spread_r;
                yaw_l = -yaw_r_v; yaw_r = yaw_r_v;
                lean = lean_s;
                arm = tuning_.sit_seated_arm_pitch; elb = tuning_.sit_seated_arm_elbow;
            } else {
                hip = hip_d; knee = knee_d;
                roll_l = -spread_d; roll_r = spread_d;
                yaw_l = 0.0f; yaw_r = 0.0f;
                lean = lean_d;
                arm = tuning_.sit_arm_forward; elb = tuning_.sit_arm_elbow;
            }
        }

        target[0] = target[6] = hip;
        target[3] = target[9] = knee;
        target[1] = roll_l; target[7] = roll_r;
        target[2] = yaw_l;  target[8] = yaw_r;
        target[14] = target[19] = arm;
        target[17] = target[22] = elb;
    }

    // Bù góc cổ chân để giữ lòng bàn chân phẳng với mặt đất.
    const Eigen::Vector3f& g = rs.projected_gravity;
    float pitch_meas = std::atan2(g.x(), -g.z());
    float roll_meas  = std::atan2(-g.y(), -g.z());
    float kg = std::clamp(tuning_.sit_ankle_gravity_gain, 0.0f, 1.0f);
    float corr_p = std::clamp(kg * (pitch_meas - lean), -0.17f, 0.17f);
    float corr_r = std::clamp(kg * roll_meas, -0.17f, 0.17f);

    target[4]  = std::clamp(-(target[0] + target[3]) - lean - corr_p, kAnkPMin, kAnkPMax);
    target[10] = std::clamp(-(target[6] + target[9]) - lean - corr_p, kAnkPMin, kAnkPMax);
    // Cổng chân roll bù hip roll để lòng bàn chân phẳng; clamp tránh vượt tầm khi dạng rộng.
    // Ở pha GIỮ, tư thế ngồi đứng yên vô hạn (sit_release_after=false), nên trần
    // này là giá trị cổ chân bị ép giữ liên tục chứ không phải một đỉnh thoáng
    // qua. sit_rest_spread=0.45 đẩy target lên 0.45..0.62 -> luôn chạm trần.
    target[5]  = std::clamp(-target[1] - corr_r, -spec::kAnkleRollLimit, spec::kAnkleRollLimit);
    target[11] = std::clamp(-target[7] - corr_r, -spec::kAnkleRollLimit, spec::kAnkleRollLimit);

    sender_.Send(target, sit_gains_kp_, sit_gains_kd_,
                 tuning_.sit_rate_limit, rs.mode_machine);

    if (tick_ % 250 == 0) {
        const char* pha = sit_phase_ == 0 ? "THU CHAN"
                        : sit_phase_ == 1 ? "HA NGUOI (do than + vuon tay)"
                        : sit_phase_ == 2 ? "NGOI HAN (giao luc cho ghe)" : "GIU";
        printf("  [sit] %s  t=%.1fs  than_do=%.0f/%.0f deg  hong_nghieng_thuc=%.0f deg\n",
               pha, sit_timer_, lean / D2R, lean_d / D2R, pitch_meas / D2R);
        fflush(stdout);
    }

    if (sit_phase_ == 3 && sit_timer_ >= std::max(0.0f, tuning_.sit_hold_s)) {
        if (tuning_.sit_release_after) {
            std::cout << "[Application] Sit completed - releasing joints.\n";
            EnterIdle("Sit completed - damping");
        } else {
            sit_phase_ = 4;  // giữ vô hạn; không quay lại nhánh này nữa
            std::cout << "[Application] Sit completed - holding pose.\n";
        }
    }
}

// Bù cổ chân theo IMU dựa trên quỹ đạo ghi được để giữ lòng bàn chân bám sàn.
void Application::ApplyAnkleFlat(std::array<float, spec::kNumJoints>& target,
                                 const JointTrajectory& motion, float t_ref) {
    float kg = tuning_.getup_ankle_gravity_gain;
    if (kg <= 0.0f || !motion.has_torso()) return;
    constexpr float kAnkPMin = spec::kAnklePitchMin, kAnkPMax = spec::kAnklePitchMax;
    constexpr float kAnkRLim = spec::kAnkleRollLimit, kCorrLim = 0.20f;
    const Eigen::Vector3f& gm = estimator_.state().projected_gravity;
    Eigen::Vector3f gr = motion.RefGravityAt(t_ref);
    float pitch_m = std::atan2(gm.x(), -gm.z());
    float pitch_r = std::atan2(gr.x(), -gr.z());
    float roll_m  = std::atan2(-gm.y(), -gm.z());
    float roll_r  = std::atan2(-gr.y(), -gr.z());
    float corr_p = std::clamp(kg * (pitch_m - pitch_r), -kCorrLim, kCorrLim);
    float corr_r = std::clamp(kg * (roll_m - roll_r), -kCorrLim, kCorrLim);
    target[4]  = std::clamp(target[4]  - corr_p, kAnkPMin, kAnkPMax);
    target[10] = std::clamp(target[10] - corr_p, kAnkPMin, kAnkPMax);
    target[5]  = std::clamp(target[5]  - corr_r, -kAnkRLim, kAnkRLim);
    target[11] = std::clamp(target[11] - corr_r, -kAnkRLim, kAnkRLim);
}

void Application::RunGetUp() {
    if (!getup_motion_) { EnterIdle("GET UP: thiếu getup.npz giữa chừng"); return; }
    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    std::array<float, spec::kNumJoints> target;
    auto f0 = getup_motion_->PoseAt(0.0f);
    float t_ref = 0.0f;   // vị trí tham chiếu để bù cổ chân (0 khi blend/giữ)

    if (traj_phase_ == 0) {
        // Bước 1: Blend từ tư thế hiện tại về tư thế nằm chuẩn (frame 0).
        float blend_t = std::max(0.05f, tuning_.getup_blend_time_s);
        float e = Ease(traj_timer_ / blend_t);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = traj_start_q_[i] + e * (f0[i] - traj_start_q_[i]);
        traj_timer_ += spec::kLoopDt;
        if (traj_timer_ >= blend_t) {
            traj_phase_ = 1;   // sang GIỮ, chờ L2+X
            std::cout << "[Application] Đã vào tư thế nằm-chuẩn - GIỮ. Bấm L2+X để đứng dậy.\n";
        }
    } else if (traj_phase_ == 1) {
        // Giữ tư thế nằm chuẩn chờ L2+X.
        target = f0;
    } else {
        // Bước 2: Phát getup.npz rồi chuyển sang Locomotion.
        float speed = std::max(0.05f, tuning_.getup_speed);
        t_ref = traj_timer_ * speed;
        target = getup_motion_->PoseAt(t_ref);
        traj_timer_ += spec::kLoopDt;
        if (t_ref >= getup_motion_->duration_s()) {
            std::cout << "[Application] ĐỨNG DẬY hoàn tất -> LOCOMOTION (policy tự cân bằng).\n";
            ApplyAnkleFlat(target, *getup_motion_, getup_motion_->duration_s());
            sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                         tuning_.getup_rate_limit, estimator_.state().mode_machine);
            lying_ = false;
            getup_liedown_block_timer_ = std::max(0.0f, tuning_.getup_liedown_block_s);
            ActivatePolicy(locomotion_.get());
            state_ = AppState::kLocomotion;
            music_.Say(tuning_.voice_locomotion, tuning_.voice_volume, tuning_.voice_speaker_id);
            return;
        }
    }
    ApplyAnkleFlat(target, *getup_motion_, t_ref);
    sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                 tuning_.getup_rate_limit, estimator_.state().mode_machine);
}

// Phát liedown.npz; hết clip tự chuyển sang DAMPING và đặt lying_.
void Application::RunLieDown() {
    if (!liedown_motion_) { EnterIdle("LIE DOWN: thiếu liedown.npz giữa chừng"); return; }
    auto Ease = [](float s) { s = std::clamp(s, 0.0f, 1.0f); return s * s * (3.0f - 2.0f * s); };

    std::array<float, spec::kNumJoints> target;
    float t_ref = 0.0f;
    if (traj_phase_ == 0) {
        float blend_t = std::max(0.05f, tuning_.liedown_blend_time_s);
        float e = Ease(traj_timer_ / blend_t);
        auto f0 = liedown_motion_->PoseAt(0.0f);
        for (int i = 0; i < spec::kNumJoints; ++i)
            target[i] = traj_start_q_[i] + e * (f0[i] - traj_start_q_[i]);
        traj_timer_ += spec::kLoopDt;
        if (traj_timer_ >= blend_t) { traj_phase_ = 1; traj_timer_ = 0.0f; }
    } else {
        float speed = std::max(0.05f, tuning_.liedown_speed);
        t_ref = traj_timer_ * speed;
        target = liedown_motion_->PoseAt(t_ref);
        traj_timer_ += spec::kLoopDt;
        if (t_ref >= liedown_motion_->duration_s()) {
            // Nằm xong: chuyển sang DAMPING và đặt lying_.
            std::cout << "[Application] NẰM XUỐNG hoàn tất -> DAMPING (nằm nghỉ). "
                         "L2+Lên để chuẩn bị đứng dậy.\n";
            ApplyAnkleFlat(target, *liedown_motion_, liedown_motion_->duration_s());
            sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                         tuning_.liedown_rate_limit, estimator_.state().mode_machine);
            music_.Stop();
            state_ = AppState::kIdle;
            lying_ = true;
            announcing_ = false;
            pending_action_ = PendingAction::kNone;
            input_.ZeroVelocity();
            input_lost_ = false;
            input_lost_count_ = 0;
            sender_.Damping(estimator_.state().mode_machine);
            return;
        }
    }
    ApplyAnkleFlat(target, *liedown_motion_, t_ref);
    sender_.Send(target, getup_gains_kp_, getup_gains_kd_,
                 tuning_.liedown_rate_limit, estimator_.state().mode_machine);
}

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
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            sender_.SyncToState(shared_low_state_);
        }
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

    bool holding = announcing_ || pending_action_ != PendingAction::kNone;
    // teleop_lock_base: khi teleop đang ARMED thì bỏ qua cần analog -> chân chỉ đứng
    // giữ thăng bằng, teleop đúng nghĩa CHỈ điều khiển tay + đầu.
    const bool legacy_arm_overlay_supported = locomotion_->SupportsArmOverlay();
    const bool h4_overlay_supported = h4_teleop_approved_ &&
        locomotion_->SupportsH4TeleopOverlay();
    const bool history_gesture_contract = tuning_.gesture_history_enabled &&
        locomotion_->SupportsHistoryGestureOverlay();
    const bool gesture_overlay_supported = legacy_arm_overlay_supported ||
        history_gesture_contract;
    const float move_intent = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy +
                                        cmd.yaw * cmd.yaw);
    const bool history_gesture_move_requested = history_gesture_contract &&
        move_intent > tuning_.gesture_history_move_threshold;

    bool history_gesture_ready = !history_gesture_contract;
    if (history_gesture_contract) {
        bool stable = rs.gyro.norm() <= tuning_.gait_stability_gyro_norm &&
                      JointVelocityRms(rs) <= tuning_.gait_stability_joint_velocity_rms;
        if (const auto* gait = dynamic_cast<const FlatPlusGaitController*>(locomotion_.get())) {
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

    // Giữ hướng khi đi thẳng: có thể ghi đè cmd_yaw_ bằng lệnh sửa (no-op nếu tắt).
    ApplyHeadingHold(rs);

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

    if (tick_ % spec::kPolicyDecimation == 0) {
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
            return;
        } catch (...) {
            std::cerr << "\n[FATAL] Unknown ONNX inference failure in " << active_->Name()
                      << "\n";
            EnterIdle("Unknown ONNX inference failure - triggering damping");
            return;
        }

        if (blend_alpha_ < 1.0f) {
            blend_alpha_ = std::min(1.0f, blend_alpha_ + spec::kPolicyDt / tuning_.blend_time_s);
            for (int i = 0; i < spec::kNumJoints; ++i) {
                ai_target_q_[i] = (1.0f - blend_alpha_) * blend_start_q_[i] +
                                   blend_alpha_ * ai_target_q_[i];
            }
        }
    }

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

    std::array<float, spec::kNumJoints> send_q = ai_target_q_;
    // H4 teleop and gestures both mask each 83-D history frame before packing.
    if (ub_active && (gesture_overlay_supported || h4_overlay_supported)) {
        if (use_teleop) teleop_.BlendInto(send_q);
        else            arm_gesture_.BlendInto(send_q);
    }
    sender_.Send(send_q, kp_eff, kd_eff,
                 tuning_.policy_rate_limit, rs.mode_machine, head_tgt);

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
            ActivatePolicy(locomotion_.get());   // ActivatePolicy trả trim locomotion
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

void Application::ActivatePolicy(PolicyController* ctrl) {
    // Trả trim locomotion về ĐÂY, không rải ở từng call-site: có 4 đường vào
    // locomotion (stand-lock, dance biến mất, đứng dậy sau ngã, bàn giao hết bài)
    // và trước đây chỉ đường bàn giao khôi phục trim -> ngã giữa dance rồi đứng
    // dậy là locomotion chạy tiếp bằng trim của bài dance đó.
    // Đặt TRƯỚC Reset() để policy nhận đúng quaternion ngay frame đầu.
    // Mimic tự nạp dance_trim_deg_N của nó ngay trước khi gọi hàm này.
    if (ctrl == locomotion_.get())
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
    voice_command_gesture_active_ = false;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_.clear();
    input_.ZeroVelocity();
    cmd_vx_ = cmd_vy_ = cmd_yaw_ = 0.0f;
    announcing_ = false;
    pending_action_ = PendingAction::kNone;
    dance_stopping_ = false;
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
    voice_command_gesture_active_ = false;
    voice_command_retract_requested_ = false;
    voice_command_gesture_id_.clear();
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
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        sender_.SyncToState(shared_low_state_);
    }
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
            hints = {"1 / R2+A : LOCOMOTION", "L2+Trái : Ngồi ghế", "L2+X : Nằm xuống"};
            break;
        case AppState::kLocomotion: {
            std::string dance_keys = mimics_.empty() ? "" : "2-8 : DANCE | ";
            hints = {"W/S/A/D : Move | Q/E : Yaw",
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
        printf("[%s] vx=%.2f vy=%.2f yaw=%.2f %s\n", StateName().c_str(), cmd_vx_,
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
