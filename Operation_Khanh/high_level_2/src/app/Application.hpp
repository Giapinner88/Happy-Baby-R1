#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <map>
#include <mutex>
#include <string>

#include <onnxruntime_cxx_api.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "../audio/MusicPlayer.hpp"
#include "../audio/IntegrationNotifier.hpp"
#include "../config/RobotSpec.hpp"
#include "../config/Tuning.hpp"
#include "../estimation/StateEstimator.hpp"
#include "../gait/GaitScheduler.hpp"
#include "../input/InputManager.hpp"
#include "../logging/MimicTelemetryLogger.hpp"
#include "../motion/JointTrajectory.hpp"
#include "../motion/MotionData.hpp"
#include "../policy/LocomotionController.hpp"
#include "../policy/MimicController.hpp"
#include "../robot/LowCmdSender.hpp"
#include "../safety/BatteryMonitor.hpp"
#include "../safety/FallDetector.hpp"

// Trạng thái điều khiển
enum class AppState {
    kDisarmed,          // Chờ built-in nhả quyền (P0-1)
    kConflict,          // Xung đột built-in (P0-1)
    kWaitingForState,
    kIdle,
    kZeroTorque,        // Xả lực (limp)
    kStandUp,
    kStandLock,
    kLocomotion,
    kReturningToDefault,
    kMimic,
    kSafeShutdown,
    kGetUp,     // Đứng lên (phát getup.npz)
    kLieDown,   // Nằm xuống (phát liedown.npz)
};

// Hành động chờ sau khi dừng
enum class PendingAction {
    kNone,
    kToStandLock,
    kToSit,
    kToLieDown,
};

// App điều khiển (500Hz)
class Application {
public:
    Application(std::string proj_dir, std::string interface_override);
    int Run();
    int Preflight();

    // Xử lý SIGTERM/SIGINT
    static void RequestStop(int sig);

private:
    void InitDds();
    void InitControllers();
    void OnLowState(const void* msg);
    void OnLowCmdSeen(const void* msg);   // Phát hiện built-in
    bool Tick();
    bool TickDisarmed(const InputCommand& cmd);   // Chờ arm
    void EnterConflict();                          // Xử lý conflict

    void HandleTransitions(const InputCommand& cmd);
    // has_chair: true = có ghế (sit rest), false = tự cân bằng (sit desc)
    void RequestFromPolicy(PendingAction act, bool has_chair = true);
    void BeginSoftStop(const char* why);
    void AbortDance();
    // warn_first: true = chờ người đỡ, false = ép cứng ngay
    void BeginStandUp(const RobotState& rs, bool warn_first);
    void BeginSit(const RobotState& rs, bool has_chair = true);
    // Kiểm tra robot đang nằm
    bool RobotLying(const RobotState& rs) const;
    void BeginGetUp(const RobotState& rs);
    void BeginLieDown(const RobotState& rs);
    void UpdateSafeStop(InputCommand& cmd);
    void UpdateBattery();                 // Giám sát pin (P0-3)
    void RunStandUp();
    void RunStandLock();
    void RunReturning();
    void RunPolicy(const InputCommand& cmd);
    // Giữ hướng đi thẳng (heading-hold)
    void ApplyHeadingHold(const RobotState& rs);
    void RunSafeShutdown();
    void RunGetUp();
    void RunLieDown();
    // Bù cổ chân bám sàn
    void ApplyAnkleFlat(std::array<float, spec::kNumJoints>& target,
                        const JointTrajectory& motion, float t_ref);

    void EnterIdle(const std::string& reason);
    void EnterZeroTorque(const std::string& reason);  // Vào kZeroTorque
    void ActivatePolicy(PolicyController* ctrl);
    void UpdateHud();
    void PrintStatus();
    void StartMimicTelemetry(int dance_key);
    void CaptureMimicTelemetry(const unitree_hg::msg::dds_::LowState_& low);
    void PublishIntegrationStatus(bool force = false);

    std::string StateName() const;

    // Cấu hình
    std::string proj_dir_;
    Tuning tuning_;

    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "r1_policy"};
    Ort::SessionOptions ort_opts_;

    // DDS Subscriber
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_sub_;
    unitree_hg::msg::dds_::LowState_ shared_low_state_;
    std::mutex state_mutex_;
    bool got_state_ = false;
    std::chrono::steady_clock::time_point last_state_time_;

    // Modules
    StateEstimator estimator_;
    GaitScheduler gait_;
    FallDetector fall_detector_;
    InputManager input_;
    LowCmdSender sender_;

    std::unique_ptr<LocomotionController> locomotion_;
    std::map<int, std::unique_ptr<MotionData>> motions_;
    std::map<int, std::unique_ptr<MimicController>> mimics_;
    std::map<int, std::string> music_files_;
    MimicTelemetryLogger telemetry_;
    int pending_dance_key_ = 0;
    PolicyController* active_ = nullptr;

    // Trạng thái
    AppState state_ = AppState::kWaitingForState;
    bool running_ = true;
    long tick_ = 0;

    std::array<float, spec::kNumJoints> ai_target_q_{};
    std::array<float, spec::kNumJoints> stand_gains_kp_{};
    std::array<float, spec::kNumJoints> stand_gains_kd_{};
    std::array<float, spec::kNumJoints> sit_gains_kp_{};
    std::array<float, spec::kNumJoints> sit_gains_kd_{};

    // Đứng dậy
    std::array<float, spec::kNumJoints> stand_start_q_{};
    float stand_timer_ = 0.0f;
    float stand_warn_timer_ = 0.0f;   // Chờ ép cứng

    // Chặn khóa cứng sau hủy dance
    float lock_block_timer_ = 0.0f;

    // Chặn ngồi sau khi stand lock
    float sit_block_timer_ = 0.0f;

    // Đang soft-stop
    bool dance_stopping_ = false;

    // Telemetry mimic -> locomotion
    int telemetry_rate_accum_ = 0;
    bool telemetry_seen_mimic_ = false;
    int telemetry_post_ticks_ = -1;

    // Các pha ngồi ghế
    std::array<float, spec::kNumJoints> sit_start_q_{};
    float sit_timer_ = 0.0f;
    int   sit_phase_ = 0;
    // Cờ có ghế ngồi hay tự cân bằng
    bool  sit_has_chair_ = true;
    bool  pending_sit_has_chair_ = true;   // Giữ has_chair khi settle

    // Getup / LieDown
    std::unique_ptr<JointTrajectory> getup_motion_;
    std::unique_ptr<JointTrajectory> liedown_motion_;
    std::array<float, spec::kNumJoints> getup_gains_kp_{};
    std::array<float, spec::kNumJoints> getup_gains_kd_{};
    std::array<float, spec::kNumJoints> traj_start_q_{};   // Điểm đầu blend-in
    float traj_timer_ = 0.0f;
    // Pha getup / liedown
    int   traj_phase_ = 0;
    float getup_liedown_block_timer_ = 0.0f;   // Chặn bấm dội
    bool  lying_ = false;   // Cờ robot đang nằm

    // Blend tư thế khi bật policy
    float blend_alpha_ = 1.0f;
    std::array<float, spec::kNumJoints> blend_start_q_{};

    // Quản lý việc đọc tên điệu nhảy trước khi thực hiện
    bool  announcing_ = false;
    float announce_timer_ = 0.0f;
    bool  announce_saw_busy_ = false;   // Latch voice phát
    bool  announce_has_voice_ = false;  // Cờ announce có voice

    // Nhạc bật khi mimic xong soft-start
    bool  music_started_ = false;

    // Chờ robot dừng vững
    PendingAction pending_action_ = PendingAction::kNone;
    float settle_timer_ = 0.0f;

    // Watchdog cho mất tín hiệu điều khiển (safe-stop)
    bool input_lost_ = false;
    int  input_lost_count_ = 0;

    float cmd_vx_ = 0.0f, cmd_vy_ = 0.0f, cmd_yaw_ = 0.0f;

    // Heading-hold: hướng (yaw) đã chốt và cờ đang giữ.
    bool  heading_hold_active_ = false;
    float heading_ref_ = 0.0f;

    // P0-2: Dừng êm
    static std::atomic<bool> s_stop_requested_;

    // P0-1: Cổng chống xung đột
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>> low_cmd_sub_;
    // Ghi thread chính, đọc thread DDS
    std::atomic<bool> armed_{true};     // Cờ armed
    float arm_hold_timer_ = 0.0f;       // Timer giữ R1+R2
    bool  arm_intent_latched_ = false;  // Latch ý định arm
    std::atomic<int64_t> last_foreign_ms_{0};   // Gói built-in gần nhất
    std::atomic<long>    foreign_count_{0};     // Tổng gói built-in thấy
    std::atomic<bool>    conflict_flag_{false}; // Cờ conflict
    // Đếm gói lạ chống nhiễu
    std::atomic<int64_t> conflict_first_ms_{0};
    std::atomic<int64_t> conflict_last_ms_{0};
    std::atomic<int> conflict_run_{0};           // tăng ở thread DDS, reset ở thread chính
    std::atomic<uint32_t> last_foreign_crc_{0};  // CRC gói lạ gần nhất
    bool conflict_diag_logged_ = false;          // Log conflict một lần
    std::chrono::steady_clock::time_point disarmed_start_time_;  // Uptime kDisarmed

    // Voice tăng/giảm tốc
    bool prev_fast_ = false;
    bool speed_voice_inited_ = false;

    // Voice khởi động trễ
    // Tránh đè voice built-in
    bool startup_voice_pending_ = false;
    std::chrono::steady_clock::time_point startup_voice_at_;

    // P0-3: Giám sát pin
    BatteryMonitor battery_;
    float battery_warn_timer_ = 0.0f;   // Throttle cảnh báo
    bool  battery_active_ = false;      // Đang giám sát pin
    bool  battery_critical_done_ = false;

    // Khai báo cuối để hủy MusicPlayer an toàn
    IntegrationNotifier integration_notifier_;
    std::atomic<bool> audio_busy_{false};
    long integration_last_tick_ = -1000;
    MusicPlayer music_;
};
