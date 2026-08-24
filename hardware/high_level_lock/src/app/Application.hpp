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
#include "../input/TeleopReceiver.hpp"
#include "../logging/MimicTelemetryLogger.hpp"
#include "../motion/ArmGesturePlayer.hpp"
#include "../motion/JointTrajectory.hpp"
#include "../motion/MotionData.hpp"
#include "../policy/LocomotionController.hpp"
#include "../policy/MimicController.hpp"
#include "../policy/ArmReferenceFilter.hpp"
#include "../policy/FlatPolicyProfile.hpp"
#include "../policy/UnifiedV10Controller.hpp"
#include "../robot/LowCmdSender.hpp"
#include "../safety/BatteryMonitor.hpp"
#include "../safety/FallDetector.hpp"

// Trạng thái chính của máy trạng thái điều khiển
enum class AppState {
    kDisarmed,          // cổng an toàn: KHÔNG publish gì tới khi built-in đã nhả (P0-1)
    kConflict,          // built-in quay lại lúc đang chạy: ngừng publish, latch (P0-1)
    kWaitingForState,
    kIdle,
    kZeroTorque,        // mode=1 kp=kd=tau=0: motor hoàn toàn thụ động (limp). Chỉ từ kIdle.
    kStandUp,
    kStandLock,
    kLocomotion,
    kReturningToDefault,
    kMimic,
    kSafeShutdown,
    kGetUp,     // L2+X từ tư thế nằm: phát lại getup.npz -> STAND LOCK
    kLieDown,   // L2+X từ STAND LOCK: phát lại liedown.npz, giữ vô hạn ở frame cuối
};

// Hành động chờ sau khi robot dừng hẳn dưới chính sách
enum class PendingAction {
    kNone,
    kToStandLock,
    kToSit,
    kToLieDown,
};

// Ứng dụng điều khiển trung tâm (vòng lặp 500Hz)
class Application {
public:
    Application(std::string proj_dir, std::string interface_override);
    int Run();
    int Preflight();

    // Gọi từ signal handler (SIGTERM/SIGINT) — chỉ set cờ atomic, async-signal-safe.
    static void RequestStop(int sig);

private:
    void InitDds();
    void InitControllers();
    void OnLowState(const void* msg);
    void OnLowCmdSeen(const void* msg);   // P0-1: nghe rt/lowcmd để phát hiện built-in
    bool Tick();
    bool TickDisarmed(const InputCommand& cmd);   // P0-1: chờ arm, KHÔNG publish
    void EnterConflict();                          // P0-1: built-in quay lại -> nhả quyền

    void HandleTransitions(const InputCommand& cmd);
    // has_chair (chỉ dùng khi act==kToSit): true = L2+Trái (có ghế, sẽ nới sang tư thế CUỐI
    // sau khi ghế đỡ). false = tự động/pin cạn (không ghế, không ai đỡ) -> giữ nguyên tư
    // thế hạ đã kiểm mô phỏng, tự cân bằng, không relax.
    void RequestFromPolicy(PendingAction act, bool has_chair = true);
    void BeginSoftStop(const char* why);
    void AbortDance();
    // warn_first = true: đã phát voice, giữ nguyên tư thế chờ người đỡ rồi mới ép cứng.
    // false: đã chờ sẵn dưới policy (đường từ đi bộ) -> ép cứng ngay.
    void BeginStandUp(const RobotState& rs, bool warn_first);
    // has_chair: xem RequestFromPolicy ở trên.
    void BeginSit(const RobotState& rs, bool has_chair = true);
    // Đứng lên (từ nằm, phát lại getup.npz) / Nằm xuống (từ đứng, phát lại liedown.npz).
    // Robot đang nằm? Kết hợp cờ lying_ (do chuỗi liedown của ta đặt) VÀ IMU
    // (projected_gravity nghiêng quá lying_tilt_deg) — bắt cả trường hợp robot được
    // đặt nằm bằng built-in/bằng tay hoặc app khởi động lại lúc robot đang nằm.
    bool RobotLying(const RobotState& rs) const;
    void BeginGetUp(const RobotState& rs);
    void BeginLieDown(const RobotState& rs);
    void UpdateSafeStop(InputCommand& cmd);
    void UpdateBattery();                 // P0-3: cảnh báo pin thấp / tự ngồi khi cạn
    void RunStandUp();
    void RunStandLock();
    void RunReturning();
    void RunPolicy(const InputCommand& cmd);
    // Giữ hướng khi đi thẳng (heading-hold): sửa cmd_yaw_ để kéo mặt về hướng đã chốt.
    // Không làm gì nếu tắt / đang đứng / người lái đang bẻ yaw. Xem tuning heading_hold_*.
    void ApplyHeadingHold(const RobotState& rs);
    void RunSafeShutdown();
    void RunGetUp();
    void RunLieDown();
    void RunZeroTorqueTeleop();
    // Bù cổ chân theo IMU để lòng bàn chân bám sàn khi phát lại getup/liedown (dùng
    // torso_quat đã ghi làm tham chiếu). No-op nếu clip không có torso_quat hoặc gain=0.
    void ApplyAnkleFlat(std::array<float, spec::kNumJoints>& target,
                        const JointTrajectory& motion, float t_ref);

    void EnterIdle(const std::string& reason);
    void EnterZeroTorque(const std::string& reason);  // L2+Y: mode=1, kp=kd=0, chỉ từ kIdle
    void ActivatePolicy(PolicyController* ctrl);
    void UpdateHud();
    void PrintStatus();
    void StartMimicTelemetry(int dance_key);
    void CaptureMimicTelemetry(const unitree_hg::msg::dds_::LowState_& low);
    void PublishIntegrationStatus(bool force = false);

    std::string StateName() const;

    // Cấu hình / hạ tầng
    std::string proj_dir_;
    Tuning tuning_;

    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "r1_policy"};
    Ort::SessionOptions ort_opts_;

    // Hạ tầng DDS subscriber
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_sub_;
    unitree_hg::msg::dds_::LowState_ shared_low_state_;
    std::mutex state_mutex_;
    bool got_state_ = false;
    std::chrono::steady_clock::time_point last_state_time_;

    // Các module chức năng
    StateEstimator estimator_;
    GaitScheduler gait_;
    FallDetector fall_detector_;
    InputManager input_;
    LowCmdSender sender_;

    // Legacy 83-D and Unified 105-D share the PolicyController surface. The
    // selected concrete controller is fixed at startup; no hot swap while armed.
    std::unique_ptr<PolicyController> locomotion_;

    // Gesture source chung: legacy sẽ overlay, Unified V10 chỉ dùng làm reference.
    // slot 1..8 -> tên gesture.
    ArmGesturePlayer arm_gesture_;
    ArmReferenceFilter unified_arm_reference_;
    std::array<std::string, 9> gesture_slot_{};   // idx 1..8; [0] không dùng
    float voice_gesture_poll_accum_s_ = 0.1f;
    bool voice_gesture_marker_fresh_ = false;
    bool voice_gesture_latched_ = false;
    bool voice_gesture_auto_enabled_ = false;
    bool voice_gesture_movement_blocked_ = false;
    float voice_gesture_still_s_ = 0.0f;
    float unified_stationary_timer_s_ = 0.0f;
    void InitGestures();
    void HandleGestureTrigger(const InputCommand& cmd);
    bool VoiceGestureSpeaking();
    void HandleVoiceGesture(bool movement_blocked);

    // Teleop thân trên (tay + đầu) qua UDP — nguồn override ưu tiên hơn gesture.
    TeleopReceiver teleop_;
    bool teleop_runtime_on_ = false;   // arm-head profile bật khi InitTeleop; L2+Phải vẫn có thể tắt.
    void InitTeleop();

    std::map<int, std::unique_ptr<MotionData>> motions_;
    std::map<int, std::unique_ptr<MimicController>> mimics_;
    std::map<int, std::string> music_files_;
    // Dance có trong config nhưng bị từ chối lúc preflight (asset/ONNX contract sai).
    // Giữ lý do để nút trigger báo đúng nguyên nhân thay vì im lặng hoặc crash process.
    std::map<int, std::string> disabled_dance_reasons_;
    MimicTelemetryLogger telemetry_;
    int pending_dance_key_ = 0;
    PolicyController* active_ = nullptr;

    // Trạng thái điều khiển
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
    float stand_warn_timer_ = 0.0f;   // chờ sau voice cảnh báo, trước khi ép cứng

    // Chặn "khóa cứng" một lúc sau khi vừa huỷ điệu nhảy (tránh bấm 0 hai lần quá nhanh)
    float lock_block_timer_ = 0.0f;

    // Chặn "ngồi" một lúc sau khi VỪA vào STAND LOCK (tránh bấm dội / kẹt phím / gói tay
    // cầm nhiễu làm robot vừa đứng xong lại ngồi luôn, không kịp phản ứng)
    float sit_block_timer_ = 0.0f;

    // Đang soft-stop: mimic policy đang tự đưa robot về đứng thẳng, chưa giao cho locomotion
    bool dance_stopping_ = false;

    // Logger mỗi điệu: accumulator cho sample-rate không cần chia hết 500 Hz, và cửa
    // sổ hậu handover để bắt transient mimic -> locomotion.
    int telemetry_rate_accum_ = 0;
    bool telemetry_seen_mimic_ = false;
    int telemetry_post_ticks_ = -1;

    // Ngồi ghế (0=thu chân, 1=hạ người, 2=nới sang tư thế cuối [chỉ khi có ghế], 3=giữ)
    std::array<float, spec::kNumJoints> sit_start_q_{};
    float sit_timer_ = 0.0f;
    int   sit_phase_ = 0;
    // true = L2+Trái (có ghế). false = tự động/pin cạn (không ghế, không ai đỡ) -> giữ nguyên
    // tư thế hạ tự cân bằng, KHÔNG relax sang tư thế ngồi-trên-ghế nông hơn.
    bool  sit_has_chair_ = true;
    bool  pending_sit_has_chair_ = true;   // giữ has_chair xuyên qua giai đoạn settle (RequestFromPolicy)

    // Đứng lên / Nằm xuống (L2+X): phát lại quỹ đạo ghi từ built-in (tools/record_motion.cpp).
    // nullptr nếu chưa nạp được file (chưa ghi) — không fatal, chỉ chặn trigger + log.
    std::unique_ptr<JointTrajectory> getup_motion_;
    std::unique_ptr<JointTrajectory> liedown_motion_;
    std::array<float, spec::kNumJoints> getup_gains_kp_{};
    std::array<float, spec::kNumJoints> getup_gains_kd_{};
    std::array<float, spec::kNumJoints> traj_start_q_{};   // tư thế lúc Begin* (điểm đầu blend-in)
    float traj_timer_ = 0.0f;
    // kGetUp:  0=blend về tư thế nằm-chuẩn, 1=GIỮ chờ L2+X, 2=đang phát getup.npz
    // kLieDown: 0=blend về frame đầu clip, 1=đang phát liedown.npz
    int   traj_phase_ = 0;
    float getup_liedown_block_timer_ = 0.0f;   // chặn bấm dội sau khi vừa xong 1 chiều
    bool  lying_ = false;   // robot đang nằm (sau khi nằm xuống + damping) -> L2+Lên = chuẩn bị đứng dậy

    // Blend tư thế khi bật policy
    float blend_alpha_ = 1.0f;
    std::array<float, spec::kNumJoints> blend_start_q_{};

    // Quản lý việc đọc tên điệu nhảy trước khi thực hiện
    bool  announcing_ = false;
    float announce_timer_ = 0.0f;
    bool  announce_saw_busy_ = false;   // đã thấy voice bắt đầu phát (latch)
    bool  announce_has_voice_ = false;  // lần announce này có voice để chờ hay không

    // Nhạc chỉ bật khi mimic soft-start xong (lúc clip thật sự bắt đầu chạy)
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

    // ── P0-2: dừng êm khi nhận SIGTERM/SIGINT ──
    static std::atomic<bool> s_stop_requested_;

    // ── P0-1: cổng chống xung đột built-in ──
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>> low_cmd_sub_;
    // atomic: ghi ở thread chính (arm/EnterConflict), đọc ở thread DDS (OnLowCmdSeen).
    std::atomic<bool> armed_{true};     // true = được phép publish. Cổng bật -> khởi đầu false.
    float arm_hold_timer_ = 0.0f;       // giữ R1+R2 liên tục bao lâu (lớp 1)
    bool  arm_intent_latched_ = false;  // đã giữ đủ -> chốt vĩnh viễn
    std::atomic<int64_t> last_foreign_ms_{0};   // mốc thời gian gói lowcmd LẠ gần nhất
    std::atomic<long>    foreign_count_{0};     // tổng số gói lowcmd của built-in đã thấy
    std::atomic<bool>    conflict_flag_{false}; // built-in quay lại lúc đang armed
    // Cửa sổ trượt đếm gói lạ (chống 1 gói lạc gây nhả quyền oan). first/last đọc chéo thread.
    std::atomic<int64_t> conflict_first_ms_{0};
    std::atomic<int64_t> conflict_last_ms_{0};
    std::atomic<int> conflict_run_{0};           // tăng ở thread DDS, reset ở thread chính
    std::atomic<uint32_t> last_foreign_crc_{0};  // chẩn đoán: CRC gói vừa bị coi là "lạ"
    bool conflict_diag_logged_ = false;          // chỉ log chẩn đoán conflict 1 lần
    std::chrono::steady_clock::time_point disarmed_start_time_;  // để tính uptime kDisarmed

    // ── Voice tăng/giảm tốc (edge) ──
    bool prev_fast_ = false;
    bool speed_voice_inited_ = false;

    // ── Voice khởi động phát TRỄ ──
    // Built-in (.161) tự phát "Development Mode" khi giữ L2+R2 vào dev mode; nếu phát
    // voice_startup ngay lúc arm sẽ đè lên nhau. Hoãn ngần startup_voice_delay_s giây.
    bool startup_voice_pending_ = false;
    std::chrono::steady_clock::time_point startup_voice_at_;

    // ── P0-3: giám sát pin ──
    BatteryMonitor battery_;
    float battery_warn_timer_ = 0.0f;   // throttle voice cảnh báo
    bool  battery_active_ = false;      // đang ở trạng thái cần giám sát pin (edge)
    bool  battery_critical_done_ = false;

    // Khai báo cuối để MusicPlayer bị hủy trước notifier/atomic mà callback của nó sử dụng.
    IntegrationNotifier integration_notifier_;
    std::atomic<bool> audio_busy_{false};
    long integration_last_tick_ = -1000;
    MusicPlayer music_;
};
