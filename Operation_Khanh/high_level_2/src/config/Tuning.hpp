#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

// Cấu hình Tuning load từ yaml
struct Tuning {
    // Network
    std::string network_interface = "eth10";

    // Headless
    bool dev_no_keyboard = false;

    // Lọc IMU & khớp
    float imu_gyro_lpf_hz = 20.0f;    // Dải: 15-25 Hz. 0.0 = tắt.
    float joint_vel_lpf_hz = 0.0f;    // Dải: 10-20 Hz. 0.0 = tắt.
    float imu_pitch_trim_deg = 0.0f;  // Bù pitch gravity. Dải: ±1..4°.
    // affects_locomotion: trim cho locomotion + fall-detector. affects_dance: trim cho dance.
    bool imu_pitch_trim_affects_locomotion = true;
    bool imu_pitch_trim_affects_dance      = false;

    // Gains ngoài policy
    float stand_kp_leg   = 200.0f;
    float stand_kp_waist = 200.0f;
    float stand_kp_arm   = 40.0f;
    float stand_kd       = 3.0f;

    // Tỉ lệ scale gains policy (mặc định 1.0)
    float policy_kp_scale = 1.0f;
    float policy_kd_scale = 1.0f;

    // Giới hạn tốc độ
    float slow_vx = 0.5f, slow_vy = 0.3f, slow_yaw = 0.5f, slow_vx_back = 0.4f;
    float fast_vx = 1.0f, fast_vy = 0.5f, fast_yaw = 1.0f, fast_vx_back = 0.5f;

    // Giới hạn gia tốc lệnh
    float cmd_accel_vx  = 2.5f;
    float cmd_accel_vy  = 2.0f;
    float cmd_accel_yaw = 3.0f;

    // Giới hạn giảm tốc (thả stick)
    float cmd_decel_vx  = 1.2f;
    float cmd_decel_vy  = 1.2f;
    float cmd_decel_yaw = 1.5f;

    // Heading-hold: giữ hướng đi thẳng, khử drift yaw.
    bool  heading_hold_enabled      = false;
    float heading_hold_kp           = 0.8f;
    float heading_hold_max_yaw      = 0.4f;
    float heading_hold_move_min     = 0.15f;
    float heading_hold_relatch_gyro = 0.3f;

    // Chuyển trạng thái & blend
    float stand_up_time_s   = 2.5f;
    float blend_time_s      = 0.5f;
    float stand_rate_limit  = 5.0f;
    float lock_rate_limit   = 0.2f;
    float return_rate_limit = 1.5f;
    float stand_lock_spread = 0.08f;
    float return_pos_tol    = 0.12f;

    // Chờ robot dừng hẳn trước khi chuyển trạng thái
    float settle_time_s   = 1.0f;
    float settle_gyro_max = 0.5f;

    // Chờ sau voice "Đã khóa đứng" trước khi ép cứng.
    float stand_lock_warn_s = 1.5f;

    // Chặn khóa cứng ngay sau khi hủy điệu nhảy.
    float dance_abort_lock_block_s = 2.0f;

    // Chặn ngồi ngay sau khi vào stand lock.
    float stand_lock_sit_block_s = 1.0f;

    // Giới hạn tốc độ thay đổi góc khớp policy
    float policy_rate_limit = 40.0f;

    // Phát hiện ngã
    bool  fall_enabled        = true;
    float fall_tilt_deg       = 50.0f;
    float fall_flip_tilt_deg  = 30.0f;
    float fall_flip_gyro      = 4.0f;   // Ngưỡng phát hiện lurch
    float fall_debounce_ms    = 30.0f;

    // An toàn tốc độ khớp (vung loạn -> Damping).
    bool  joint_speed_guard_enabled = true;
    float joint_speed_limit         = 25.0f;   // rad/s
    float joint_speed_debounce_ms   = 30.0f;

    // Cấu hình ngồi ghế.
    // desc: tư thế lúc hạ, tự cân bằng (chưa chắc có ghế).
    // rest: tư thế ngồi trên ghế (đã có ghế đỡ).
    float sit_hip_deg         = 150.0f;  // [DESC] Hông lúc hạ (130-160)
    float sit_knee_deg        = 100.0f;  // [DESC] Gối lúc hạ (90-115)
    float sit_spread          = 0.08f;   // [DESC] Dạng chân nhẹ
    float sit_rest_hip_deg    = 87.0f;   // [REST] Hông cuối
    float sit_rest_knee_deg   = 82.0f;   // [REST] Gối cuối
    float sit_rest_spread     = 0.45f;   // [REST] Dạng chân cuối
    float sit_rest_hip_yaw    = 0.45f;   // [REST] Xoay mũi chân cuối
    float sit_lean_deg        = 58.0f;   // Đổ thân lúc hạ
    float sit_seated_lean_deg = 20.0f;   // Đổ thân cuối
    float sit_arm_forward     = -1.5f;   // Vai vươn trước
    float sit_arm_elbow       = 0.3f;    // Khuỷu duỗi
    // Tư thế tay cuối
    float sit_seated_arm_pitch = 0.0f;   // Vai lúc ngồi
    float sit_seated_arm_elbow = 0.42f;  // Khuỷu lúc ngồi
    float sit_ankle_gravity_gain = 0.4f; // Bù cổ chân bám sàn
    float sit_descent_time_s  = 4.0f;
    float sit_settle_time_s   = 1.5f;    // Thời gian settle
    float sit_hold_s          = 0.5f;
    bool  sit_release_after   = false;
    float sit_kp_leg          = 200.0f;
    float sit_kd              = 3.0f;
    float sit_rate_limit      = 3.0f;
    float sit_gather_time_s   = 1.5f;

    // Đứng lên / Nằm xuống (quỹ đạo ghi từ record_motion.cpp, PD thuần)
    std::string getup_motion_file   = "motions/getup.npz";
    std::string liedown_motion_file = "motions/liedown.npz";
    float getup_kp_leg    = 220.0f;
    float getup_kp_waist  = 200.0f;
    float getup_kp_arm    = 60.0f;
    float getup_kd        = 4.0f;
    float getup_rate_limit    = 6.0f;
    float liedown_rate_limit  = 4.0f;
    float getup_blend_time_s   = 0.6f;   // Blend-in getup
    float liedown_blend_time_s = 0.6f;
    float getup_speed     = 1.0f;
    float liedown_speed   = 1.0f;
    float getup_liedown_block_s = 1.0f;  // Chặn bấm dội
    // Bù cổ chân giữ bàn chân phẳng với sàn
    float getup_ankle_gravity_gain = 0.5f;
    // Ngưỡng nghiêng coi là nằm (độ)
    float lying_tilt_deg = 60.0f;
    std::string voice_get_up   = "";     // Voice get up
    std::string voice_lie_down = "";

    // Chế độ an toàn khi mất điều khiển
    bool  safe_stop_enabled   = true;
    float safe_stop_debounce_ms = 300.0f;

    // P0-1: Cổng chống xung đột với built-in
    bool  arm_gate_enabled        = false;
    bool  arm_require_button      = true;    // Yêu cầu combo R1+R2 để arm
    float arm_hold_s              = 5.0f;    // Thời gian giữ combo
    float arm_silence_ms          = 400.0f;  // Thời gian built-in im để nhả quyền
    bool  arm_require_seen_builtin = true;   // Chỉ arm sau khi nghe built-in
    int   arm_min_foreign_seen    = 5;       // Số gói built-in tối thiểu
    // Phát hiện conflict để nhả quyền
    int   arm_conflict_min        = 3;
    float arm_conflict_window_ms  = 150.0f;
    // Tự động nhả quyền khi conflict
    bool  arm_conflict_release    = true;
    // Timeout bypass điều kiện built-in
    float arm_no_builtin_timeout_s = 15.0f;

    // P0-3: Giám sát pin
    bool  battery_monitor_enabled = true;
    std::string battery_topic     = "rt/lf/bmsstate";
    int   battery_warn_pct        = 20;
    int   battery_critical_pct    = 8;
    std::string battery_critical_action = "sit";
    float battery_announce_period_s = 60.0f;
    float battery_stale_s         = 30.0f;

    // Âm thanh tiếng Việt
    bool  voice_enabled    = true;
    float voice_volume     = 0.9f;
    int   voice_speaker_id = 0;
    std::string voice_startup    = "Máy tính phát triển đã sẵn sàng";
    // Hoãn voice khởi động tránh đè voice built-in
    float startup_voice_delay_s  = 0.0f;
    std::string voice_stand_lock = "Đã khóa đứng";
    std::string voice_locomotion = "Bật chế độ đi bộ";
    std::string voice_sit_down   = "Đang ngồi xuống";
    std::string voice_safe_stop  = "Kích hoạt chế độ an toàn";
    std::string voice_mimic[7];
    // Cấu hình câu voice (để rỗng = không phát)
    std::string voice_fast_speed = "";   // Tốc độ nhanh
    std::string voice_slow_speed = "";   // Tốc độ chậm
    std::string voice_conflict   = "";   // Khi conflict
    std::string voice_battery_low      = "";  // Pin thấp
    std::string voice_battery_critical = "";  // Pin cạn
    std::string voice_zero_torque      = "";  // Xả lực

    // Thời gian giữ combo kích hoạt động tác nguy hiểm
    float hold_to_trigger_s = 3.0f;

    // Watchdogs
    float state_timeout_ms       = 1000.0f;
    // Timeout mất kết nối remote
    float remote_timeout_ms      = 3000.0f;
    // Khôi phục remote
    float remote_recover_ms      = 200.0f;
    bool  remote_require_neutral = true;
    float x11_release_ms         = 2500.0f;

    // Điệu nhảy (mimic)
    static constexpr int kMaxDances = 7;
    std::string dance_folder[kMaxDances];
    float dance_speed[kMaxDances]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};  // Dải bám: 0.6 - 0.85
    float dance_volume[kMaxDances] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // Bù pitch riêng cho từng bài nhảy
    float dance_trim_deg[kMaxDances] = {};
    bool  dance_trim_configured[kMaxDances] = {};

    float DanceTrimDeg(int dance_key) const {
        const int idx = dance_key - 2;
        return (idx >= 0 && idx < kMaxDances && dance_trim_configured[idx])
                   ? dance_trim_deg[idx]
                   : imu_pitch_trim_deg;
    }

    int   dance_start_frame = -1;
    int   dance_start_search_frames = 200;
    float mimic_announce_delay_s = 2.0f;    // Delay khi không có voice
    float mimic_announce_min_s = 0.3f;      // Chờ voice tối thiểu
    float mimic_announce_timeout_s = 20.0f; // Chờ voice tối đa

    // Soft-start: đưa robot vào tư thế mở màn (0.8 - 2.0 s)
    float mimic_warmup_s = 1.2f;

    // Soft-stop: đưa robot về đứng thẳng (0.8 - 2.0 s)
    float mimic_cooldown_s = 1.0f;

    // Điều kiện bàn giao mimic -> locomotion (đứng yên)
    float mimic_handover_tilt  = 0.12f;  // sin(nghiêng), ~7°
    float mimic_handover_gyro  = 0.5f;
    float mimic_handover_min_s = 0.3f;   // Chờ tối thiểu trước khi giao sớm
    float mimic_handover_max_s = 3.0f;   // Chờ yên tối đa

    // CSV telemetry sim2real
    bool  mimic_telemetry_enabled = true;
    int   mimic_telemetry_hz = 100;       // Tần số telemetry (Hz)
    float mimic_telemetry_post_s = 5.0f;  // Ghi sau handover
    std::string mimic_telemetry_dir = "logs/telemetry";
    
    std::string flat_model  = "policy_r1_1.onnx";

    // Khớp đầu
    float head_yaw_kp = 20.0f, head_yaw_kd = 10.0f;
    float head_pitch_kp = 10.0f, head_pitch_kd = 1.0f;

    // Nạp cấu hình từ file yaml
    bool LoadFromFile(const std::string& path);

private:
    void Apply(const std::string& key, const std::string& value, bool& known);
};
