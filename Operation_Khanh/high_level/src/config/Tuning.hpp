#pragma once
/**
 * Tuning.hpp — Các tham số có thể tune.
 * Các giá trị này có thể được ghi đè bởi config/tuning.yaml khi chạy.
 */

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

struct Tuning {
    // ── Mạng / giao tiếp ─────────────────────────────────
    std::string network_interface = "eth10"; // argv[1] nạp đè nếu có

    // ── Dev-mode / điều khiển ────────────────────────────
    // true = KHÔNG mở cửa sổ bàn phím X11, chạy thuần gamepad R3-1
    bool dev_no_keyboard = false;

    // ── IMU filter (estimation) ──────────────────────────
    // LPF bậc 1 chạy ở 500Hz trên gyro TRƯỚC khi lấy mẫu 50Hz cho obs. 0 = tắt.
    float imu_gyro_lpf_hz = 20.0f;
    // LPF cho vận tốc khớp dq.
    float joint_vel_lpf_hz = 0.0f;
    // Trim pitch (độ) áp vào quaternion TRƯỚC khi tính projected_gravity
    float imu_pitch_trim_deg = 0.0f;

    // ── Gains khi KHÔNG chạy policy (đứng dậy / khóa đứng / returning) ──
    float stand_kp_leg   = 200.0f;
    float stand_kp_waist = 200.0f;
    float stand_kp_arm   = 40.0f;
    float stand_kd       = 3.0f;

    // ── Thí nghiệm scale gains policy (mặc định 1.0 = đúng train) ──
    float policy_kp_scale = 1.0f;
    float policy_kd_scale = 1.0f;

    // ── Giới hạn tốc độ lệnh ──
    float slow_vx = 0.5f, slow_vy = 0.3f, slow_yaw = 0.5f, slow_vx_back = 0.4f;
    float fast_vx = 1.0f, fast_vy = 0.5f, fast_yaw = 1.0f, fast_vx_back = 0.5f;

    // Giới hạn gia tốc lệnh (command smoother, m/s² và rad/s²)
    float cmd_accel_vx  = 2.5f;
    float cmd_accel_vy  = 2.0f;
    float cmd_accel_yaw = 3.0f;

    // ── Chuyển trạng thái / blend ────────────────────────
    float stand_up_time_s   = 2.5f;  // thời gian nội suy đứng dậy
    float blend_time_s      = 0.5f;  // blend tư thế khi bật policy (chống giật)
    float stand_rate_limit  = 5.0f;  // rad/s khi đứng dậy
    float lock_rate_limit   = 0.2f;  // rad/s khi giữ STAND_LOCK (dang chân từ từ)
    float return_rate_limit = 1.5f;  // rad/s khi về default trước dance
    float stand_lock_spread = 0.08f; // dang hip_roll ra 2 bên khi khóa đứng
    float return_pos_tol    = 0.12f; // rad — ngưỡng "đã về default"

    // ── DỪNG DƯỚI POLICY trước khi bàn giao sang giữ-cứng ────────
    float settle_time_s   = 1.0f;  // tối thiểu bấy nhiêu giây đứng yên dưới policy
    float settle_gyro_max = 0.5f;  // rad/s — |gyro| phải dưới mức này mới coi là yên

    // ── Rate limit khi POLICY chạy ───────────────────────
    float policy_rate_limit = 40.0f;

    // ── Fall detector ────────────────────────────────────
    bool  fall_enabled        = true;
    float fall_tilt_deg       = 50.0f;  // nghiêng quá góc này -> ngã
    float fall_flip_tilt_deg  = 30.0f;  // nghiêng vừa + xoay nhanh -> ngã
    float fall_flip_gyro      = 6.0f;   // rad/s (norm 3 trục)
    float fall_debounce_ms    = 30.0f;

    // ── Nút 9 "Ngồi ghế" ──
    float sit_knee_deg        = 90.0f;  // góc gối khi ngồi (chỉnh theo chiều cao ghế)
    float sit_descent_time_s  = 4.0f;   // thời gian hạ xuống (chậm/êm)
    float sit_hold_s          = 0.5f;   // giữ lực ở tư thế ngồi trước khi xử lý cuối
    bool  sit_release_after   = false;  // true = xả lực (damp) sau khi ngồi; false = GIỮ tư thế
    // Gains + rate RIÊNG cho lúc ngồi
    float sit_kp_leg          = 200.0f; // stiffness chân lúc ngồi
    float sit_kd              = 3.0f;   // damping mọi khớp lúc ngồi
    float sit_rate_limit      = 3.0f;   // rad/s giới hạn slew lúc ngồi
    // Ngồi 3 pha: THU CHÂN -> HẠ -> GIỮ.
    float sit_gather_time_s   = 1.5f;   // thời gian thu chân trước khi hạ
    float sit_stance          = 0.0f;   // hip_roll đích khi thu (rad, ≥0 = khép; 0 = bỏ dang)
    bool  sit_lateral_flat    = true;   // bù ankle_roll = -hip_roll (bàn chân phẳng ngang)

    // ── Safe-stop: mất TOÀN BỘ input (gamepad + bàn phím), DDS còn sống ──
    bool  safe_stop_enabled   = true;
    float safe_stop_debounce_ms = 300.0f; // giữ mất-input liên tục bấy nhiêu mới kích

    // ── Voice tiếng Việt (phát qua loa robot) ────────────────────
    bool  voice_enabled    = true;
    float voice_volume     = 0.9f;   // 0..1
    int   voice_speaker_id = 0;      // id giọng TTS (thử đổi nếu firmware có nhiều giọng)
    std::string voice_startup    = "Máy tính phát triển đã sẵn sàng";
    std::string voice_stand_lock = "Đã khóa đứng";
    std::string voice_locomotion = "Bật chế độ đi bộ";
    std::string voice_sit_down   = "Đang ngồi xuống";
    std::string voice_safe_stop  = "Kích hoạt chế độ an toàn";
    std::string voice_mimic[7];      // voice_mimic_2..8 -> index 0..6

    // ── Watchdog ─────────────────────────────────────────
    float state_timeout_ms  = 1000.0f; // mất lowstate -> damping
    float remote_timeout_ms = 1500.0f; // mất gamepad
    float x11_release_ms    = 2500.0f; // mất X11 -> nhả phím di chuyển

    // ── Dance / mimic ────────────────────────────────────
    // Cấu hình mảng 7 điệu nhảy tương ứng với phím 2 đến 8.
    static constexpr int kMaxDances = 7;
    std::string dance_folder[kMaxDances];
    float dance_speed[kMaxDances]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    float dance_volume[kMaxDances] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    int   dance_start_frame = -1;
    int   dance_start_search_frames = 200;
    float mimic_announce_delay_s = 2.0f;
    
    std::string flat_model  = "policy_r1_1.onnx"; // trong policies/flat/ (phải là file có thật)

    // ── Đầu (không do policy điều khiển) ─────────────────
    float head_yaw_kp = 20.0f, head_yaw_kd = 10.0f;
    float head_pitch_kp = 10.0f, head_pitch_kd = 1.0f;

    // Nạp file "key: value".
    bool LoadFromFile(const std::string& path);

private:
    void Apply(const std::string& key, const std::string& value, bool& known);
};
