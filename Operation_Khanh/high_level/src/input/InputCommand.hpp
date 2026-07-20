#pragma once
/**
 * InputCommand.hpp — Lệnh đầu vào đã chuẩn hóa từ mọi nguồn (gamepad/bàn phím).
 */

struct InputCommand {
    // Chuyển trạng thái (edge — chỉ true đúng 1 lần mỗi lần bấm)
    bool want_stand_lock = false;
    bool want_locomotion = false;
    int want_mimic_key = 0; // 0 = no dance, 2..8 = dance key (gamepad 2..6)
    bool want_reset = false;
    bool want_safe_shutdown = false;      // L2+X / phím 9: ngồi ghế feet-flat rồi GIỮ

    // Khẩn cấp (level — giữ nút là giữ true)
    bool want_emergency_stop = false;

    // Tốc độ
    bool want_fast_speed = false;
    bool want_slow_speed = false;
    bool want_speed_toggle = false;

    // Vận tốc thô [-1, 1] (InputManager scale + làm mượt)
    float vx = 0.0f;
    float vy = 0.0f;
    float yaw = 0.0f;

    // false = nguồn này đã mất tín hiệu (phục vụ watchdog)
    bool is_active = true;
};
