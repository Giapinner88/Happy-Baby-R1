#pragma once

// Cấu trúc chứa lệnh điều khiển chuẩn hóa từ gamepad / bàn phím
struct InputCommand {
    bool want_stand_lock = false;
    bool want_locomotion = false;
    int  want_mimic_key = 0; // 0 = no dance, 2-8 = dance key
    bool want_reset = false;
    bool want_safe_shutdown = false;  // giữ L2+Trái (hoặc phím 9): ngồi ghế
    bool want_get_up_down = false;  // L2+X: Đứng lên hoặc nằm xuống tùy trạng thái

    bool want_emergency_stop = false;
    bool want_zero_torque = false;    // L2+Y: Xả lực hoàn toàn (chỉ từ kIdle)

    bool want_fast_speed = false;
    bool want_slow_speed = false;
    bool want_speed_toggle = false;

    // Cử chỉ tay: 0 = không; 1..8 = slot gesture (double-click nút R3 không kèm modifier).
    // Map nút: 1=Up 2=Down 3=Left 4=Right 5=A 6=B 7=X 8=Y.
    int want_gesture = 0;

    // Bật/tắt teleop live sau khi GIỮ L2 + Phải đủ teleop_toggle_hold_s.
    // Unified V10 chỉ reserve sự kiện này; nó không mở UDP/ghi đè tay.
    bool want_teleop_toggle = false;

    float vx = 0.0f;
    float vy = 0.0f;
    float yaw = 0.0f;

    bool is_active = true;
};
