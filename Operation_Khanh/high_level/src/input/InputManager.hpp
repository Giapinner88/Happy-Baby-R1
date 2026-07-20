#pragma once
/**
 * InputManager.hpp — Quản lý lệnh đầu vào từ gamepad R3-1 và bàn phím X11.
 */

#include <chrono>
#include <cmath>

#include "../config/Tuning.hpp"
#include "GamepadR3.hpp"
#include "InputCommand.hpp"
#include "KeyboardX11.hpp"

class InputManager {
public:
    void Configure(const Tuning& tuning) {
        tuning_ = &tuning;
        gamepad_.Configure(tuning);
        keyboard_.Configure(tuning);
        last_update_ = std::chrono::steady_clock::now();
    }

    void Start() { keyboard_.Start(); }
    void Stop() { keyboard_.Stop(); }
    KeyboardX11& keyboard() { return keyboard_; }

    void Update(const unitree_hg::msg::dds_::LowState_& low) { gamepad_.Update(low); }

    InputCommand GetMergedCommand() {
        InputCommand g = gamepad_.GetCommand();
        InputCommand k = keyboard_.GetCommand();
        const Tuning& t = *tuning_;

        InputCommand m;
        m.want_emergency_stop = g.want_emergency_stop || k.want_emergency_stop;
        if (!m.want_emergency_stop) {
            // Khẩn cấp hủy các lệnh khác
            m.want_stand_lock = g.want_stand_lock || k.want_stand_lock;
            m.want_locomotion = g.want_locomotion || k.want_locomotion;
            m.want_mimic_key  = g.want_mimic_key ? g.want_mimic_key : k.want_mimic_key;
            m.want_reset      = g.want_reset || k.want_reset;
            m.want_safe_shutdown      = g.want_safe_shutdown || k.want_safe_shutdown;
        }
        // Robot chạy khi có ít nhất 1 nguồn sống
        m.is_active = g.is_active || k.is_active;

        // Chế độ tốc độ: remote gán cứng, bàn phím toggle
        if (g.want_fast_speed) is_fast_ = true;
        if (g.want_slow_speed) is_fast_ = false;
        if (k.want_speed_toggle) is_fast_ = !is_fast_;

        float lim_vx   = is_fast_ ? t.fast_vx : t.slow_vx;
        float lim_back = is_fast_ ? t.fast_vx_back : t.slow_vx_back;
        float lim_vy   = is_fast_ ? t.fast_vy : t.slow_vy;
        float lim_yaw  = is_fast_ ? t.fast_yaw : t.slow_yaw;

        auto scale_vx = [&](float in) { return in * (in < 0.0f ? lim_back : lim_vx); };

        // Chọn nguồn ưu tiên (Gamepad > Bàn phím)
        float raw_vx, raw_vy, raw_yaw;
        if (std::abs(g.vx) > 0.05f || std::abs(g.vy) > 0.05f || std::abs(g.yaw) > 0.05f) {
            raw_vx = scale_vx(g.vx);
            raw_vy = g.vy * lim_vy;
            raw_yaw = g.yaw * lim_yaw;
        } else {
            raw_vx = scale_vx(k.vx);
            raw_vy = k.vy * lim_vy;
            raw_yaw = k.yaw * lim_yaw;
        }

        // Command smoother
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_update_).count() / 1e6f;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;
        last_update_ = now;

        auto slew = [](float cur, float target, float accel, float dt) {
            float max_step = accel * dt;
            float diff = target - cur;
            if (diff > max_step) return cur + max_step;
            if (diff < -max_step) return cur - max_step;
            return target;
        };
        vx_ = slew(vx_, raw_vx, t.cmd_accel_vx, dt);
        vy_ = slew(vy_, raw_vy, t.cmd_accel_vy, dt);
        yaw_ = slew(yaw_, raw_yaw, t.cmd_accel_yaw, dt);

        m.vx = vx_;
        m.vy = vy_;
        m.yaw = yaw_;
        return m;
    }

    // Xóa lệnh vận tốc
    void ZeroVelocity() { vx_ = vy_ = yaw_ = 0.0f; }

    bool IsFastMode() const { return is_fast_; }

private:
    const Tuning* tuning_ = nullptr;
    GamepadR3 gamepad_;
    KeyboardX11 keyboard_;
    bool is_fast_ = false;
    float vx_ = 0.0f, vy_ = 0.0f, yaw_ = 0.0f;
    std::chrono::steady_clock::time_point last_update_;
};
