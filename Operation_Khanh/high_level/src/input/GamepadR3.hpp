#pragma once
/**
 * GamepadR3.hpp — Quản lý lệnh từ tay cầm Unitree R3-1.
 * Layout phím theo chuẩn Unitree SDK.
 */

#include <chrono>
#include <cstring>

#include <unitree/idl/hg/LowState_.hpp>

#include "../config/Tuning.hpp"
#include "InputCommand.hpp"

// ── Cấu trúc dữ liệu remote R3-1 ─────────
typedef union {
    struct {
        uint8_t R1 : 1; uint8_t L1 : 1; uint8_t start : 1; uint8_t select : 1;
        uint8_t R2 : 1; uint8_t L2 : 1; uint8_t F1 : 1;    uint8_t F2 : 1;
        uint8_t A : 1;  uint8_t B : 1;  uint8_t X : 1;     uint8_t Y : 1;
        uint8_t up : 1; uint8_t right : 1; uint8_t down : 1; uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct {
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx, rx, ry, L2, ly;
    uint8_t idle[16];
} xRockerBtnDataStruct;

typedef union {
    xRockerBtnDataStruct RF_RX;
    uint8_t buff[40];
} RemoteDataRx;

class GamepadR3 {
public:
    void Configure(const Tuning& tuning) {
        timeout_ms_ = tuning.remote_timeout_ms;
        last_rx_time_ = std::chrono::steady_clock::now();
    }

    void Update(const unitree_hg::msg::dds_::LowState_& low) {
        std::memcpy(remote_.buff, low.wireless_remote().data(), 40);
        // Không có remote -> buffer toàn 0
        for (int i = 0; i < 40; ++i) {
            if (remote_.buff[i] != 0) {
                last_rx_time_ = std::chrono::steady_clock::now();
                break;
            }
        }
    }

    InputCommand GetCommand() {
        InputCommand cmd;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - last_rx_time_).count();
        cmd.is_active = (ms <= timeout_ms_);

        auto cur = remote_.RF_RX.btn.components;
        auto last = reinterpret_cast<xKeySwitchUnion*>(&last_btn_)->components;

        if (cur.L2 && cur.up && !last.up)     cmd.want_stand_lock = true;
        if (cur.R2 && cur.A && !last.A)       cmd.want_locomotion = true;
        
        if (cur.R1 && cur.up && !last.up)       cmd.want_mimic_key = 2;
        if (cur.R1 && cur.right && !last.right) cmd.want_mimic_key = 3;
        if (cur.R1 && cur.down && !last.down)   cmd.want_mimic_key = 4;
        if (cur.R1 && cur.left && !last.left)   cmd.want_mimic_key = 5;
        if (cur.R1 && cur.A && !last.A)         cmd.want_mimic_key = 6;

        if (cur.R2 && cur.up && !last.up)     cmd.want_fast_speed = true;
        if (cur.R2 && cur.down && !last.down) cmd.want_slow_speed = true;
        if (cur.L2 && cur.X && !last.X)       cmd.want_safe_shutdown = true;
        // E-stop: L2+B (chuẩn Unitree)
        if (cur.L2 && cur.B)                  cmd.want_emergency_stop = true;

        last_btn_ = remote_.RF_RX.btn.value;

        // Deadzone riêng cho từng trục
        auto deadzone = [](float v) { return (std::abs(v) > 0.05f) ? v : 0.0f; };
        float lx = remote_.RF_RX.lx, ly = remote_.RF_RX.ly, rx = remote_.RF_RX.rx;
        cmd.vx  = deadzone(ly);        // tiến/lùi
        cmd.vy  = deadzone(-lx);       // quy ước Unitree: -lx là vy
        cmd.yaw = deadzone(-rx);       // -rx là yaw
        return cmd;
    }

private:
    RemoteDataRx remote_{};
    uint16_t last_btn_ = 0;
    float timeout_ms_ = 1500.0f;
    std::chrono::steady_clock::time_point last_rx_time_;
};
