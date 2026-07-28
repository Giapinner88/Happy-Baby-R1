#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <unitree/idl/hg/LowState_.hpp>

#include "../config/Tuning.hpp"
#include "InputCommand.hpp"

// Cấu trúc dữ liệu tay cầm Unitree R3-1 (40 byte).
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

enum class RemoteLinkState {
    kWaiting,
    kHealthy,
    kSuspect,
    kLost,
    kRecovering,
};

// Đọc lệnh R3-1 và phân biệt gián đoạn ngắn (SUSPECT) với mất remote (LOST).
class GamepadR3 {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    void Configure(const Tuning& tuning) { ConfigureAt(tuning, Clock::now()); }

    void ConfigureAt(const Tuning& tuning, TimePoint now) {
        timeout_ms_ = std::max(1.0f, tuning.remote_timeout_ms);
        recover_ms_ = std::max(0.0f, tuning.remote_recover_ms);
        require_neutral_ = tuning.remote_require_neutral;
        hold_s_ = tuning.hold_to_trigger_s;
        last_valid_time_ = now;
        zero_started_ = now;
        recover_started_ = now;
        state_ = RemoteLinkState::kWaiting;
        packet_valid_ = false;
        recover_timer_active_ = false;
        last_btn_ = 0;
        ResetHeldActions();
    }

    void Update(const unitree_hg::msg::dds_::LowState_& low) {
        UpdateAt(low, Clock::now());
    }

    // Cho phép unit test dùng mốc thời gian giả; runtime luôn gọi Update().
    void UpdateAt(const unitree_hg::msg::dds_::LowState_& low, TimePoint now) {
        std::memcpy(remote_.buff, low.wireless_remote().data(), sizeof(remote_.buff));
        packet_valid_ = false;
        for (uint8_t byte : remote_.buff) {
            if (byte != 0) { packet_valid_ = true; break; }
        }

        if (!packet_valid_) {
            ResetHeldActions();
            recover_timer_active_ = false;
            if (state_ == RemoteLinkState::kHealthy) {
                zero_started_ = now;
                SetState(RemoteLinkState::kSuspect, now);
            } else if (state_ == RemoteLinkState::kWaiting &&
                       ElapsedMs(last_valid_time_, now) >= timeout_ms_) {
                SetState(RemoteLinkState::kLost, now);
            } else if (state_ == RemoteLinkState::kSuspect &&
                       ElapsedMs(last_valid_time_, now) >= timeout_ms_) {
                SetState(RemoteLinkState::kLost, now);
            } else if (state_ == RemoteLinkState::kRecovering) {
                SetState(RemoteLinkState::kLost, now);
            }
            return;
        }

        last_valid_time_ = now;
        if (state_ == RemoteLinkState::kHealthy) return;

        if (state_ == RemoteLinkState::kSuspect) {
            SetState(RemoteLinkState::kHealthy, now);
            return;
        }

        if (state_ != RemoteLinkState::kRecovering) {
            recover_timer_active_ = false;
            SetState(RemoteLinkState::kRecovering, now);
        }

        const bool neutral = ControlsNeutral();
        if (require_neutral_ && !neutral) {
            recover_timer_active_ = false;
            return;
        }
        if (!recover_timer_active_) {
            recover_started_ = now;
            recover_timer_active_ = true;
        }
        if (ElapsedMs(recover_started_, now) >= recover_ms_) {
            // Đồng bộ edge detector trước khi nhận lại lệnh.
            last_btn_ = remote_.RF_RX.btn.value;
            ResetHeldActions();
            SetState(RemoteLinkState::kHealthy, now);
        }
    }

    // Arm run_r1 bằng R1+R2 sau khi built-in đã vào dev mode.
    bool HoldingArmCombo() const {
        if (state_ != RemoteLinkState::kHealthy || !packet_valid_) return false;
        const auto c = remote_.RF_RX.btn.components;
        return c.R1 && c.R2;
    }

    InputCommand GetCommand() {
        InputCommand cmd;
        cmd.is_active = InputActive();

        // SUSPECT chưa báo mất input, nhưng gói 0 không được tạo lệnh hoặc tăng hold timer.
        if (state_ != RemoteLinkState::kHealthy || !packet_valid_) {
            if (packet_valid_) last_btn_ = remote_.RF_RX.btn.value;
            ResetHeldActions();
            return cmd;
        }

        const auto cur = remote_.RF_RX.btn.components;
        const auto last = reinterpret_cast<xKeySwitchUnion*>(&last_btn_)->components;

        if (cur.L2 && cur.up && !last.up)     cmd.want_stand_lock = true;
        if (cur.R2 && cur.A && !last.A)       cmd.want_locomotion = true;

        if (cur.R1 && cur.up && !last.up)       cmd.want_mimic_key = 2;
        if (cur.R1 && cur.right && !last.right) cmd.want_mimic_key = 3;
        if (cur.R1 && cur.down && !last.down)   cmd.want_mimic_key = 4;
        if (cur.R1 && cur.left && !last.left)   cmd.want_mimic_key = 5;
        if (cur.R1 && cur.A && !last.A)         cmd.want_mimic_key = 6;

        if (cur.R2 && cur.up && !last.up)     cmd.want_fast_speed = true;
        if (cur.R2 && cur.down && !last.down) cmd.want_slow_speed = true;
        if (cur.L2 && cur.B)                  cmd.want_emergency_stop = true;
        if (cur.L2 && cur.Y && !last.Y)       cmd.want_zero_torque = true;

        const auto now = Clock::now();
        auto held = [&](bool pressed, TimePoint& start, bool& active, bool& fired) {
            if (!pressed) { active = false; fired = false; return false; }
            if (!active) { active = true; fired = false; start = now; return false; }
            if (!fired && std::chrono::duration<float>(now - start).count() >= hold_s_) {
                fired = true;
                return true;
            }
            return false;
        };
        if (held(cur.L2 && cur.left, l2left_start_, l2left_active_, l2left_fired_))
            cmd.want_safe_shutdown = true;
        if (held(cur.L2 && cur.X, l2x_start_, l2x_active_, l2x_fired_))
            cmd.want_get_up_down = true;

        last_btn_ = remote_.RF_RX.btn.value;
        auto deadzone = [](float value) { return std::abs(value) > 0.05f ? value : 0.0f; };
        cmd.vx = deadzone(remote_.RF_RX.ly);
        cmd.vy = deadzone(-remote_.RF_RX.lx);
        cmd.yaw = deadzone(-remote_.RF_RX.rx);
        return cmd;
    }

    RemoteLinkState link_state() const { return state_; }
    bool packet_valid() const { return packet_valid_; }
    bool InputActive() const {
        return state_ == RemoteLinkState::kHealthy || state_ == RemoteLinkState::kSuspect;
    }
    const char* LinkStateName() const { return StateName(state_); }

private:
    static double ElapsedMs(TimePoint from, TimePoint to) {
        return std::chrono::duration<double, std::milli>(to - from).count();
    }

    static const char* StateName(RemoteLinkState state) {
        switch (state) {
            case RemoteLinkState::kWaiting:    return "WAITING";
            case RemoteLinkState::kHealthy:    return "HEALTHY";
            case RemoteLinkState::kSuspect:    return "SUSPECT";
            case RemoteLinkState::kLost:       return "LOST";
            case RemoteLinkState::kRecovering: return "RECOVERING";
        }
        return "UNKNOWN";
    }

    bool ControlsNeutral() const {
        if (remote_.RF_RX.btn.value != 0) return false;
        return std::abs(remote_.RF_RX.lx) <= 0.05f &&
               std::abs(remote_.RF_RX.ly) <= 0.05f &&
               std::abs(remote_.RF_RX.rx) <= 0.05f &&
               std::abs(remote_.RF_RX.ry) <= 0.05f &&
               std::abs(remote_.RF_RX.L2) <= 0.05f;
    }

    void SetState(RemoteLinkState next, TimePoint now) {
        if (next == state_) return;
        const RemoteLinkState previous = state_;
        state_ = next;
        if (next == RemoteLinkState::kSuspect) {
            std::cout << "[R3] REMOTE_SUSPECT zero_ms=0\n";
        } else if (next == RemoteLinkState::kLost) {
            std::cout << "[R3] REMOTE_LOST zero_ms="
                      << static_cast<long>(ElapsedMs(last_valid_time_, now))
                      << " timeout_ms=" << static_cast<long>(timeout_ms_) << "\n";
        } else if (next == RemoteLinkState::kRecovering) {
            std::cout << "[R3] REMOTE_RECOVERING require_neutral="
                      << (require_neutral_ ? 1 : 0) << "\n";
        } else if (next == RemoteLinkState::kHealthy) {
            const long gap_ms = previous == RemoteLinkState::kSuspect
                ? static_cast<long>(ElapsedMs(zero_started_, now)) : 0;
            std::cout << "[R3] REMOTE_RECOVERED gap_ms=" << gap_ms << "\n";
        }
    }

    void ResetHeldActions() {
        l2x_active_ = l2x_fired_ = false;
        l2left_active_ = l2left_fired_ = false;
    }

    RemoteDataRx remote_{};
    uint16_t last_btn_ = 0;
    float timeout_ms_ = 3000.0f;
    float recover_ms_ = 200.0f;
    float hold_s_ = 3.0f;
    bool require_neutral_ = true;
    bool packet_valid_ = false;
    bool recover_timer_active_ = false;
    RemoteLinkState state_ = RemoteLinkState::kWaiting;
    TimePoint last_valid_time_{};
    TimePoint zero_started_{};
    TimePoint recover_started_{};

    TimePoint l2x_start_{}, l2left_start_{};
    bool l2x_active_ = false, l2x_fired_ = false;
    bool l2left_active_ = false, l2left_fired_ = false;
};
