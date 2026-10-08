#include <array>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>

#include "input/GamepadR3.hpp"

using LowState_ = unitree_hg::msg::dds_::LowState_;

namespace {

LowState_ Packet(uint16_t buttons = 0) {
    LowState_ low;
    std::array<uint8_t, 40> bytes{};
    bytes[0] = 0x55;
    bytes[1] = 0x51;
    bytes[2] = static_cast<uint8_t>(buttons & 0xff);
    bytes[3] = static_cast<uint8_t>(buttons >> 8);
    low.wireless_remote(bytes);
    return low;
}

LowState_ ZeroPacket() {
    LowState_ low;
    low.wireless_remote(std::array<uint8_t, 40>{});
    return low;
}

GamepadR3::TimePoint At(int milliseconds) {
    return GamepadR3::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

}  // namespace

// Standard assert disappears under NDEBUG; CTest also runs in Release.
#undef assert
#define assert(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

int main() {
    Tuning tuning;
    tuning.remote_timeout_ms = 3000.0f;
    tuning.remote_recover_ms = 200.0f;
    tuning.remote_require_neutral = true;

    GamepadR3 pad;
    pad.ConfigureAt(tuning, At(0));
    const LowState_ neutral = Packet();
    const LowState_ zero = ZeroPacket();

    // Startup: cần remote hợp lệ và trung tính ổn định 200 ms.
    pad.UpdateAt(neutral, At(1));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    assert(!pad.GetCommandAt(At(1)).is_active);
    pad.UpdateAt(neutral, At(200));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    pad.UpdateAt(neutral, At(201));
    assert(pad.link_state() == RemoteLinkState::kHealthy);
    assert(pad.GetCommandAt(At(201)).is_active);

    // Dropout 2,999 s chưa bị coi là mất remote.
    pad.UpdateAt(zero, At(210));
    assert(pad.link_state() == RemoteLinkState::kSuspect);
    assert(pad.GetCommandAt(At(210)).is_active);
    pad.UpdateAt(zero, At(3200));
    assert(pad.link_state() == RemoteLinkState::kSuspect);
    pad.UpdateAt(neutral, At(3201));
    assert(pad.link_state() == RemoteLinkState::kHealthy);

    // Mất trên 3 s: LOST; reconnect khi còn giữ nút không được nhận lệnh.
    pad.UpdateAt(zero, At(3210));
    pad.UpdateAt(zero, At(6202));
    assert(pad.link_state() == RemoteLinkState::kLost);
    assert(!pad.GetCommandAt(At(6202)).is_active);

    const LowState_ select_held = Packet(0x0008);
    pad.UpdateAt(select_held, At(6210));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    assert(!pad.GetCommandAt(At(6210)).is_active);
    pad.UpdateAt(neutral, At(6220));
    pad.UpdateAt(neutral, At(6419));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    pad.UpdateAt(neutral, At(6420));
    assert(pad.link_state() == RemoteLinkState::kHealthy);

    // Dropout ngắn không được phát lại edge của nút đang giữ.
    const LowState_ stand_combo = Packet(0x1020);  // L2 + Up
    pad.UpdateAt(neutral, At(6500));
    (void)pad.GetCommandAt(At(6500));
    pad.UpdateAt(stand_combo, At(6510));
    assert(pad.GetCommandAt(At(6510)).want_stand_lock);
    pad.UpdateAt(zero, At(6520));
    assert(!pad.GetCommandAt(At(6520)).want_stand_lock);
    pad.UpdateAt(stand_combo, At(6530));
    assert(pad.link_state() == RemoteLinkState::kHealthy);
    assert(!pad.GetCommandAt(At(6530)).want_stand_lock);

    // Bare START toggles teleop authority on the press edge, once per press.
    // Motion still needs the Quest trigger deadman, and revoking must be instant.
    pad.ConfigureAt(tuning, At(7000));
    pad.UpdateAt(neutral, At(7001));
    pad.UpdateAt(neutral, At(7201));
    assert(pad.link_state() == RemoteLinkState::kHealthy);
    const LowState_ start_held = Packet(0x0004);  // START (bare)
    pad.UpdateAt(start_held, At(7210));
    assert(pad.GetCommandAt(At(7210)).want_teleop_toggle);
    // Holding does not re-fire.
    for (int64_t ms = 7220; ms <= 7720; ms += 100) {
        pad.UpdateAt(start_held, At(ms));
        assert(!pad.GetCommandAt(At(ms)).want_teleop_toggle);
    }
    // Release and press again: a new edge toggles again.
    pad.UpdateAt(neutral, At(7730));
    (void)pad.GetCommandAt(At(7730));
    pad.UpdateAt(start_held, At(7740));
    assert(pad.GetCommandAt(At(7740)).want_teleop_toggle);

    // START with a modifier is a different (future) combo: it must never toggle
    // teleop, no matter how long it is held.
    pad.UpdateAt(neutral, At(8000));
    (void)pad.GetCommandAt(At(8000));
    const LowState_ start_with_mod = Packet(0x0024);  // L2 + START
    for (int64_t ms = 8010; ms <= 9010; ms += 100) {
        pad.UpdateAt(start_with_mod, At(ms));
        assert(!pad.GetCommandAt(At(ms)).want_teleop_toggle);
    }
    // Releasing only the modifier while START stays down is not a START press.
    pad.UpdateAt(start_held, At(9020));
    assert(!pad.GetCommandAt(At(9020)).want_teleop_toggle);
    pad.UpdateAt(neutral, At(9030));
    (void)pad.GetCommandAt(At(9030));
    pad.UpdateAt(start_held, At(9040));
    assert(pad.GetCommandAt(At(9040)).want_teleop_toggle);

    // A stale LowState must not read as a live remote. Application re-feeds the
    // same cached packet every tick when DDS dies, so its non-zero bytes kept the
    // link HEALTHY forever and left R1+R2 frozen in the "held" position.
    GamepadR3 stale_pad;
    stale_pad.ConfigureAt(tuning, At(10000));
    stale_pad.UpdateAt(neutral, At(10010));       // kWaiting -> kRecovering
    stale_pad.UpdateAt(neutral, At(10220));       // đủ remote_recover_ms -> kHealthy
    assert(stale_pad.link_state() == RemoteLinkState::kHealthy);
    const LowState_ arm_combo = Packet(0x0011);   // R1 (bit0) + R2 (bit4)
    stale_pad.UpdateAt(arm_combo, At(10230));
    assert(stale_pad.HoldingArmCombo());

    // Same bytes, but the state carrying them is older than the DDS watchdog.
    stale_pad.UpdateAt(arm_combo, At(10240), /*state_fresh=*/false);
    assert(!stale_pad.packet_valid());
    assert(!stale_pad.HoldingArmCombo());
    assert(stale_pad.link_state() == RemoteLinkState::kSuspect);
    stale_pad.UpdateAt(arm_combo, At(13300), /*state_fresh=*/false);
    assert(stale_pad.link_state() == RemoteLinkState::kLost);
    assert(!stale_pad.HoldingArmCombo());

    return 0;
}
