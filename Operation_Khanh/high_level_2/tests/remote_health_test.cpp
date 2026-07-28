#include <array>
#include <cassert>
#include <chrono>

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

}  // namespace

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
    assert(!pad.GetCommand().is_active);
    pad.UpdateAt(neutral, At(200));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    pad.UpdateAt(neutral, At(201));
    assert(pad.link_state() == RemoteLinkState::kHealthy);
    assert(pad.GetCommand().is_active);

    // Dropout 2,999 s chưa bị coi là mất remote.
    pad.UpdateAt(zero, At(210));
    assert(pad.link_state() == RemoteLinkState::kSuspect);
    assert(pad.GetCommand().is_active);
    pad.UpdateAt(zero, At(3200));
    assert(pad.link_state() == RemoteLinkState::kSuspect);
    pad.UpdateAt(neutral, At(3201));
    assert(pad.link_state() == RemoteLinkState::kHealthy);

    // Mất trên 3 s: LOST; reconnect khi còn giữ nút không được nhận lệnh.
    pad.UpdateAt(zero, At(3210));
    pad.UpdateAt(zero, At(6202));
    assert(pad.link_state() == RemoteLinkState::kLost);
    assert(!pad.GetCommand().is_active);

    const LowState_ select_held = Packet(0x0008);
    pad.UpdateAt(select_held, At(6210));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    assert(!pad.GetCommand().is_active);
    pad.UpdateAt(neutral, At(6220));
    pad.UpdateAt(neutral, At(6419));
    assert(pad.link_state() == RemoteLinkState::kRecovering);
    pad.UpdateAt(neutral, At(6420));
    assert(pad.link_state() == RemoteLinkState::kHealthy);

    // Dropout ngắn không được phát lại edge của nút đang giữ.
    const LowState_ stand_combo = Packet(0x1020);  // L2 + Up
    pad.UpdateAt(neutral, At(6500));
    (void)pad.GetCommand();
    pad.UpdateAt(stand_combo, At(6510));
    assert(pad.GetCommand().want_stand_lock);
    pad.UpdateAt(zero, At(6520));
    assert(!pad.GetCommand().want_stand_lock);
    pad.UpdateAt(stand_combo, At(6530));
    assert(pad.link_state() == RemoteLinkState::kHealthy);
    assert(!pad.GetCommand().want_stand_lock);

    return 0;
}
