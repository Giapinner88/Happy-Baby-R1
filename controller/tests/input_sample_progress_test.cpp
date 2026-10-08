#include <array>
#include <cstdlib>
#include <iostream>
#include "input/GamepadR3.hpp"
#include "safety/SampleProgress.hpp"

void Require(bool ok) { if (!ok) std::abort(); }
auto At(int ms) { return GamepadR3::TimePoint{} + std::chrono::milliseconds(ms); }
auto Packet(uint16_t buttons) {
    unitree_hg::msg::dds_::LowState_ low;
    std::array<uint8_t,40> bytes{};
    bytes[0]=0x55; bytes[1]=0x51; bytes[2]=buttons&255; bytes[3]=buttons>>8;
    low.wireless_remote(bytes);
    return low;
}
int main() {
    Tuning tuning; tuning.remote_recover_ms=0; tuning.hold_to_trigger_s=0.5f;
    constexpr uint16_t kL2Left = 0x8020;  // held combo: sit
    GamepadR3 pad; SampleProgress progress;
    pad.ConfigureAt(tuning, At(0));
    auto feed = [&](int ms, uint16_t buttons, bool new_sample=true, bool fresh=true) {
        pad.UpdateAt(Packet(buttons), At(ms), fresh, new_sample,
                     progress.Update(new_sample, At(ms)));
        return pad.GetCommandAt(At(ms));
    };
    feed(0,0); feed(10,0);
    Require(pad.link_state()==RemoteLinkState::kHealthy);
    for (int ms=20; ms<=400; ms+=10) Require(!feed(ms,kL2Left).want_safe_shutdown);
    // A cached packet, still within remote expiry, never completes a hold.
    for (int ms=410; ms<=700; ms+=10) Require(!feed(ms,kL2Left,false).want_safe_shutdown);
    Require(pad.link_state()==RemoteLinkState::kHealthy);
    Require(!feed(710,kL2Left).want_safe_shutdown); // first packet after gap gives no time credit
    for (int ms=720; ms<830; ms+=10) Require(!feed(ms,kL2Left).want_safe_shutdown);
    Require(feed(830,kL2Left).want_safe_shutdown); // progress before gap was preserved
    Require(!feed(840,kL2Left).want_safe_shutdown);
    feed(850,0);
    // Preserve explicit simultaneous E-stop + Mode Z behavior.
    const auto both = feed(860,0x0a20); // L2+B+Y
    Require(both.want_emergency_stop && both.want_zero_torque);
    Require(!feed(870,0x0a20,false).want_zero_torque);
    feed(2000,0x0a20,false,false);
    Require(!pad.packet_valid());
    // Long scheduler pause even without intervening Update calls adds no hold time.
    Require(progress.Update(true,At(4000))==0.0);
    std::cout << "INPUT_SAMPLE_PROGRESS_OK\n";
}
