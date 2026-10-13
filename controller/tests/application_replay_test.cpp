// Golden replay of the whole Application state machine.
//
// Drives the real Application (real ONNX locomotion/mimic policies, gestures,
// get-up/lie-down trajectories) through a fixed gamepad script and hashes every
// LowCmd it emits. The plant is ideal: each motor that received kp > 0 reports
// the commanded q back as its measured position, dq = 0, torso upright.
//
// The golden file pins today's behaviour. A refactor must reproduce it bit for
// bit; an intentional behaviour change regenerates it with
//   HB_REPLAY_UPDATE=1 ./application_replay_test
// and the diff of tests/golden/application_replay.txt goes into review.
//
// Config comes from tests/fixtures/replay (a frozen copy), so editing the live
// config/ does not move the golden. Non-deterministic inputs are pinned:
// sample timestamps advance exactly 2 ms per tick, command slew is immediate,
// audio/teleop/telemetry/voice marker are disabled.
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "app/Application.hpp"

using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;

namespace {

std::vector<LowCmd> g_packets;

// R3-1 button bits (see GamepadR3.hpp xKeySwitchUnion).
enum Btn : uint16_t {
    kR1 = 0x0001, kStart = 0x0004, kR2 = 0x0010, kL2 = 0x0020,
    kA = 0x0100, kB = 0x0200, kX = 0x0400, kY = 0x0800,
    kUp = 0x1000, kRight = 0x2000, kDown = 0x4000, kLeft = 0x8000,
};

struct Fnv {
    uint64_t h = 1469598103934665603ull;
    void Bytes(const void* p, size_t n) {
        const auto* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ull; }
    }
    void F(float v) { Bytes(&v, sizeof v); }
    void I(int64_t v) { Bytes(&v, sizeof v); }
};

}  // namespace

struct LowCmdSenderTestAccess {
    static void Configure(LowCmdSender& sender, const Tuning& tuning) {
        sender.tuning_ = &tuning;
        sender.test_packet_sink_ = [](const LowCmd& p) { g_packets.push_back(p); };
    }
};

struct ApplicationTestAccess {
    Application& a;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::array<float, spec::kNumMotorsIdl> measured_q{};
    uint16_t buttons = 0;
    float lx = 0, rx = 0, ly = 0;
    Fnv window;
    std::ostringstream log;
    int last_state = -1;
    std::string last_gesture;
    long tick = 0;

    void Configure() {
        Tuning& t = a.tuning_;
        t.state_timeout_ms = 1e9f;          // fake sample clock may run ahead of wall time
        t.remote_state_timeout_ms = 1e9f;
        t.remote_recover_ms = 0;
        t.cmd_accel_vx = t.cmd_accel_vy = t.cmd_accel_yaw = 1e9f;   // slew uses wall dt
        t.cmd_decel_vx = t.cmd_decel_vy = t.cmd_decel_yaw = 1e9f;
        t.teleop_enabled = false;
        t.safe_stop_enabled = false;
        t.battery_monitor_enabled = false;
        t.mimic_telemetry_enabled = false;
        t.voice_enabled = false;
        t.gesture_voice_marker.clear();
        t.startup_voice_delay_s = 0.0f;

        a.InitControllers();
        a.estimator_.Configure(t, spec::kLoopDt);
        a.fall_detector_.Configure(t, spec::kLoopDt);
        a.input_.Configure(t);
        LowCmdSenderTestAccess::Configure(a.sender_, t);
        a.armed_ = true;
        a.state_ = AppState::kWaitingForState;
    }

    LowState Packet() const {
        LowState low;
        std::array<uint8_t, 40> remote{};
        remote[0] = 0x55; remote[1] = 0x51;
        remote[2] = buttons & 0xff; remote[3] = buttons >> 8;
        std::memcpy(&remote[4], &lx, 4);
        std::memcpy(&remote[8], &rx, 4);
        std::memcpy(&remote[20], &ly, 4);
        low.wireless_remote(remote);
        low.imu_state().quaternion({1, 0, 0, 0});
        low.imu_state().gyroscope({0, 0, 0});
        low.imu_state().accelerometer({0, 0, -9.81f});
        for (int m = 0; m < spec::kNumMotorsIdl; ++m) {
            low.motor_state()[m].q(measured_q[m]);
            low.motor_state()[m].dq(0.0f);
        }
        low.mode_machine(5);
        low.tick(static_cast<uint32_t>(tick));
        return low;
    }

    void Step() {
        {
            std::lock_guard<std::mutex> lock(a.state_mutex_);
            a.shared_low_state_ = Packet();
            a.got_state_ = true;
            ++a.state_sequence_;
            a.last_state_time_ = t0 + std::chrono::microseconds(2000 * tick);
        }
        const size_t before = g_packets.size();
        if (!a.Tick()) { std::cerr << "Tick() requested exit at " << tick << '\n'; std::abort(); }
        ++a.tick_;

        const int state = static_cast<int>(a.state_);
        window.I(state);
        for (size_t p = before; p < g_packets.size(); ++p) {
            const auto& cmd = g_packets[p].motor_cmd();
            for (int m = 0; m < spec::kNumMotorsIdl; ++m) {
                window.I(cmd[m].mode());
                window.F(cmd[m].q()); window.F(cmd[m].dq());
                window.F(cmd[m].kp()); window.F(cmd[m].kd()); window.F(cmd[m].tau());
            }
        }
        if (!g_packets.empty()) {
            const auto& cmd = g_packets.back().motor_cmd();
            for (int m = 0; m < spec::kNumMotorsIdl; ++m)
                if (cmd[m].kp() > 0.0f) measured_q[m] = cmd[m].q();
        }
        g_packets.clear();

        if (state != last_state) {
            log << "T " << tick << ' ' << a.StateName() << '\n';
            last_state = state;
        }
        if (a.arm_gesture_.ActiveName() != last_gesture) {
            last_gesture = a.arm_gesture_.ActiveName();
            log << "G " << tick << ' ' << (last_gesture.empty() ? "-" : last_gesture) << '\n';
        }
        ++tick;
        if (tick % 250 == 0) {
            log << "H " << tick << ' ' << std::hex << window.h << std::dec << '\n';
            window = Fnv{};
        }
    }

    void Wait(int ticks) { for (int i = 0; i < ticks; ++i) Step(); }
    // Advance until the state machine reaches `s`; the script must not stall.
    void WaitFor(AppState s, int max_ticks) {
        for (int i = 0; i < max_ticks; ++i) {
            if (a.state_ == s) return;
            Step();
        }
        std::cerr << "replay script stalled at tick " << tick << " in " << a.StateName()
                  << " waiting for state " << static_cast<int>(s) << '\n';
        std::abort();
    }
    // Edge press: hold for `hold` ticks, then release for `gap` ticks.
    void Press(uint16_t b, int hold = 5, int gap = 5) {
        buttons = b; Wait(hold); buttons = 0; Wait(gap);
    }
    void DoubleClick(uint16_t b) { Press(b, 5, 10); Press(b, 5, 10); }
    void Stick(float y, float turn, int ticks) {
        ly = y; rx = turn; Wait(ticks); ly = 0; rx = 0;
    }

    void Script() {
        WaitFor(AppState::kIdle, 10);  Wait(50);
        Press(kL2 | kUp);                           // stand up: warn, then ramp
        WaitFor(AppState::kStandLock, 5000); Wait(200);
        Press(kR2 | kA);                            // LOCOMOTION, policy blend-in
        WaitFor(AppState::kLocomotion, 10); Wait(600);
        Stick(0.8f, 0.0f, 1000);                    // walk forward (heading hold)
        Stick(0.0f, 0.5f, 500);                     // turn
        Wait(1000);                                 // gait settles, gesture stand dwell
        DoubleClick(kDown); Wait(2500);             // gesture slot 2
        Stick(0.5f, 0.0f, 300); Wait(1500);         // movement retracts the gesture
        Press(kR1 | kUp);                           // announce dance 2
        WaitFor(AppState::kReturningToDefault, 3000);
        WaitFor(AppState::kMimic, 5000); Wait(2500);
        Press(kL2 | kUp);                           // abort -> soft-stop -> LOCOMOTION
        WaitFor(AppState::kLocomotion, 5000); Wait(1100);   // dance_abort_lock_block_s
        Press(kL2 | kUp);                           // settle under policy -> STAND LOCK
        WaitFor(AppState::kStandUp, 3000);
        WaitFor(AppState::kStandLock, 5000); Wait(600);     // stand_lock_sit_block_s
        Press(kL2 | kLeft, 1600, 5);                // hold: sit with chair
        WaitFor(AppState::kSafeShutdown, 10); Wait(4000);
        Press(kL2 | kUp);                           // stand back up from the chair
        WaitFor(AppState::kStandLock, 5000); Wait(200);
        Press(kL2 | kX, 1600, 5);                   // hold: lie down
        WaitFor(AppState::kLieDown, 10);
        WaitFor(AppState::kIdle, 20000); Wait(100);
        Press(kL2 | kUp);                           // lying: prepare get-up (blend, hold)
        WaitFor(AppState::kGetUp, 10); Wait(1500);
        Press(kL2 | kX, 1600, 5);                   // hold: get up -> LOCOMOTION
        WaitFor(AppState::kLocomotion, 20000); Wait(500);
        Press(kL2 | kB);                            // E-stop
        WaitFor(AppState::kIdle, 10); Wait(100);
        Press(kL2 | kY);                            // ZERO TORQUE
        WaitFor(AppState::kZeroTorque, 10); Wait(300);
        Press(kL2 | kY);                            // back to IDLE
        WaitFor(AppState::kIdle, 10); Wait(100);
    }
};

int main() {
    setenv("HB_INTEGRATION_SOCKET", "/tmp/hb-application-replay-test-no-listener/socket", 1);
    Application app(HB_REPLAY_FIXTURE, "lo");
    ApplicationTestAccess r{app};
    r.Configure();
    r.Script();
    const std::string got = r.log.str();

    if (const char* update = std::getenv("HB_REPLAY_UPDATE"); update && *update == '1') {
        std::ofstream(HB_REPLAY_GOLDEN) << got;
        std::cout << "APPLICATION_REPLAY_GOLDEN_UPDATED " << HB_REPLAY_GOLDEN << '\n';
        return 0;
    }
    std::ifstream in(HB_REPLAY_GOLDEN);
    if (!in) { std::cerr << "missing golden " << HB_REPLAY_GOLDEN << '\n'; return 1; }
    std::stringstream want;
    want << in.rdbuf();
    if (got == want.str()) {
        std::cout << "APPLICATION_REPLAY_OK\n";
        return 0;
    }
    std::istringstream gs(got), ws(want.str());
    std::string gl, wl;
    for (int line = 1;; ++line) {
        const bool g_ok = static_cast<bool>(std::getline(gs, gl));
        const bool w_ok = static_cast<bool>(std::getline(ws, wl));
        if (!g_ok && !w_ok) break;
        if (gl != wl || g_ok != w_ok) {
            std::cerr << "replay diverges at golden line " << line << "\n  want: "
                      << (w_ok ? wl : "<eof>") << "\n  got:  " << (g_ok ? gl : "<eof>") << '\n';
            break;
        }
    }
    return 1;
}
