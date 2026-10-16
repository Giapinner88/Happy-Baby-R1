// Walk <-> teleop locomotion modes (R2+A / R2+X) with two distinct bundles.
//
// Drives the real Application through an ideal plant (as application_replay_test)
// in a temporary project whose policies/locomotion holds two bundles: walk_a
// (the armfree flat_plus_gait_h4_v1 artifact) and teleop_b (the U1
// r1_wb_legs_v1_blind artifact, 12 leg actions, leg default differs by 0.048 rad),
// as in the live config. Checks:
//   * R2+A accepts START only while stopped and locks base; R2+X accepts it
//     while moving and preserves base velocity;
//   * a mode request while walking stops the base and only swaps after the gait
//     is STAND-stable for policy_swap_dwell_s;
//   * a bundle whose policy.yaml hash differs refuses startup;
//   * IDLE always returns to walk mode.
//   * R2+X supports physical/voice gestures, including a second request after
//     handover, and retracts a gesture before granting H4 ownership.
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "app/Application.hpp"

namespace fs = std::filesystem;
using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;

namespace {

void Require(bool ok, int line) {
    if (!ok) { std::cerr << "application_loco_mode_test failed at line " << line << '\n'; std::abort(); }
}
#define CHECK(x) Require((x), __LINE__)

enum Btn : uint16_t {
    kStart = 0x0004, kR2 = 0x0010, kL2 = 0x0020,
    kA = 0x0100, kB = 0x0200, kX = 0x0400, kUp = 0x1000,
};

const char* kWalkSha = "7daeea2a351abda42e2e361836637b85bb8122c7d1c5d09339b909ecad815ccb";
const char* kTeleopSha = "9cdf005ef7a7842b630744f298d992ad370ee2320f99d69691fc2db86b9e5fa5";

void WriteBundle(const fs::path& loco, const std::string& name, const std::string& source,
                 const std::string& contract, const std::string& sha) {
    fs::create_directories(loco / name);
    fs::create_symlink(fs::path(HB_PROJECT_PATH) / "policies/locomotion" / source / "policy.onnx",
                       loco / name / "policy.onnx");
    std::ofstream(loco / name / "policy.yaml")
        << "contract: " << contract << "\nsha256: " << sha << "\nteleop_h4: true\n";
}

// Temporary project: frozen replay config + two bundles selected as walk/teleop.
fs::path MakeProject(const std::string& teleop_sha) {
    char tmpl[] = "/tmp/hb-loco-mode-test.XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) std::abort();
    const fs::path root(dir);
    const fs::path src(HB_PROJECT_PATH);
    fs::copy(src / "tests/fixtures/replay/config", root / "config");
    fs::create_symlink(src / "motions", root / "motions");
    fs::create_directories(root / "policies/locomotion");
    for (const char* sub : {"dance", "gestures", "flat"})
        fs::create_symlink(src / "policies" / sub, root / "policies" / sub);
    WriteBundle(root / "policies/locomotion", "walk_a", "flat_plus_gait_0917_armfree",
                "flat_plus_gait_h4_v1", kWalkSha);
    WriteBundle(root / "policies/locomotion", "teleop_b", "u1_seed42",
                "r1_wb_legs_v1_blind", teleop_sha);
    std::ofstream(root / "config/locomotion.yaml", std::ios::app)
        << "\nwalk_policy: walk_a\nteleop_policy: teleop_b\n"
           "policy_swap_dwell_s: 0.5\npolicy_swap_timeout_s: 6.0\n";
    return root;
}

std::vector<LowCmd> g_packets;

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
    float ly = 0;
    long tick = 0;

    void Configure() {
        Tuning& t = a.tuning_;
        t.state_timeout_ms = 1e9f;
        t.remote_state_timeout_ms = 1e9f;
        t.remote_recover_ms = 0;
        t.cmd_accel_vx = t.cmd_accel_vy = t.cmd_accel_yaw = 1e9f;
        t.cmd_decel_vx = t.cmd_decel_vy = t.cmd_decel_yaw = 1e9f;
        t.teleop_enabled = true;      // receiver not started: no socket, weight stays 0
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
        CHECK(a.Tick());
        ++a.tick_;
        if (!g_packets.empty()) {
            const auto& cmd = g_packets.back().motor_cmd();
            for (int m = 0; m < spec::kNumMotorsIdl; ++m)
                if (cmd[m].kp() > 0.0f) measured_q[m] = cmd[m].q();
        }
        g_packets.clear();
        ++tick;
    }
    void Wait(int n) { for (int i = 0; i < n; ++i) Step(); }
    void WaitFor(AppState s, int max_ticks) {
        for (int i = 0; i < max_ticks && a.state_ != s; ++i) Step();
        CHECK(a.state_ == s);
    }
    void Press(uint16_t b) { buttons = b; Wait(5); buttons = 0; Wait(5); }
    void DoubleClick(uint16_t b) { Press(b); Press(b); }

    float ArmOffset() const {
        float gap = 0.0f;
        for (int j = 14; j < 24; ++j)
            gap = std::max(gap, std::fabs(
                measured_q[spec::MotorIdl(j)] - a.locomotion_->default_q()[j]));
        return gap;
    }

    void WaitForGestureIdle() {
        for (int i = 0; i < 15000 && !a.arm_gesture_.Idle(); ++i) Step();
        CHECK(a.arm_gesture_.Idle());
        CHECK(a.state_ == AppState::kLocomotion);
        CHECK(ArmOffset() < a.tuning_.gesture_handover_measured_position_tol_rad);
        Wait(5);  // allow the voice owner to clear a completed command
    }

    std::string VoiceRequest(const std::string& id, const std::string& gesture) {
        const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        CHECK(fd >= 0);
        timeval timeout{2, 0};
        CHECK(::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        const std::string path = a.proj_dir_ + "/gesture.sock";
        std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
        CHECK(::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
        const std::string request = "{\"request_id\":\"" + id +
            "\",\"command\":\"play\",\"gesture_id\":\"" + gesture + "\"}\n";
        CHECK(::write(fd, request.data(), request.size()) == static_cast<ssize_t>(request.size()));
        Wait(5);
        char reply[2048]{};
        const ssize_t size = ::read(fd, reply, sizeof(reply));
        CHECK(size > 0);
        ::close(fd);
        return std::string(reply, static_cast<size_t>(size));
    }

    void CheckR2XGestures() {
        CHECK(a.PolicyCaps().gesture && !a.PolicyCaps().legacy_arm);
        CHECK(!a.PolicyCaps().history_gesture);
        CHECK(a.voice_gesture_owner_.Init(a.proj_dir_ + "/gesture.sock"));
        Wait(600);

        // Real gamepad double-click and emitted LowCmd arm targets.
        DoubleClick(kUp); Wait(200);
        CHECK(a.arm_gesture_.ActiveName() == a.gesture_slot_[1]);
        CHECK(ArmOffset() > 0.02f);
        DoubleClick(kUp);
        WaitForGestureIdle();

        // Voice play goes through the real IPC owner, completes and accepts a
        // second gesture. Movement still rejects the voice request.
        for (const std::string& gesture : {std::string("wave"), std::string("heart")}) {
            const std::string reply = VoiceRequest(gesture, gesture);
            CHECK(reply.find("\"status\":\"accepted\"") != std::string::npos);
            Wait(200);
            CHECK(a.voice_command_gesture_active_);
            CHECK(ArmOffset() > 0.02f);
            WaitForGestureIdle();
            CHECK(!a.voice_command_gesture_active_);
        }
        ly = 0.8f; Wait(100);
        CHECK(VoiceRequest("moving", "wave").find("movement_requested") != std::string::npos);
        ly = 0.0f; Wait(600);

        // Double-click A enables voice-auto; the marker starts/stops it.
        a.tuning_.gesture_voice_marker = a.proj_dir_ + "/voice_speaking";
        a.tuning_.gesture_voice_stale_s = 3600.0f;
        DoubleClick(kA);
        CHECK(a.voice_gesture_auto_enabled_);
        std::ofstream(a.tuning_.gesture_voice_marker) << "speaking\n";
        Wait(300);
        CHECK(a.arm_gesture_.ActiveName() == a.tuning_.gesture_voice_name);
        CHECK(ArmOffset() > 0.02f);
        fs::remove(a.tuning_.gesture_voice_marker);
        Wait(100);
        WaitForGestureIdle();
        DoubleClick(kA);
        CHECK(!a.voice_gesture_auto_enabled_);

        // H4 must wait for bounded return/handover instead of cancelling the
        // active gesture. Packets stay loopback and LowCmd stays in the sink.
        DoubleClick(kUp); Wait(200);
        CHECK(ArmOffset() > 0.02f);
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        const int tx = ::socket(AF_INET, SOCK_DGRAM, 0);
        CHECK(tx >= 0);
        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        CHECK(::bind(tx, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) == 0);
        socklen_t size = sizeof(dst);
        CHECK(::getsockname(tx, reinterpret_cast<sockaddr*>(&dst), &size) == 0);
        const int port = ntohs(dst.sin_port);
        ::close(tx);
        std::array<float, 10> def_arm{};
        for (int j = 0; j < 10; ++j) def_arm[j] = a.locomotion_->default_q()[14 + j];
        a.teleop_.Init(def_arm, true, port, 1e9f, 0.4f, 0.5f, 8.0f, 1.0f, 0.6f);
        const int out = ::socket(AF_INET, SOCK_DGRAM, 0);
        CHECK(out >= 0);
        TeleopReceiver::Packet packet{};
        packet.magic = TeleopReceiver::kMagic;
        packet.pad = 1;
        packet.arm_valid = 1;
        std::copy(def_arm.begin(), def_arm.end(), packet.arm_q);
        for (int seq = 1; seq <= 2; ++seq) {
            packet.seq = seq;
            packet.enable = seq == 2;
            CHECK(::sendto(out, &packet, sizeof(packet), 0,
                  reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) == sizeof(packet));
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        CHECK(a.teleop_.H4Authorized());
        Step();
        CHECK(!a.arm_gesture_.Idle());
        CHECK(!a.teleop_.Active());
        CHECK(ArmOffset() > 0.02f);
        WaitForGestureIdle();
        CHECK(a.teleop_.Active());
        CHECK(VoiceRequest("h4_busy", "wave").find("teleop_owns_upper_body") != std::string::npos);
        DoubleClick(kUp);
        CHECK(a.arm_gesture_.Idle());
        Press(kStart);
        a.teleop_.Stop();
        ::close(out);
        Wait(1000);
        a.voice_gesture_owner_.Close();
    }

    void Script() {
        CHECK(a.walk_ctrl_ && a.teleop_ctrl_ && a.walk_ctrl_ != a.teleop_ctrl_);
        CHECK(a.teleop_ctrl_->ActionSize() == 12 && a.walk_ctrl_->ActionSize() == 24);
        CHECK(a.h4_teleop_approved_);

        WaitFor(AppState::kIdle, 10); Wait(50);
        Press(kL2 | kUp);
        WaitFor(AppState::kStandLock, 5000); Wait(200);

        // R2+A: walk mode, walk controller; H4 START also works after the
        // operator velocity command has settled to zero.
        Press(kR2 | kA);
        CHECK(a.state_ == AppState::kLocomotion);
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
        CHECK(a.active_ == a.walk_ctrl_.get());
        CHECK(a.PolicyCaps().h4);
        Wait(600);
        ly = 0.8f; Wait(100);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);
        ly = 0.0f; Wait(1000);
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);

        // Request teleop while walking: base stops, swap waits for a stable STAND.
        ly = 0.8f; Wait(800);
        buttons = kR2 | kX; Wait(5);
        CHECK(a.loco_swap_pending_ && a.loco_mode_ == LocoMode::kWalk);
        Wait(5); buttons = 0;
        CHECK(a.cmd_vx_ == 0.0f);                       // holding while pending
        const long requested = tick;
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) {
            Step();
            if (a.loco_swap_pending_) CHECK(a.locomotion_ == a.walk_ctrl_.get());
        }
        ly = 0.0f;
        CHECK(!a.loco_swap_pending_);
        CHECK(a.loco_mode_ == LocoMode::kTeleop);
        CHECK(a.locomotion_ == a.teleop_ctrl_.get() && a.active_ == a.teleop_ctrl_.get());
        // Never sooner than the configured dwell after the request.
        CHECK((tick - requested) * spec::kLoopDt >= a.tuning_.policy_swap_dwell_s);
        CHECK(a.state_ == AppState::kLocomotion);
        CHECK(a.PolicyCaps().h4);

        CheckR2XGestures();

        // R2+X uses teleop_policy: START is accepted while moving and base
        // velocity remains available during the H4 session.
        Wait(100);
        ly = 0.8f; Wait(100);
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        CHECK(std::fabs(a.cmd_vx_) > 1e-4f);
        ly = 0.0f; Wait(1000);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);

        // Teleop -> walk (armed teleop is released first).
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        Press(kR2 | kA);
        CHECK(!a.teleop_runtime_on_);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
        CHECK(a.state_ == AppState::kLocomotion);

        // IDLE resets to walk mode even if teleop was selected.
        Wait(200);
        Press(kR2 | kX);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kTeleop);
        Press(kL2 | kB);
        CHECK(a.state_ == AppState::kIdle);
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
    }
};

int main() {
    setenv("HB_INTEGRATION_SOCKET", "/tmp/hb-loco-mode-test-no-listener/socket", 1);

    // A teleop bundle whose recorded hash differs must stop startup.
    {
        const fs::path bad = MakeProject(std::string(64, '0'));
        Application app(bad.string(), "lo");
        ApplicationTestAccess r{app};
        bool threw = false;
        try { r.Configure(); } catch (const std::exception& e) {
            threw = std::string(e.what()).find("teleop_b") != std::string::npos;
        }
        fs::remove_all(bad);
        CHECK(threw);
    }

    const fs::path root = MakeProject(kTeleopSha);
    {
        Application app(root.string(), "lo");
        ApplicationTestAccess r{app};
        r.Configure();
        r.Script();
    }
    fs::remove_all(root);
    std::cout << "APPLICATION_LOCO_MODE_OK\n";
}
