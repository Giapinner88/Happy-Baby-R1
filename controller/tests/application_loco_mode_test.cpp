// Walk <-> teleop locomotion modes (R2+A / R2+X) with two distinct bundles.
//
// Drives the real Application through an ideal plant (as application_replay_test)
// in a temporary project whose policies/locomotion holds two bundles: walk_a
// (the armfree flat_plus_gait_h4_v1 artifact) and teleop_b (the U1
// r1_wb_legs_v1_blind artifact, 12 leg actions, leg default differs by 0.048 rad),
// as in the live config. Checks:
//   * R2+A and R2+X only select the policy: START follows teleop_lock_base the
//     same way in both (false: accepted while moving, base keeps velocity;
//     true: needs zero sticks, then locks base);
//   * a bundle carrying a legacy `teleop_h4: false` line still gets H4;
//   * losing the remote lets a running dance finish (motion guard still
//     applies) but never starts one: an announced dance is cancelled and a
//     RETURNING entry goes back to LOCOMOTION;
//   * a pending STAND LOCK is cancelled by L2+Up again, R2+A or R2+X (no swap);
//   * L2+Up during RETURNING cancels the dance into LOCOMOTION (no blend) and a
//     held L2+Up is not taken as a STAND LOCK request.
//   * a mode request while walking stops the base and only swaps after the gait
//     is STAND-stable for policy_swap_dwell_s;
//   * a bundle whose policy.yaml hash differs refuses startup;
//   * IDLE always returns to walk mode.
//   * with a gait-FSM teleop bundle (U5, r1_wb_legs_gait_h4_v1_blind) standing
//     still but tilted 9 deg — its FSM never reaches STAND (entry <= 5 deg) — R2+A
//     still swaps back to walk (hardware 2026-10-29: swap timed out every time).
//   * R2+X supports physical/voice gestures, including a second request after
//     handover, and retracts a gesture before granting H4 ownership.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

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
    kR1 = 0x0001, kStart = 0x0004, kR2 = 0x0010, kL2 = 0x0020,
    kA = 0x0100, kB = 0x0200, kX = 0x0400, kUp = 0x1000,
};

const char* kWalkSha = "7daeea2a351abda42e2e361836637b85bb8122c7d1c5d09339b909ecad815ccb";
const char* kTeleopSha = "9cdf005ef7a7842b630744f298d992ad370ee2320f99d69691fc2db86b9e5fa5";
const char* kU5Sha = "c2ef27766e15f30539bda177787ba8ab05a7e70e2a2a32feac975cee39ed894e";

void WriteBundle(const fs::path& loco, const std::string& name, const std::string& source,
                 const std::string& contract, const std::string& sha,
                 const std::string& extra = "") {
    fs::create_directories(loco / name);
    fs::create_symlink(fs::path(HB_PROJECT_PATH) / "policies/locomotion" / source / "policy.onnx",
                       loco / name / "policy.onnx");
    std::ofstream(loco / name / "policy.yaml")
        << "contract: " << contract << "\nsha256: " << sha << "\n" << extra;
}

// Temporary project: frozen replay config + two bundles selected as walk/teleop.
fs::path MakeProject(const std::string& teleop_sha, const std::string& teleop_source = "u1_seed42",
                     const std::string& teleop_contract = "r1_wb_legs_v1_blind") {
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
    WriteBundle(root / "policies/locomotion", "teleop_b", teleop_source,
                teleop_contract, teleop_sha);
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
    bool remote_off = false;   // all-zero wireless_remote, as when the remote is lost
    float pitch_rad = 0;  // IMU pitch of the ideal plant (still, upright by default)
    long tick = 0;

    void Configure() {
        Tuning& t = a.tuning_;
        t.state_timeout_ms = 1e9f;
        t.remote_state_timeout_ms = 1e9f;
        t.remote_recover_ms = 0;
        t.cmd_accel_vx = t.cmd_accel_vy = t.cmd_accel_yaw = 1e9f;
        t.cmd_decel_vx = t.cmd_decel_vy = t.cmd_decel_yaw = 1e9f;
        t.teleop_enabled = true;      // receiver not started: no socket, weight stays 0
        t.teleop_lock_base = false;   // production default
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
        if (remote_off) remote.fill(0);
        low.wireless_remote(remote);
        low.imu_state().quaternion({std::cos(0.5f * pitch_rad), 0, std::sin(0.5f * pitch_rad), 0});
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

    // Same START/base rules whichever policy key is active.
    void CheckStartRules() {
        a.tuning_.teleop_lock_base = false;
        ly = 0.8f; Wait(100);
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        CHECK(std::fabs(a.cmd_vx_) > 1e-4f);
        ly = 0.0f; Wait(1000);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);

        a.tuning_.teleop_lock_base = true;
        ly = 0.8f; Wait(100);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);
        ly = 0.0f; Wait(1000);
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        ly = 0.8f; Wait(100);
        CHECK(a.cmd_vx_ == 0.0f);
        ly = 0.0f; Wait(100);
        Press(kStart);
        CHECK(!a.teleop_runtime_on_);
        a.tuning_.teleop_lock_base = false;
        Wait(1000);
    }

    void RemoteOff(bool off) { remote_off = off; buttons = 0; ly = 0.0f; }

    // No remote means no L2+B: a running dance finishes, a new one never starts.
    void CheckControlLostDance() {
        a.tuning_.safe_stop_enabled = true;
        // GamepadR3 declares LOST on the wall clock (remote_timeout_ms), so the
        // remote-off phase runs at the real 500 Hz pace, as on the robot.
        const int lost_ticks = static_cast<int>(
            (a.tuning_.remote_timeout_ms + a.tuning_.safe_stop_debounce_ms) /
            (1000.0f * spec::kLoopDt)) + 500;
        const auto paced_until = [&](auto done) {
            auto next = std::chrono::steady_clock::now();
            for (int i = 0; i < lost_ticks && !done(); ++i) {
                Step();
                next += std::chrono::microseconds(static_cast<int>(spec::kLoopDt * 1e6f));
                std::this_thread::sleep_until(next);
            }
        };

        // Running dance keeps running to its natural end, then LOCOMOTION.
        Press(kR1 | kUp);
        WaitFor(AppState::kMimic, 6000);
        Wait(500);
        CHECK(a.state_ == AppState::kMimic && !a.dance_stopping_);
        RemoteOff(true);
        paced_until([&] { return a.input_lost_; });
        CHECK(a.input_lost_);
        CHECK(a.state_ == AppState::kMimic && !a.dance_stopping_);
        for (int i = 0; i < 60000 && a.state_ == AppState::kMimic; ++i) {
            const bool was_stopping = a.dance_stopping_;
            Step();
            // Soft-stop may only start from the clip's natural end.
            if (!was_stopping && a.dance_stopping_)
                CHECK(a.mimics_.at(1)->RemainingFrames() <= 1.0f);
        }
        CHECK(a.state_ == AppState::kLocomotion);
        Wait(500);
        CHECK(a.state_ == AppState::kLocomotion && a.cmd_vx_ == 0.0f);
        RemoteOff(false);
        Wait(1000);

        // RETURNING held (unreachable tolerance) -> back to LOCOMOTION.
        const float tol = a.tuning_.return_pos_tol, timeout = a.tuning_.return_timeout_s;
        a.tuning_.return_pos_tol = -1.0f;   // ideal plant reaches default exactly
        a.tuning_.return_timeout_s = 60.0f;
        Press(kR1 | kUp);
        WaitFor(AppState::kReturningToDefault, 3000);
        RemoteOff(true);
        paced_until([&] { return a.state_ != AppState::kReturningToDefault; });
        CHECK(a.state_ == AppState::kLocomotion && a.active_ == a.locomotion_);
        CHECK(a.blend_alpha_ >= 1.0f);
        Wait(1000);
        CHECK(a.state_ == AppState::kLocomotion);
        RemoteOff(false);
        Wait(1000);
        a.tuning_.return_pos_tol = tol;
        a.tuning_.return_timeout_s = timeout;

        // Announcing dance (long announce) -> cancelled, never reaches RETURNING.
        a.tuning_.mimic_announce_delay_s = 10.0f;
        Press(kR1 | kUp);
        CHECK(a.announcing_);
        RemoteOff(true);
        paced_until([&] { return !a.announcing_; });
        CHECK(!a.announcing_);
        Wait(5000);
        CHECK(a.state_ == AppState::kLocomotion);
        RemoteOff(false);
        Wait(1000);
        a.tuning_.mimic_announce_delay_s = 2.0f;
        a.tuning_.safe_stop_enabled = false;
    }

    // Pending STAND LOCK can be taken back; L2+Up in RETURNING returns to walking.
    void CheckStandLockCancelAndReturning() {
        CHECK(a.state_ == AppState::kLocomotion && a.loco_mode_ == LocoMode::kWalk);
        // Without a cancel the request still completes (baseline).
        Press(kL2 | kUp);
        CHECK(a.pending_action_ == PendingAction::kToStandLock);
        WaitFor(AppState::kStandLock, 4000);
        Press(kR2 | kA);
        CHECK(a.state_ == AppState::kLocomotion);
        Wait(600);

        for (uint16_t cancel : {uint16_t(kL2 | kUp), uint16_t(kR2 | kA), uint16_t(kR2 | kX)}) {
            Press(kL2 | kUp);
            CHECK(a.pending_action_ == PendingAction::kToStandLock);
            Press(cancel);
            CHECK(a.pending_action_ == PendingAction::kNone);
            CHECK(!a.loco_swap_pending_ && a.loco_mode_ == LocoMode::kWalk);
            Wait(1500);
            CHECK(a.state_ == AppState::kLocomotion);
            CHECK(a.pending_action_ == PendingAction::kNone);
            ly = 0.8f; Wait(100);
            CHECK(std::fabs(a.cmd_vx_) > 1e-4f);      // sticks work again
            ly = 0.0f; Wait(600);
        }

        // L2+Up while RETURNING -> LOCOMOTION, held button ignored.
        const float tol = a.tuning_.return_pos_tol;
        a.tuning_.return_pos_tol = -1.0f;   // ideal plant reaches default exactly
        Press(kR1 | kUp);
        WaitFor(AppState::kReturningToDefault, 3000);
        Wait(50);
        buttons = kL2 | kUp; Wait(5);
        CHECK(a.state_ == AppState::kLocomotion && a.active_ == a.locomotion_);
        CHECK(a.blend_alpha_ >= 1.0f);
        Wait(300); buttons = 0; Wait(5);
        CHECK(a.pending_action_ == PendingAction::kNone);
        Wait(1500);
        CHECK(a.state_ == AppState::kLocomotion);
        a.tuning_.return_pos_tol = tol;
        Wait(600);
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
        CheckStartRules();

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

        // R2+X: identical START/base rules with the other policy.
        Wait(100);
        CheckStartRules();

        // Teleop -> walk (armed teleop is released first).
        Press(kStart);
        CHECK(a.teleop_runtime_on_);
        Press(kR2 | kA);
        CHECK(!a.teleop_runtime_on_);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
        CHECK(a.state_ == AppState::kLocomotion);

        Wait(500);
        CheckStandLockCancelAndReturning();
        CheckControlLostDance();

        // IDLE resets to walk mode even if teleop was selected.
        Wait(200);
        Press(kR2 | kX);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kTeleop);
        Press(kL2 | kB);
        CHECK(a.state_ == AppState::kIdle);
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
    }

    // R2+X on a gait-FSM leg policy standing still but tilted beyond its STAND entry
    // limit: the operator must still be able to return to the walk policy.
    void ScriptTiltedSwapBack() {
        const auto* gait = dynamic_cast<const WholeBodyLegsGaitController*>(a.teleop_ctrl_.get());
        CHECK(gait != nullptr);
        WaitFor(AppState::kIdle, 10); Wait(50);
        Press(kL2 | kUp);
        WaitFor(AppState::kStandLock, 5000); Wait(200);
        Press(kR2 | kA);
        CHECK(a.loco_mode_ == LocoMode::kWalk);
        Wait(600);
        Press(kR2 | kX);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kTeleop && a.locomotion_ == a.teleop_ctrl_.get());

        pitch_rad = 9.0f * 3.14159265f / 180.0f;
        Wait(1500);
        CHECK(a.state_ == AppState::kLocomotion);
        CHECK(gait->gait().mode() != GaitMode::Stand && gait->gait().stable());

        const long requested = tick;  // the request is taken on the first tick of Press
        Press(kR2 | kA);
        for (int i = 0; i < 4000 && a.loco_swap_pending_; ++i) Step();
        CHECK(a.loco_mode_ == LocoMode::kWalk && a.locomotion_ == a.walk_ctrl_.get());
        CHECK((tick - requested) * spec::kLoopDt >= a.tuning_.policy_swap_dwell_s);
        CHECK(a.state_ == AppState::kLocomotion);
        pitch_rad = 0.0f;
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

    const fs::path tilted = MakeProject(kU5Sha, "u5_seed42", "r1_wb_legs_gait_h4_v1_blind");
    {
        Application app(tilted.string(), "lo");
        ApplicationTestAccess r{app};
        r.Configure();
        r.ScriptTiltedSwapBack();
    }
    fs::remove_all(tilted);
    std::cout << "APPLICATION_LOCO_MODE_OK\n";
}
