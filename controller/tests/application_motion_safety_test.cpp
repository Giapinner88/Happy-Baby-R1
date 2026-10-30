// Full Application/LowCmdSender integration without DDS, sockets or motor output.
#include "app/Application.hpp"
#include <cnpy.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <unistd.h>

using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
static std::vector<LowCmd> packets;
static int checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) throw std::runtime_error("motion integration line " + std::to_string(__LINE__)); } while (0)
static Eigen::Quaternionf Rx(float deg) {
    return Eigen::Quaternionf(Eigen::AngleAxisf(deg / motion_safety::kRadToDeg, Eigen::Vector3f::UnitX()));
}
struct LowCmdSenderTestAccess {
    static void Setup(LowCmdSender& s, const Tuning& t) {
        s.tuning_ = &t;
        s.test_packet_sink_ = [](const LowCmd& p) { packets.push_back(p); };
    }
};
class StandingProbe : public PolicyController {
public:
    StandingProbe() { default_q_ = spec::kDefaultJointPos; kp_.fill(40); kd_.fill(2); }
    int ObsSize() const override { return 1; }
    std::string Name() const override { return "test-only"; }
protected:
    void BuildObservation(const ControlContext&, std::vector<float>&) override {}
};
class MimicProbe : public MimicController {
public:
    MimicProbe(const MotionData& m, float warmup) : MimicController(m, 0, 1, warmup) {
        kp_.fill(40); kd_.fill(2);
    }
    using MimicController::BuildObservation;
};

struct ApplicationTestAccess {
    static void ActiveConfig(Application& a) {
        CHECK(a.tuning_.state_motion_guard);
        CHECK(a.tuning_.dance_tracking_error_deg == 45);
        CHECK(a.tuning_.dance_tracking_debounce_s == .10f);
        CHECK(a.tuning_.motion_safety_state_timeout_s == .10f);
    }
    static void Setup(Application& a) {
        a.tuning_.state_motion_guard = true;
        a.tuning_.remote_recover_ms = 0;
        a.tuning_.teleop_enabled = a.tuning_.gesture_enabled = false;
        a.tuning_.battery_monitor_enabled = a.tuning_.safe_stop_enabled = false;
        a.tuning_.imu_pitch_trim_deg = a.tuning_.locomotion_trim_deg = 0;
        a.tuning_.state_timeout_ms = 1000; // the independent 100 ms guard must win
        a.tuning_.arm_hold_s = .5f;
        a.motion_guard_ = motion_safety::Guard(a.tuning_.MotionSafetyConfig());
        a.motion_guard_monitoring_ = false;
        a.motion_recovery_sample_seen_ = a.motion_recovery_released_ = false;
        a.estimator_.Configure(a.tuning_, spec::kLoopDt);
        a.estimator_.Reset();
        a.input_.Configure(a.tuning_);
        a.walk_ctrl_ = std::make_unique<StandingProbe>();
        a.locomotion_ = a.walk_ctrl_.get();
        a.active_ = a.locomotion_;
        a.state_ = AppState::kLocomotion;
        a.armed_ = true;
        a.tick_ = 1; // no inference decimation; target path is still exercised
        a.ai_target_q_ = spec::kDefaultJointPos;
        a.blend_alpha_ = 1;
        a.mimics_.clear();
        LowCmdSenderTestAccess::Setup(a.sender_, a.tuning_);
        packets.clear();
    }
    static LowState Packet(uint32_t tick, Eigen::Quaternionf root = Eigen::Quaternionf::Identity(), uint16_t buttons = 0) {
        LowState s;
        s.tick(tick);
        s.imu_state().quaternion({root.w(), root.x(), root.y(), root.z()});
        s.imu_state().accelerometer({0, 0, -9.81f});
        std::array<uint8_t, 40> bytes{};
        bytes[0] = 0x55; bytes[1] = 0x51; bytes[2] = buttons & 255; bytes[3] = buttons >> 8;
        s.wireless_remote(bytes);
        for (int i = 0; i < spec::kNumJoints; ++i) s.motor_state()[spec::MotorIdl(i)].q(spec::kDefaultJointPos[i]);
        return s;
    }
    static void Tick(Application& a, const LowState& s) { a.OnLowState(&s); CHECK(a.Tick()); }
    static void ZeroPackets(size_t start = 0) {
        CHECK(packets.size() > start);
        for (size_t n = start; n < packets.size(); ++n) for (const auto& m : packets[n].motor_cmd()) {
            CHECK(m.mode() == 0 && m.q() == 0 && m.dq() == 0 && m.kp() == 0 && m.kd() == 0 && m.tau() == 0);
        }
    }
    static void Latched(Application& a, const char* reason) {
        CHECK(a.motion_guard_.Latched());
        CHECK(std::string(a.motion_guard_.Reason()) == reason);
        CHECK(a.state_ == AppState::kIdle && a.active_ == nullptr);
        CHECK(a.armed_); // exactly the normal L2+B state: still owns lowcmd
        ZeroPackets(packets.size() - 1);
    }
    static MimicProbe* Dance(Application& a, MotionData& m, int stage) {
        a.pending_dance_key_ = 7;
        a.tuning_.dance_tilt_guard[6] = false; // legacy off must not bypass new guard
        auto p = std::make_unique<MimicProbe>(m, stage == 0 ? .2f : 0);
        auto* ptr = p.get();
        a.mimics_[7] = std::move(p);
        a.active_ = ptr;
        a.state_ = AppState::kMimic;
        RobotState rs{};
        rs.q = spec::kDefaultJointPos;
        ptr->Reset(rs);
        ControlContext ctx{rs, 0, 0, 0, {0, 1}};
        std::vector<float> obs(ptr->ObsSize());
        if (stage != 0) ptr->BuildObservation(ctx, obs);
        if (stage == 2) {
            ptr->BeginCooldown(spec::kDefaultJointPos, .2f, rs);
            ptr->BuildObservation(ctx, obs);
        }
        CHECK(ptr->TelemetryStage() == stage);
        return ptr;
    }
    static void UprightExit(Application& a, MotionData& upright, MotionData& inverted) {
        for (int stage : {1, 2}) {
            Setup(a); auto* m = Dance(a, upright, stage);
            CHECK(m->SafetyUprightExpected());
            for (int k=0; k<=15; ++k) {
                auto low=Packet(100+2*k, Rx(35));
                low.imu_state().gyroscope({5,0,0}); Tick(a, low);
                CHECK(a.motion_guard_.Latched()==(k==15));
            }
            Latched(a, "dance_upright_fast_flip");
            size_t start=packets.size();
            for(int k=0;k<20;++k){auto low=Packet(132+2*k,Rx(35),0x0110);
                low.imu_state().gyroscope({5,0,0});Tick(a,low);}
            ZeroPackets(start); // no automatic resume, still armed IDLE
        }
        Setup(a); auto* warm= Dance(a,upright,0);
        CHECK(!warm->SafetyUprightExpected());
        Setup(a); auto* inverted_cool= Dance(a,inverted,2);
        CHECK(!inverted_cool->SafetyUprightExpected());
        ControlContext ctx{a.estimator_.state(),0,0,0,{0,1}};
        std::vector<float> obs(inverted_cool->ObsSize());
        for(int k=0;k<12;++k)inverted_cool->BuildObservation(ctx,obs);
        CHECK(inverted_cool->CooldownDone());
        CHECK(inverted_cool->SafetyUprightExpected());
    }
    static void Run(Application& a, MotionData& motion) {
        // Guard stop and the ordinary L2+B path must produce the same motor
        // packet and the same still-armed IDLE state, without changing gains.
        Setup(a); Tick(a, Packet(100));
        Tick(a, Packet(102, Eigen::Quaternionf::Identity(), 0x0220));
        CHECK(a.state_ == AppState::kIdle && a.armed_);
        const LowCmd emergency = packets.back();
        Setup(a); a.StopForMotionFault("comparison_stop");
        CHECK(a.state_ == AppState::kIdle && a.armed_);
        const LowCmd stopped_packet = packets.back();
        CHECK(stopped_packet.mode_machine() == emergency.mode_machine());
        for (size_t i=0; i<emergency.motor_cmd().size(); ++i) {
            const auto& x = stopped_packet.motor_cmd()[i];
            const auto& y = emergency.motor_cmd()[i];
            CHECK(x.mode()==y.mode() && x.q()==y.q() && x.dq()==y.dq() &&
                  x.kp()==y.kp() && x.kd()==y.kd() && x.tau()==y.tau());
        }
        // Upright guards use fresh sensor time, not repetitions of a cached packet.
        Setup(a);
        for (int i = 0; i < 15; ++i) { Tick(a, Packet(100 + 2*i, Rx(70))); CHECK(!a.motion_guard_.Latched()); }
        Tick(a, Packet(130, Rx(70))); Latched(a, "upright_tilt");
        size_t stopped = packets.size();
        for (int i = 0; i < 20; ++i) Tick(a, Packet(132 + 2*i, Eigen::Quaternionf::Identity(), i%2 ? 0x0110 : 0x1001));
        ZeroPackets(stopped); // upright recovery and loco/dance requests cannot resume

        for (int stage : {0, 1, 2}) {
            Setup(a); auto* mimic = Dance(a, motion, stage);
            const auto reference = mimic->TelemetryReferenceTorsoWorld();
            // A commanded inversion during clip/cooldown is legal, unlike locomotion.
            for (int i = 0; i < 70; ++i) Tick(a, Packet(100 + 2*i, reference));
            CHECK(!a.motion_guard_.Latched());
            size_t begin = packets.size();
            // An off legacy flag cannot allow a persistent 60-degree deviation.
            for (int i = 0; i < 50; ++i) {
                Tick(a, Packet(240 + 2*i, Rx(60) * reference));
                CHECK(!a.motion_guard_.Latched()); // first bad sample receives no preceding credit
            }
            Tick(a, Packet(340, Rx(60) * reference)); Latched(a, "dance_tracking");
            CHECK(packets.size() > begin);
        }
        // Global yaw is ignored, and waist rotation must be included when
        // converting a pelvis IMU to the torso tracked by the reference.
        Setup(a); auto* mimic = Dance(a, motion, 1);
        auto ref = mimic->TelemetryReferenceTorsoWorld();
        Eigen::Quaternionf yaw(Eigen::AngleAxisf(1.5f, Eigen::Vector3f::UnitZ()));
        for (int i = 0; i < 70; ++i) Tick(a, Packet(100 + 2*i, yaw * ref));
        CHECK(!a.motion_guard_.Latched());
        for (int i = 0; i < 70; ++i) {
            auto s = Packet(240 + 2*i, ref * Rx(-25));
            s.motor_state()[spec::MotorIdl(spec::kWaistRollPolicyIdx)].q(25 / motion_safety::kRadToDeg);
            Tick(a, s);
        }
        CHECK(!a.motion_guard_.Latched());
        // A short tilt pulse must reset rather than accumulate across good samples.
        for (int i = 0; i < 20; ++i) Tick(a, Packet(380 + 2*i, Rx(60) * ref));
        for (int i = 0; i < 70; ++i) Tick(a, Packet(420 + 2*i, ref));
        CHECK(!a.motion_guard_.Latched());

        for (int fault = 0; fault < 6; ++fault) {
            Setup(a); Dance(a, motion, 1);
            auto s = Packet(100);
            const float nan = std::numeric_limits<float>::quiet_NaN();
            if (fault == 0) s.motor_state()[0].q(nan);
            if (fault == 1) s.motor_state()[spec::kHeadPitchIdl].dq(nan);
            if (fault == 2) s.imu_state().gyroscope({nan, 0, 0});
            if (fault == 3) s.imu_state().quaternion({0, 0, 0, 0});
            if (fault == 4) s.imu_state().quaternion({3, 0, 0, 0});
            if (fault == 5) s.imu_state().accelerometer({0, nan, 0});
            Tick(a, s); Latched(a, "invalid_state"); ZeroPackets();
        }
        Setup(a);
        for (int i = 0; i <= 15; ++i) {
            auto s = Packet(100 + 2*i); s.motor_state()[spec::kHeadYawIdl].dq(30); Tick(a, s);
        }
        Latched(a, "joint_speed"); // head speed is independent of orientation
        Setup(a); Tick(a, Packet(100)); Tick(a, Packet(98)); Latched(a, "state_clock_reversed");
        Setup(a);
        const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        a.motion_guard_.Sensor(100, now-.2, Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), true, 0);
        a.motion_guard_monitoring_ = true;
        auto stale = Packet(100); a.OnLowState(&stale);
        a.last_state_progress_time_ = std::chrono::steady_clock::now()-std::chrono::milliseconds(200);
        Tick(a, Packet(100)); Latched(a, "stale_state");
        Tick(a, Packet(102)); Latched(a, "stale_state");

        // The control consumer paused, while the callback received fresh progression.
        Setup(a);
        a.motion_guard_.Sensor(100, now-.2, Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero(), true, 0);
        a.motion_guard_monitoring_ = true;
        Tick(a, Packet(300));
        CHECK(!a.motion_guard_.Latched()); CHECK(a.state_ == AppState::kLocomotion);

        Setup(a); a.state_ = AppState::kMimic; a.pending_dance_key_ = 7;
        Tick(a, Packet(100)); Latched(a, "invalid_reference"); ZeroPackets();
        Setup(a); a.ai_target_q_[0] = std::numeric_limits<float>::quiet_NaN();
        Tick(a, Packet(100)); Latched(a, "invalid_command"); ZeroPackets();
        Setup(a); a.tick_ = spec::kPolicyDecimation; // deliberately uninitialized ONNX
        Tick(a, Packet(100)); Latched(a, "inference_failure"); ZeroPackets();

        // Returning upright, feedback reconnecting, and holding an old request
        // cannot restart anything. No DISARM/restart is needed for a NEW command.
        a.tick_ = 1;
        stopped = packets.size();
        for (int i = 0; i < 20; ++i) Tick(a, Packet(200+2*i, Eigen::Quaternionf::Identity(), 0x1020));
        Latched(a, "inference_failure");
        ZeroPackets(stopped);
        for (int i = 0; i < 20; ++i) Tick(a, Packet(240+2*i));
        Latched(a, "inference_failure"); // neutral by itself does not run the old policy
        stopped = packets.size();
        auto moving = Packet(280, Eigen::Quaternionf::Identity(), 0x1020);
        moving.motor_state()[0].dq(1); // exceeds existing entry quiet limit
        Tick(a, moving); Latched(a, "inference_failure");
        for (int i = 0; i < 10; ++i) Tick(a, Packet(282+2*i, Eigen::Quaternionf::Identity(), 0x1020));
        Latched(a, "inference_failure"); // rejected requests are not queued
        ZeroPackets(stopped);
        Tick(a, Packet(302));
        // An advancing callback with a frozen physical tick is not recovery.
        Tick(a, Packet(302, Eigen::Quaternionf::Identity(), 0x1020));
        Latched(a, "inference_failure");
        Tick(a, Packet(304));
        auto bad = Packet(306, Eigen::Quaternionf::Identity(), 0x1020);
        bad.motor_state()[spec::kHeadPitchIdl].q(std::numeric_limits<float>::quiet_NaN());
        Tick(a, bad); Latched(a, "inference_failure");
        Tick(a, Packet(308)); Tick(a, Packet(310));
        Tick(a, Packet(312, Rx(85), 0x1020));
        Latched(a, "inference_failure"); // cannot stand while lying without getup asset
        Tick(a, Packet(314));
        size_t resumed = packets.size();
        Tick(a, Packet(316, Eigen::Quaternionf::Identity(), 0x1020));
        CHECK(!a.motion_guard_.Latched() && a.armed_ && a.state_ == AppState::kStandUp);
        CHECK(packets.size() > resumed);
        CHECK(packets.back().motor_cmd()[0].kp() > 0);
        CHECK(!a.announcing_ && !a.dance_stopping_); // never resumes the interrupted dance
        // The same guard remains effective after manual recovery.
        for (int i = 0; i <= 15; ++i) Tick(a, Packet(318+2*i, Rx(70)));
        Latched(a, "upright_tilt");
        // Optional DISARM retains the OLD Mode-Z-only procedure; it is not
        // required to recover from damping and has no guard-specific shortcut.
        a.tuning_.arm_gate_enabled = true;
        a.state_fresh_ = a.remote_state_fresh_ = a.new_state_sample_ = true;
        Tick(a, Packet(350)); Tick(a, Packet(352));
        CHECK(a.armed_ && a.state_ == AppState::kIdle);
        Tick(a, Packet(354, Eigen::Quaternionf::Identity(), 0x0820)); // L2+Y
        CHECK(!a.motion_guard_.Latched() && a.armed_ && a.state_ == AppState::kZeroTorque);

        Setup(a);
        a.getup_motion_ = std::make_unique<JointTrajectory>();
        a.getup_motion_->Load(std::string(HB_PROJECT_PATH) + "/motions/getup.npz");
        a.StopForMotionFault("dance_tracking");
        Tick(a, Packet(100, Rx(85))); Tick(a, Packet(102, Rx(85)));
        Latched(a, "dance_tracking");
        Tick(a, Packet(104, Rx(85), 0x1020));
        CHECK(!a.motion_guard_.Latched() && a.armed_ && a.state_ == AppState::kGetUp);
        CHECK(a.traj_phase_ == 0 && !a.announcing_); // existing prepare/hold/play sequence
        for (int i=0; i<30; ++i) Tick(a, Packet(106+2*i, Rx(85)));
        CHECK(!a.motion_guard_.Latched() && a.state_ == AppState::kGetUp);
    }
};

int main() {
    setenv("HB_INTEGRATION_SOCKET", "/tmp/hb-motion-safety-no-listener/socket", 1);
    const auto dir = std::filesystem::temp_directory_path() / ("hb_motion_guard_" + std::to_string(getpid()));
    std::filesystem::create_directory(dir);
    const std::string npz = (dir / "motion.npz").string();
    std::vector<float> q(32*24), dq(q.size(), 0), quat(32*15*4, 0);
    for (int f=0; f<32; ++f) {
        for (int j=0; j<24; ++j) q[f*24+j] = spec::kDefaultJointPos[j];
        for (int b=0; b<15; ++b) quat[(f*15+b)*4] = 1;
        const auto r = Rx(180);
        quat[(f*15+14)*4]=r.w(); quat[(f*15+14)*4+1]=r.x();
    }
    float fps=50;
    cnpy::npz_save(npz, "fps", &fps, {1}, "w");
    cnpy::npz_save(npz, "joint_pos", q.data(), {32,24}, "a");
    cnpy::npz_save(npz, "joint_vel", dq.data(), {32,24}, "a");
    cnpy::npz_save(npz, "body_quat_w", quat.data(), {32,15,4}, "a");
    MotionData motion; motion.Load(npz);
    Application app(HB_PROJECT_PATH, "lo");
    ApplicationTestAccess::ActiveConfig(app);
    ApplicationTestAccess::Run(app, motion);
    for(int f=0;f<32;++f){quat[(f*15+14)*4]=1;quat[(f*15+14)*4+1]=0;}
    const std::string upright_npz=(dir/"upright.npz").string();
    cnpy::npz_save(upright_npz,"fps",&fps,{1},"w");
    cnpy::npz_save(upright_npz,"joint_pos",q.data(),{32,24},"a");
    cnpy::npz_save(upright_npz,"joint_vel",dq.data(),{32,24},"a");
    cnpy::npz_save(upright_npz,"body_quat_w",quat.data(),{32,15,4},"a");
    MotionData upright;upright.Load(upright_npz);
    ApplicationTestAccess::UprightExit(app,upright,motion);
    for (const char* entry : {"state_motion_guard: typo\n", "dance_tracking_error_deg: nan\n",
            "dance_tracking_error_deg: 0\n", "dance_tracking_debounce_s: 0\n",
            "motion_safety_state_timeout_s: 1\n", "dance_tracking_debounce_s: 0.1sec\n",
            "dance_tracking_eror_deg: 45\n", "state_motion_guard: true\nfall_enabled: false\n",
            "state_motion_guard: true\njoint_speed_guard_enabled: false\n"}) {
        const auto p=dir/"bad.yaml"; std::ofstream(p) << entry;
        Tuning t; CHECK(!t.LoadFromFile(p.string()));
    }
    std::filesystem::remove_all(dir);
    std::cout << "APPLICATION_MOTION_SAFETY_OK checks=" << checks << '\n';
}
