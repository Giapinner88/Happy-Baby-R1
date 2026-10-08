#include <cstdlib>
#include <iostream>
#include <vector>
#include "app/Application.hpp"

using LowState = unitree_hg::msg::dds_::LowState_;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
static std::vector<LowCmd> packets;
static void Require(bool ok, int line) {
    if (!ok) { std::cerr << "Application safety test failed at " << line << '\n'; std::abort(); }
}
#define CHECK(x) Require((x), __LINE__)

struct LowCmdSenderTestAccess {
    static void Configure(LowCmdSender& sender, const Tuning& tuning) {
        sender.tuning_ = &tuning;
        sender.test_packet_sink_ = [](const LowCmd& p) { packets.push_back(p); };
    }
};

struct ApplicationTestAccess {
    static LowState Packet(uint16_t btn = 0, float dq = 0) {
        LowState low;
        std::array<uint8_t,40> bytes{};
        bytes[0]=0x55; bytes[1]=0x51; bytes[2]=btn&255; bytes[3]=btn>>8;
        low.wireless_remote(bytes);
        low.imu_state().quaternion({1,0,0,0});
        low.imu_state().gyroscope({0,0,0});
        low.imu_state().accelerometer({0,0,-9.81f});
        low.motor_state()[0].dq(dq);
        return low;
    }
    static void Setup(Application& a) {
        a.tuning_.remote_recover_ms=0;
        a.tuning_.arm_hold_s=0.5f;
        a.tuning_.state_timeout_ms=50;
        a.tuning_.remote_state_timeout_ms=1000;
        a.tuning_.teleop_enabled=false;
        a.tuning_.gesture_enabled=false;
        a.tuning_.safe_stop_enabled=false;
        a.tuning_.battery_monitor_enabled=false;
        a.estimator_.Configure(a.tuning_, spec::kLoopDt);
        a.input_.Configure(a.tuning_);
        a.input_.Update(Packet()); a.input_.Update(Packet());
        a.estimator_.Update(Packet());
        a.state_=AppState::kZeroTorque;
        a.armed_=true;
        a.tick_=1;
        LowCmdSenderTestAccess::Configure(a.sender_,a.tuning_);
        packets.clear();
    }
    static bool DisarmStep(Application& a, uint16_t buttons, bool fresh=true,
                           bool new_sample=true, float speed=0, float head_speed=0) {
        auto low=Packet(buttons,speed);
        low.motor_state()[spec::kHeadPitchIdl].dq(head_speed);
        a.estimator_.Update(low);
        a.state_fresh_=fresh;
        a.remote_state_fresh_=true;
        a.new_state_sample_=new_sample;
        a.input_sample_dt_s_=new_sample?0.01:0;
        a.input_.Update(low,true,new_sample,a.input_sample_dt_s_);
        return a.TryDisarm(a.input_.GetMergedCommand(), low);
    }
    static void Run(Application& a) {
        Setup(a);
        // Cannot disarm from powered posture, nor by bringing a held combo into Z.
        a.state_=AppState::kStandLock;
        for(int i=0;i<60;++i) CHECK(!DisarmStep(a,0x11));
        a.state_=AppState::kZeroTorque;
        for(int i=0;i<60;++i) CHECK(!DisarmStep(a,0x11));
        CHECK(!DisarmStep(a,0));
        for(int i=0;i<20;++i) CHECK(!DisarmStep(a,0x11));
        for(int i=0;i<1000;++i) CHECK(!DisarmStep(a,0x11,true,false));
        // Motion cancels earlier intent and requires another release.
        CHECK(!DisarmStep(a,0x11,true,true,0.5f));
        CHECK(!DisarmStep(a,0));
        for(int i=0;i<20;++i) CHECK(!DisarmStep(a,0x11));
        CHECK(!DisarmStep(a,0x11,true,true,0,0.2f));
        for(int i=0;i<60;++i) CHECK(!DisarmStep(a,0x11));
        CHECK(!DisarmStep(a,0));
        for(int i=0;i<20;++i) CHECK(!DisarmStep(a,0x11));
        CHECK(!DisarmStep(a,0x11,false));
        CHECK(!DisarmStep(a,0));
        for(int i=0;i<49;++i) CHECK(!DisarmStep(a,0x11));
        CHECK(DisarmStep(a,0x11));
        CHECK(a.state_==AppState::kDisarmed && !a.armed_);
        CHECK(packets.empty());
        CHECK(!a.arm_intent_latched_ && a.arm_hold_timer_==0);

        // Whole Tick stays silent after disarm, including held R1+R2.
        auto low=Packet(0x11); a.OnLowState(&low);
        for(int i=0;i<100;++i) CHECK(a.Tick());
        CHECK(a.state_==AppState::kDisarmed && !a.armed_ && packets.empty());
        CHECK(a.arm_neutral_required_);
        low=Packet(); a.OnLowState(&low); CHECK(a.Tick());
        CHECK(!a.arm_neutral_required_);
        CHECK(!a.arm_intent_latched_ && !a.armed_);

        // A latched arm intent cannot acquire authority on stale/no-new data.
        a.arm_intent_latched_=true;
        a.foreign_count_=100;
        a.last_foreign_ms_=0;
        a.new_state_sample_=false; a.state_fresh_=false; a.remote_state_fresh_=true;
        CHECK(a.TickDisarmed(InputCommand{}));
        CHECK(!a.armed_);

        // The existing, explicit E-stop+ZeroTorque semantics are retained.
        Setup(a); a.state_=AppState::kStandLock;
        low=Packet(0x0a20); a.OnLowState(&low); CHECK(a.Tick());
        CHECK(a.state_==AppState::kZeroTorque && a.armed_);
        CHECK(!packets.empty() && packets.back().motor_cmd()[0].mode()==1);
        low=Packet(0x0220); a.OnLowState(&low); CHECK(a.Tick()); // E-stop alone
        CHECK(a.state_==AppState::kIdle && a.armed_);
        CHECK(packets.back().motor_cmd()[0].mode()==0);

        // Short PD deadline expires while the longer remote deadline stays healthy.
        Setup(a);
        low=Packet(); a.OnLowState(&low);
        a.last_state_time_=std::chrono::steady_clock::now()-std::chrono::milliseconds(100);
        CHECK(a.Tick());
        CHECK(!a.state_fresh_ && a.remote_state_fresh_);
        CHECK(a.state_==AppState::kIdle);
        for(const auto& motor:packets.back().motor_cmd()) CHECK(motor.kp()==0 && motor.kd()==0);
        a.OnLowState(&low); CHECK(a.Tick());
        CHECK(a.state_fresh_ && a.state_==AppState::kIdle); // reconnect never resumes Mode Z
        CHECK(packets.back().motor_cmd()[0].mode()==0);
    }
};

int main() {
    // Never send status to a running user's integration service during tests.
    setenv("HB_INTEGRATION_SOCKET", "/tmp/hb-application-safety-test-no-listener/socket", 1);
    Application app(HB_PROJECT_PATH, "lo");
    ApplicationTestAccess::Run(app);
    std::cout << "APPLICATION_SAFETY_OK\n";
}
