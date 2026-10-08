// Cổng an toàn cuối của LowCmdSender: không một giá trị không hữu hạn nào được
// vào packet motor, và một frame hỏng không được làm hỏng vĩnh viễn last_cmd_q_.
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "robot/LowCmdSender.hpp"

using LowCmd_ = unitree_hg::msg::dds_::LowCmd_;

struct LowCmdSenderTestAccess {
    static void Configure(LowCmdSender& sender, const Tuning& tuning,
                          const std::array<float, spec::kNumMotorsIdl>& origin) {
        sender.tuning_ = &tuning;
        sender.last_cmd_q_ = origin;
    }
    static void BuildSend(LowCmdSender& sender, LowCmd_& cmd,
                          const std::array<float, spec::kNumJoints>& target,
                          const std::array<float, spec::kNumJoints>& kp,
                          const std::array<float, spec::kNumJoints>& kd,
                          float max_rate, const HeadTarget& head = {}) {
        sender.BuildSend(cmd, target, kp, kd, max_rate, 0, head);
    }
    static float LastCmd(const LowCmdSender& sender, int idl) {
        return sender.last_cmd_q_[static_cast<size_t>(idl)];
    }
};

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

struct Rig {
    Tuning tuning;
    LowCmdSender sender;
    std::array<float, spec::kNumJoints> target{};
    std::array<float, spec::kNumJoints> kp{};
    std::array<float, spec::kNumJoints> kd{};
    LowCmd_ cmd{};

    Rig() {
        std::array<float, spec::kNumMotorsIdl> origin{};
        origin.fill(0.0f);
        LowCmdSenderTestAccess::Configure(sender, tuning, origin);
        target.fill(0.0f);
        kp.fill(100.0f);
        kd.fill(3.0f);
    }
    void Send(float max_rate = 30.0f, const HeadTarget& head = {}) {
        LowCmdSenderTestAccess::BuildSend(sender, cmd, target, kp, kd, max_rate, head);
    }
    float Q(int policy_idx) const {
        return cmd.motor_cmd()[static_cast<size_t>(spec::MotorIdl(policy_idx))].q();
    }
    float Kp(int policy_idx) const {
        return cmd.motor_cmd()[static_cast<size_t>(spec::MotorIdl(policy_idx))].kp();
    }
};

// Không một trường nào của packet được phép là NaN/Inf, kể cả khi đầu vào bẩn.
void RequirePacketFinite(const LowCmd_& cmd) {
    for (int i = 0; i < spec::kNumMotorsIdl; ++i) {
        const auto& m = cmd.motor_cmd()[static_cast<size_t>(i)];
        REQUIRE(std::isfinite(m.q()));
        REQUIRE(std::isfinite(m.kp()));
        REQUIRE(std::isfinite(m.kd()));
        REQUIRE(std::isfinite(m.dq()));
        REQUIRE(std::isfinite(m.tau()));
    }
}

void TestTargetNotFinite() {
    Rig rig;
    rig.target[5] = kNaN;
    rig.target[7] = kInf;
    rig.target[3] = 0.4f;          // khớp lành vẫn phải chạy bình thường
    rig.Send();
    RequirePacketFinite(rig.cmd);
    REQUIRE(rig.Q(5) == 0.0f);     // giữ nguyên lệnh cũ
    REQUIRE(rig.Q(7) == 0.0f);
    REQUIRE(rig.Q(3) > 0.0f);
    REQUIRE(rig.sender.HasFault());
}

// Lỗi cũ: std::clamp(NaN) trả NaN -> last_cmd_q_ nhiễm và khớp hỏng vĩnh viễn
// kể cả sau khi policy trả lại số hợp lệ.
void TestNanDoesNotPoisonState() {
    Rig rig;
    rig.target[5] = kNaN;
    rig.Send();
    rig.sender.ClearFault();

    rig.target[5] = 0.2f;
    for (int i = 0; i < 200; ++i) rig.Send();
    REQUIRE(std::isfinite(rig.Q(5)));
    REQUIRE(std::fabs(rig.Q(5) - 0.2f) < 1e-3f);
    REQUIRE(!rig.sender.HasFault());
    REQUIRE(std::isfinite(LowCmdSenderTestAccess::LastCmd(rig.sender, spec::MotorIdl(5))));
}

// Vượt cổng thì clamp và đếm, KHÔNG được đổi trạng thái (damping giữa lúc đang
// dồn trọng lượng lên một chân còn nguy hơn giá trị bị cắt).
void TestOutOfGuardClampsWithoutFault() {
    Rig rig;
    rig.target[5] = 1.0e6f;
    const uint64_t before = rig.sender.ClampCount();
    for (int i = 0; i < 5000; ++i) rig.Send();
    REQUIRE(rig.sender.ClampCount() > before);
    REQUIRE(!rig.sender.HasFault());
    REQUIRE(rig.Q(5) <= spec::kJointGuardMax[5] + 1e-6f);
}

// Cổng phải rộng hơn mọi quỹ đạo đang chạy: ankle_roll trong getup.npz chạm
// 0.369 rad (vượt MJCF ±0.2618) và KHÔNG được cắt ở đây — siết về tầm cơ khí
// là việc riêng, phải đo trên robot trước.
void TestShippedMotionNotClamped() {
    Rig rig;
    const uint64_t before = rig.sender.ClampCount();
    rig.target[5]  = 0.369f;
    rig.target[11] = -0.342f;
    rig.target[0]  = -2.948f;
    rig.target[3]  = 2.442f;
    for (int i = 0; i < 5000; ++i) rig.Send();
    REQUIRE(rig.sender.ClampCount() == before);
    REQUIRE(std::fabs(rig.Q(5) - 0.369f) < 1e-3f);
}

void TestGainGate() {
    Rig rig;
    rig.kp[2] = kNaN;
    rig.kd[4] = kInf;
    rig.kp[6] = 1.0e9f;
    rig.kp[8] = -50.0f;
    rig.Send();
    RequirePacketFinite(rig.cmd);
    REQUIRE(rig.Kp(2) == 0.0f);
    REQUIRE(rig.Kp(6) == spec::kMaxCmdKp);
    REQUIRE(rig.Kp(8) == 0.0f);
    REQUIRE(rig.sender.HasFault());
}

// max_rate NaN không phải "giới hạn rộng" mà là MẤT HẲN rate-limit:
// std::clamp(delta, NaN, NaN) trả delta nguyên vẹn -> nhảy thẳng tới target.
void TestRateGate() {
    Rig rig;
    rig.target[3] = 2.0f;
    rig.Send(kNaN);
    REQUIRE(rig.Q(3) == 0.0f);
    REQUIRE(rig.sender.HasFault());
    rig.sender.ClearFault();

    rig.Send(1.0e9f);              // trần rate: 200 rad/s -> 0.4 rad mỗi frame
    REQUIRE(rig.Q(3) <= spec::kMaxCmdRate * spec::kLoopDt + 1e-6f);
}

// Backstop cuối: LowState cũ -> mọi gain = 0 ở lớp DUY NHẤT ghi rt/lowcmd,
// không phụ thuộc watchdog trạng thái hay gate riêng của từng chế độ.
void TestStaleSensorDropsAllGains() {
    Rig rig;
    rig.kp.fill(220.0f);
    rig.kd.fill(4.0f);
    rig.target[3] = 0.5f;
    rig.sender.SetSensorFresh(false);
    HeadTarget head{true, 0.2f, 0.1f, 1.5f};
    rig.Send(30.0f, head);
    for (int i = 0; i < spec::kNumJoints; ++i) {
        REQUIRE(rig.Kp(i) == 0.0f);
        REQUIRE(rig.cmd.motor_cmd()[static_cast<size_t>(spec::MotorIdl(i))].kd() == 0.0f);
    }
    REQUIRE(rig.cmd.motor_cmd()[spec::kHeadYawIdl].kp() == 0.0f);
    REQUIRE(rig.cmd.motor_cmd()[spec::kHeadPitchIdl].kp() == 0.0f);

    rig.sender.SetSensorFresh(true);
    rig.Send(30.0f, head);
    REQUIRE(rig.Kp(3) == 220.0f);
}

void TestHeadGate() {
    Rig rig;
    HeadTarget head{true, kNaN, 5.0f, 1.5f};
    rig.Send(30.0f, head);
    RequirePacketFinite(rig.cmd);
    REQUIRE(rig.sender.HasFault());
    const float pitch = rig.cmd.motor_cmd()[spec::kHeadPitchIdl].q();
    REQUIRE(pitch >= spec::kHeadPitchMin && pitch <= spec::kHeadPitchMax);
    // Both signs, both axes, through the actual packet builder and slew.
    for (float sign : {-1.0f, 1.0f}) {
        head = HeadTarget{true, sign * 5.0f, sign * 5.0f, 1.5f};
        for (int i = 0; i < 1000; ++i) rig.Send(30.0f, head);
        const float yaw = rig.cmd.motor_cmd()[spec::kHeadYawIdl].q();
        const float p = rig.cmd.motor_cmd()[spec::kHeadPitchIdl].q();
        REQUIRE(std::fabs(yaw - (sign < 0 ? spec::kHeadYawMin : spec::kHeadYawMax)) < 1e-5f);
        REQUIRE(std::fabs(p - (sign < 0 ? spec::kHeadPitchMin : spec::kHeadPitchMax)) < 1e-5f);
    }
}

void TestInvalidHeadReturnsAtTeleopRate() {
    Rig rig;
    std::array<float, spec::kNumMotorsIdl> origin{};
    origin[spec::kHeadYawIdl] = 0.2f;
    LowCmdSenderTestAccess::Configure(rig.sender, rig.tuning, origin);
    rig.Send(30.0f, HeadTarget{false, 0.0f, 0.0f, 1.5f});
    const float yaw = rig.cmd.motor_cmd()[spec::kHeadYawIdl].q();
    REQUIRE(yaw < 0.2f);
    REQUIRE(yaw >= 0.2f - 1.5f * spec::kLoopDt - 1e-6f);
}

}  // namespace

int main() {
    TestTargetNotFinite();
    TestNanDoesNotPoisonState();
    TestOutOfGuardClampsWithoutFault();
    TestShippedMotionNotClamped();
    TestGainGate();
    TestStaleSensorDropsAllGains();
    TestRateGate();
    TestHeadGate();
    TestInvalidHeadReturnsAtTeleopRate();
    std::cout << "HB_LOWCMD_SAFETY_GATE_OK guard_slack=" << spec::kGuardSlack
              << " max_kp=" << spec::kMaxCmdKp
              << " max_rate=" << spec::kMaxCmdRate << "\n";
    return 0;
}
