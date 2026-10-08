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

    static void Build(LowCmdSender& sender, LowCmd_& cmd,
                      const std::array<float, spec::kNumJoints>& target,
                      bool arm_valid, float arm_kp, float arm_kd,
                      float max_rate, uint8_t mode_machine,
                      const HeadTarget& head,
                      float hold_kp = 0.0f, float hold_kd = 0.0f,
                      float hold_max_rate = 0.0f) {
        sender.BuildUpperBodyZeroTorque(cmd, target, arm_valid, arm_kp, arm_kd,
                                        max_rate, mode_machine, head,
                                        hold_kp, hold_kd, hold_max_rate);
    }

    static uint32_t ExpectedCrc(LowCmd_& cmd) {
        return LowCmdSender::Crc32(reinterpret_cast<uint32_t*>(&cmd),
                                   (sizeof(LowCmd_) >> 2) - 1);
    }
};

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

bool Near(float a, float b, float eps = 1e-6f) {
    return std::fabs(a - b) <= eps;
}

bool IsArmIdl(int idl) {
    for (int i = spec::kArmBeginPolicyIdx;
         i < spec::kArmBeginPolicyIdx + spec::kNumArmJoints; ++i) {
        if (spec::MotorIdl(i) == idl) return true;
    }
    return false;
}

bool IsHeldIdl(int idl) {
    for (int held : teleop_hold::kNonTeleopIdl) {
        if (held == idl) return true;
    }
    return false;
}

}  // namespace

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

int main() {
    Tuning tuning;
    tuning.head_yaw_kp = 7.0f;
    tuning.head_yaw_kd = 0.7f;
    tuning.head_pitch_kp = 8.0f;
    tuning.head_pitch_kd = 0.8f;

    std::array<float, spec::kNumMotorsIdl> origin{};
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl)
        origin[idl] = 0.01f * static_cast<float>(idl);

    std::array<float, spec::kNumJoints> target{};
    for (int i = 0; i < spec::kNumJoints; ++i)
        target[i] = origin[spec::MotorIdl(i)] + (i % 2 == 0 ? 1.0f : -1.0f);

    LowCmdSender sender;
    LowCmdSenderTestAccess::Configure(sender, tuning, origin);
    LowCmd_ cmd{};
    const HeadTarget head{true, 1.0f, -1.0f, 0.30f};
    LowCmdSenderTestAccess::Build(sender, cmd, target, true, 40.0f, 2.0f,
                                  0.30f, 7, head);

    constexpr float kStep = 0.30f * spec::kLoopDt;
    REQUIRE(cmd.mode_machine() == 7);
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
        const auto& motor = cmd.motor_cmd()[idl];
        REQUIRE(motor.mode() == 1);
        REQUIRE(Near(motor.tau(), 0.0f));
        REQUIRE(Near(motor.dq(), 0.0f));
        if (IsArmIdl(idl)) {
            int policy = spec::kArmBeginPolicyIdx;
            while (spec::MotorIdl(policy) != idl) ++policy;
            const float expected = origin[idl] + (policy % 2 == 0 ? kStep : -kStep);
            REQUIRE(Near(motor.q(), expected));
            REQUIRE(Near(motor.kp(), 40.0f));
            REQUIRE(Near(motor.kd(), 2.0f));
        } else if (idl == spec::kHeadYawIdl) {
            REQUIRE(Near(motor.q(), origin[idl] + kStep));
            REQUIRE(Near(motor.kp(), tuning.head_yaw_kp));
            REQUIRE(Near(motor.kd(), tuning.head_yaw_kd));
        } else if (idl == spec::kHeadPitchIdl) {
            REQUIRE(Near(motor.q(), origin[idl] - kStep));
            REQUIRE(Near(motor.kp(), tuning.head_pitch_kp));
            REQUIRE(Near(motor.kd(), tuning.head_pitch_kd));
        } else {
            // Chân, waist và cả các slot IDL không dùng phải limp tuyệt đối.
            REQUIRE(Near(motor.q(), 0.0f));
            REQUIRE(Near(motor.kp(), 0.0f));
            REQUIRE(Near(motor.kd(), 0.0f));
        }
    }
    const uint32_t crc = cmd.crc();
    REQUIRE(crc != 0u);
    REQUIRE(crc == LowCmdSenderTestAccess::ExpectedCrc(cmd));

    // Packet head-only không được cấp gain cho tay.
    LowCmdSenderTestAccess::Configure(sender, tuning, origin);
    LowCmd_ head_only{};
    LowCmdSenderTestAccess::Build(sender, head_only, target, false, 40.0f, 2.0f,
                                  0.30f, 1, head);
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
        if (idl == spec::kHeadYawIdl || idl == spec::kHeadPitchIdl) continue;
        REQUIRE(Near(head_only.motor_cmd()[idl].kp(), 0.0f));
        REQUIRE(Near(head_only.motor_cmd()[idl].kd(), 0.0f));
        REQUIRE(Near(head_only.motor_cmd()[idl].q(), 0.0f));
    }

    // Cả hai channel invalid: toàn bộ 35 motor là zero torque.
    LowCmdSenderTestAccess::Configure(sender, tuning, origin);
    LowCmd_ inactive{};
    LowCmdSenderTestAccess::Build(sender, inactive, target, false, 40.0f, 2.0f,
                                  0.30f, 1, HeadTarget{});
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
        const auto& motor = inactive.motor_cmd()[idl];
        REQUIRE(motor.mode() == 1);
        REQUIRE(Near(motor.q(), 0.0f));
        REQUIRE(Near(motor.dq(), 0.0f));
        REQUIRE(Near(motor.tau(), 0.0f));
        REQUIRE(Near(motor.kp(), 0.0f));
        REQUIRE(Near(motor.kd(), 0.0f));
    }

    // Khi bật pilot giữ chân+eo, chốt đúng encoder hiện tại và không chạm các
    // slot ngoài mapping. Một encoder NaN phải làm riêng khớp đó tiếp tục limp.
    unitree_hg::msg::dds_::LowState_ low{};
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
        low.motor_state()[idl].q() = origin[idl];
    }
    constexpr int kInvalidHeldIdl = 5;
    low.motor_state()[kInvalidHeldIdl].q() =
        std::numeric_limits<float>::quiet_NaN();

    LowCmdSenderTestAccess::Configure(sender, tuning, origin);
    sender.LatchNonTeleopHold(low);
    REQUIRE(sender.NonTeleopHoldLatched());
    LowCmd_ held{};
    LowCmdSenderTestAccess::Build(sender, held, target, true, 40.0f, 2.0f,
                                  0.30f, 1, head,
                                  20.0f, 3.0f, 0.20f);
    for (int idl = 0; idl < spec::kNumMotorsIdl; ++idl) {
        const auto& motor = held.motor_cmd()[idl];
        if (IsHeldIdl(idl) && idl != kInvalidHeldIdl) {
            REQUIRE(Near(motor.q(), origin[idl]));
            REQUIRE(Near(motor.kp(), 20.0f));
            REQUIRE(Near(motor.kd(), 3.0f));
        } else if (idl == kInvalidHeldIdl) {
            REQUIRE(Near(motor.q(), 0.0f));
            REQUIRE(Near(motor.kp(), 0.0f));
            REQUIRE(Near(motor.kd(), 0.0f));
        }
    }

    // Nhả authority phải xóa latch; dù caller còn truyền gain, chân+eo vẫn limp.
    sender.ClearNonTeleopHold();
    REQUIRE(!sender.NonTeleopHoldLatched());
    LowCmd_ released{};
    LowCmdSenderTestAccess::Build(sender, released, target, false, 40.0f, 2.0f,
                                  0.30f, 1, HeadTarget{},
                                  20.0f, 3.0f, 0.20f);
    for (int idl : teleop_hold::kNonTeleopIdl) {
        REQUIRE(Near(released.motor_cmd()[idl].q(), 0.0f));
        REQUIRE(Near(released.motor_cmd()[idl].kp(), 0.0f));
        REQUIRE(Near(released.motor_cmd()[idl].kd(), 0.0f));
    }
    return 0;
}
