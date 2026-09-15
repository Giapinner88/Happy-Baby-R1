#pragma once

#include "PolicyController.hpp"

// V10-only 105-D decoder. It uses gesture references now and never masks
// observations or overwrites arms. V11 must add its own controller/teleop
// adapter and contract test before it can be enabled.
class UnifiedV10Controller final : public PolicyController {
public:
    static constexpr int kObsSize = spec::kFlatObsSize + 2 + 2 * spec::kNumArmJoints;
    static constexpr int kCrouchOffset = spec::kFlatObsSize;
    static constexpr int kArmQRefOffset = kCrouchOffset + 2;
    static constexpr int kArmDqRefOffset = kArmQRefOffset + spec::kNumArmJoints;
    static_assert(kObsSize == 105, "Unified observation contract must remain 105-D");

    int ObsSize() const override { return kObsSize; }
    std::string Name() const override { return "UnifiedV10"; }
    bool UsesArmReference() const override { return true; }

protected:
    std::string ExpectedPolicyContract() const override { return "r1_unified_v10"; }
    std::string ExpectedObservationLayout() const override {
        return "base_ang_vel[3],projected_gravity[3],twist[3],phase[2],"
               "joint_pos[24],joint_vel[24],last_action[24],"
               "crouch[2],arm_q_ref[10],arm_dq_ref[10]";
    }

    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const RobotState& s = ctx.state;
        obs[0] = s.gyro.x();
        obs[1] = s.gyro.y();
        obs[2] = s.gyro.z();
        obs[3] = s.projected_gravity.x();
        obs[4] = s.projected_gravity.y();
        obs[5] = s.projected_gravity.z();
        obs[6] = ctx.cmd_vx;
        obs[7] = ctx.cmd_vy;
        obs[8] = ctx.cmd_yaw;
        obs[9] = ctx.gait_phase[0];
        obs[10] = ctx.gait_phase[1];
        for (int i = 0; i < spec::kNumJoints; ++i) {
            obs[11 + i] = s.q[i] - default_q_[i];
            obs[35 + i] = s.dq[i];
            obs[59 + i] = last_action_[i];
        }
        obs[kCrouchOffset] = ctx.crouch_alpha;
        obs[kCrouchOffset + 1] = ctx.crouch_alpha_rate;
        for (int j = 0; j < spec::kNumArmJoints; ++j) {
            obs[kArmQRefOffset + j] = ctx.arm_q_ref[j];
            obs[kArmDqRefOffset + j] = ctx.arm_dq_ref[j];
        }
    }
};
