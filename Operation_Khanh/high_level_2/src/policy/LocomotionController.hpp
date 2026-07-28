#pragma once

#include "PolicyController.hpp"

// Controller cho chính sách đi bộ (Locomotion flat policy)
class LocomotionController : public PolicyController {
public:
    int ObsSize() const override { return spec::kFlatObsSize; }
    std::string Name() const override { return "Locomotion"; }

protected:
    // Xây dựng vector quan sát 83 chiều (Flat Obs) khớp với ONNX metadata
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

        obs[9]  = ctx.gait_phase[0];
        obs[10] = ctx.gait_phase[1];

        for (int i = 0; i < spec::kNumJoints; ++i) {
            obs[11 + i] = s.q[i] - default_q_[i];
            obs[35 + i] = s.dq[i];
            obs[59 + i] = last_action_[i];
        }
    }
};
