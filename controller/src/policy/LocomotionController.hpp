#pragma once

#include "BaseObservation83.hpp"
#include "PolicyController.hpp"

// Controller cho chính sách đi bộ (Locomotion flat policy)
class LocomotionController : public PolicyController {
public:
    int ObsSize() const override { return spec::kFlatObsSize; }
    std::string Name() const override { return "Locomotion"; }

    // Đường duy nhất được phép chạy overlay tay kiểu cũ.
    bool SupportsArmOverlay() const override { return true; }

protected:
    // Xây dựng vector quan sát 83 chiều (Flat Obs) khớp với ONNX metadata
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        r1::obs83::Build(ctx, default_q_, last_action_, obs);

        // Overlay động tác tay: che q_rel/dq của tay (idx 14..23), GIỮ last_action,
        // + bù thăng bằng feedforward (chỉ obs[3]/obs[6]). NO-OP khi không có gesture.
        if (ctx.arm_override_active) {
            r1::obs83::MaskArmState(obs, ctx.arm_mask_keep);
            obs[3] += ctx.gravity_x_bias;
            obs[6] -= ctx.cmd_vx_bias;
        }
    }
};
