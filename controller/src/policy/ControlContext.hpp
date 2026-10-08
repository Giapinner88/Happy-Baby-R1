#pragma once

// Dữ liệu điều khiển truyền vào controller mỗi policy step.
//
// Tách khỏi PolicyController.hpp để bộ dựng observation THUẦN
// (BaseObservation83.hpp) không phải kéo theo OnnxPolicy và cả ONNX runtime.
// Nhờ vậy golden regression của frame 83-D link được mà không cần
// onnxruntime, và "thuần" là thuộc tính có thật chứ không chỉ là lời nói.

#include <array>

#include "../config/RobotSpec.hpp"
#include "../estimation/StateEstimator.hpp"

// Dữ liệu điều khiển cung cấp cho controller tại mỗi bước
struct ControlContext {
    const RobotState& state;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_yaw = 0.0f;
    std::array<float, 2> gait_phase{0.0f, 0.0f};

    // Overlay động tác tay (mặc định TẮT -> obs dựng y hệt khi không có gesture).
    // arm_mask_keep = (1 - weight): che q_rel/dq của tay (giữ last_action).
    // gravity_x_bias / cmd_vx_bias: feedforward bù thăng bằng, CHỈ chạm obs[3]/obs[6]
    // (KHÔNG đi qua cmd_vx thật để không nhiễu cmd_norm/gait).
    bool  arm_override_active = false;
    float arm_mask_keep       = 1.0f;
    float gravity_x_bias      = 0.0f;
    float cmd_vx_bias         = 0.0f;

};
