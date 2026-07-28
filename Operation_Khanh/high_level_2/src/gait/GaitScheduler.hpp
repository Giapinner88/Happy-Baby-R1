#pragma once

#include <array>
#include <cmath>

#include "../config/RobotSpec.hpp"

// Đồng hồ tạo pha di chuyển (gait clock)
class GaitScheduler {
public:
    void Reset() { time_ = 0.0f; }

    void Update(float dt) {
        time_ += dt;
        if (time_ > 3600.0f) time_ = std::fmod(time_, spec::kGaitPeriodS);
    }

    // Trả về pha obs (sin/cos) dựa trên chuẩn hóa lệnh
    std::array<float, 2> PhaseObs(float cmd_norm) const {
        if (cmd_norm < spec::kCmdGateNorm) return {0.0f, 0.0f};
        float ratio = std::fmod(time_, spec::kGaitPeriodS) / spec::kGaitPeriodS;
        return {std::sin(2.0f * static_cast<float>(M_PI) * ratio),
                std::cos(2.0f * static_cast<float>(M_PI) * ratio)};
    }

private:
    float time_ = 0.0f;
};
