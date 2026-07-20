#pragma once
/**
 * GaitScheduler.hpp — Đồng hồ gait (Gait clock).
 */

#include <array>
#include <cmath>

#include "../config/RobotSpec.hpp"

class GaitScheduler {
public:
    void Reset() { time_ = 0.0f; }

    void Update(float dt) {
        time_ += dt;
        if (time_ > 3600.0f) time_ = std::fmod(time_, spec::kGaitPeriodS);
    }

    std::array<float, 2> PhaseObs(float cmd_norm) const {
        if (cmd_norm < spec::kCmdGateNorm) return {0.0f, 0.0f};
        float ratio = std::fmod(time_, spec::kGaitPeriodS) / spec::kGaitPeriodS;
        return {std::sin(2.0f * static_cast<float>(M_PI) * ratio),
                std::cos(2.0f * static_cast<float>(M_PI) * ratio)};
    }

private:
    float time_ = 0.0f;
};
