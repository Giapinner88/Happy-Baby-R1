#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "../config/RobotSpec.hpp"

// Native 09/09 observation: episode_length * dt, gated by command 3-norm.
// A policy reset is the deployed analogue of an episode reset. The first
// observation sees t=0; STAND/zero command masks phase without stopping time.
class CommandGatedPhase {
public:
    void Configure(float period_s) {
        if (!std::isfinite(period_s) || period_s <= 0.0f)
            throw std::invalid_argument("invalid command-gated phase period");
        period_s_ = period_s;
    }

    void Reset() { step_ = 0; }

    std::array<float, 2> Next(float vx, float vy, float yaw) {
        // Match Torch's float32 integer-step multiplication, rather than a
        // drifting sum of dt. Increment even while the output is masked.
        // Python's step_dt is the literal0.02. kLoopDt*decimation rounds to
        // a neighboring float on this runtime, which accumulates phase error.
        constexpr float native_step_dt = 0.02f;
        const float elapsed = static_cast<float>(step_++) * native_step_dt;
        if (std::sqrt(vx * vx + vy * vy + yaw * yaw) < spec::kCmdGateNorm)
            return {0.0f, 0.0f};
        const float ratio = std::fmod(elapsed, period_s_) / period_s_;
        const float angle = ratio * static_cast<float>(M_PI) * 2.0f;
        return {std::sin(angle), std::cos(angle)};
    }

private:
    std::uint64_t step_ = 0;
    float period_s_ = spec::kGaitPeriodS;
};
