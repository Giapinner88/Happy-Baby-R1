#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace r1::slope_meta {

struct CommandAxisConfig {
    float minimum = 0.0f;
    float maximum = 0.0f;
    float slew_rate = 0.0f;
};

// Meta-only command boundary. It prevents commands from accumulating while the
// recurrent gate is in SAFETY_HOLD, clamps them to the expert training range,
// and releases them gradually after the gate becomes ready.
class SlopeMetaCommandGovernor {
public:
    void Configure(const std::array<CommandAxisConfig, 3>& config) {
        for (const auto& axis : config) {
            if (!std::isfinite(axis.minimum) || !std::isfinite(axis.maximum)
                || !std::isfinite(axis.slew_rate)
                || axis.minimum > 0.0f || axis.maximum < 0.0f
                || axis.minimum >= axis.maximum || axis.slew_rate <= 0.0f) {
                throw std::invalid_argument("invalid slope meta command governor config");
            }
        }
        config_ = config;
        configured_ = true;
        Reset();
    }

    std::array<float, 3> Update(
        const std::array<float, 3>& requested,
        bool commands_allowed,
        float dt) {
        if (!configured_) {
            throw std::logic_error("slope meta command governor is not configured");
        }
        if (!commands_allowed) {
            Reset();
            return command_;
        }
        if (!std::isfinite(dt) || dt <= 0.0f) {
            throw std::invalid_argument("slope meta command governor dt must be positive");
        }

        for (std::size_t axis = 0; axis < command_.size(); ++axis) {
            const float finite_request = std::isfinite(requested[axis])
                ? requested[axis] : 0.0f;
            const float target = std::clamp(
                finite_request, config_[axis].minimum, config_[axis].maximum);
            const float maximum_step = config_[axis].slew_rate * dt;
            command_[axis] += std::clamp(
                target - command_[axis], -maximum_step, maximum_step);
        }
        return command_;
    }

    void Reset() { command_.fill(0.0f); }

    const std::array<float, 3>& Command() const { return command_; }

private:
    std::array<CommandAxisConfig, 3> config_{};
    std::array<float, 3> command_{};
    bool configured_ = false;
};

}  // namespace r1::slope_meta
