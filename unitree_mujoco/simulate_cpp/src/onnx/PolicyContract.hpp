#pragma once

#include <array>
#include <string>
#include <vector>

#include "runtime/R1Config.hpp"

namespace r1::policy {

struct PolicyContract {
    std::array<float, R1Config::NUM_JOINTS> default_position{};
    std::array<float, R1Config::NUM_JOINTS> action_scale{};
    std::array<float, R1Config::NUM_JOINTS> stiffness{};
    std::array<float, R1Config::NUM_JOINTS> damping{};
};

class OnnxModel;

PolicyContract LoadPolicyContract(
    const OnnxModel& model,
    PolicyContract fallback,
    const std::vector<std::string>& expected_joint_names);

}  // namespace r1::policy
