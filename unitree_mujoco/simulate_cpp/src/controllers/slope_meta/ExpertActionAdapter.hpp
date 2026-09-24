#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace r1::slope_meta {

struct JointActionContract {
    std::vector<float> scale;
    std::vector<float> offset;
};

inline void ValidateContract(
    const JointActionContract& contract,
    std::size_t action_size) {
    if (contract.scale.size() != action_size
        || contract.offset.size() != action_size) {
        throw std::runtime_error("joint action contract size mismatch");
    }
    for (const float scale : contract.scale) {
        if (!std::isfinite(scale) || std::abs(scale) < 1.0e-8f) {
            throw std::runtime_error("joint action scale is invalid");
        }
    }
    for (const float offset : contract.offset) {
        if (!std::isfinite(offset)) {
            throw std::runtime_error("joint action offset is invalid");
        }
    }
}

inline std::vector<float> AdaptObservation(
    const std::vector<float>& observation,
    const JointActionContract& canonical,
    const JointActionContract& expert,
    std::size_t joint_position_offset,
    std::size_t previous_action_offset) {
    const std::size_t action_size = canonical.scale.size();
    ValidateContract(canonical, action_size);
    ValidateContract(expert, action_size);
    if (joint_position_offset + action_size > observation.size()
        || previous_action_offset + action_size > observation.size()) {
        throw std::runtime_error("expert observation offsets are invalid");
    }

    std::vector<float> result = observation;
    for (std::size_t joint = 0; joint < action_size; ++joint) {
        result[joint_position_offset + joint] =
            observation[joint_position_offset + joint]
            + canonical.offset[joint] - expert.offset[joint];
        const float previous_target = canonical.offset[joint]
            + canonical.scale[joint] * observation[previous_action_offset + joint];
        result[previous_action_offset + joint] =
            (previous_target - expert.offset[joint]) / expert.scale[joint];
    }
    return result;
}

inline std::vector<float> RawActionToJointTarget(
    const std::vector<float>& raw_action,
    const JointActionContract& contract) {
    ValidateContract(contract, raw_action.size());
    std::vector<float> target(raw_action.size());
    for (std::size_t joint = 0; joint < raw_action.size(); ++joint) {
        target[joint] = contract.offset[joint]
            + contract.scale[joint] * raw_action[joint];
    }
    return target;
}

inline std::vector<float> JointTargetToRawAction(
    const std::vector<float>& target,
    const JointActionContract& contract) {
    ValidateContract(contract, target.size());
    std::vector<float> raw_action(target.size());
    for (std::size_t joint = 0; joint < target.size(); ++joint) {
        raw_action[joint] =
            (target[joint] - contract.offset[joint]) / contract.scale[joint];
    }
    return raw_action;
}

}  // namespace r1::slope_meta
