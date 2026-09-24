#include "controllers/slope_meta/ExpertActionAdapter.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

void RequireNear(float actual, float expected) {
    if (std::abs(actual - expected) > 1.0e-6f) {
        throw std::runtime_error("expert action adapter value mismatch");
    }
}

}  // namespace

int main() {
    const r1::slope_meta::JointActionContract canonical{
        {0.2f, 0.3f}, {1.0f, -1.0f}};
    const r1::slope_meta::JointActionContract expert{
        {0.5f, 0.25f}, {0.8f, -0.7f}};
    const std::vector<float> observation{0.4f, -0.2f, 0.5f, -0.5f};

    const auto adapted = r1::slope_meta::AdaptObservation(
        observation, canonical, expert, 0, 2);
    RequireNear(adapted[0], 0.6f);
    RequireNear(adapted[1], -0.5f);
    RequireNear(adapted[2], 0.6f);
    RequireNear(adapted[3], -1.8f);

    const auto target = r1::slope_meta::RawActionToJointTarget(
        {0.6f, -1.8f}, expert);
    RequireNear(target[0], 1.1f);
    RequireNear(target[1], -1.15f);
    const auto common_raw =
        r1::slope_meta::JointTargetToRawAction(target, canonical);
    RequireNear(common_raw[0], 0.5f);
    RequireNear(common_raw[1], -0.5f);
    return 0;
}
