#include "controllers/slope_meta/SlopeMetaRuntime.hpp"

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void RequireSameAction(
    const std::vector<float>& action,
    const std::vector<float>& expected) {
    Require(action.size() == expected.size(), "held action dimension changed");
    for (std::size_t index = 0; index < action.size(); ++index) {
        Require(std::isfinite(action[index]) && action[index] == expected[index],
                "SAFETY_HOLD changed the last valid meta_policy target");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::invalid_argument("usage: meta_policy_runtime_test PACKAGE_DIR");
    }

    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "meta_policy_runtime_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    r1::slope_meta::SlopeMetaRuntime runtime(
        std::filesystem::path(argv[1]), environment, options);

    Require(runtime.ObservationSize() == 83, "meta_policy observation must be 83-D");
    Require(runtime.ExpertNames()
                == std::vector<std::string>({"slope_up", "slope_down", "flat"}),
            "meta_policy registry order mismatch");
    Require(runtime.Weights().size() == 3
                && runtime.Weights()[0] == 0.0f
                && runtime.Weights()[1] == 0.0f
                && runtime.Weights()[2] == 1.0f,
            "meta_policy must bootstrap with the Flat expert");
    Require(!runtime.SafetyHold(),
            "normal meta_policy reset must not freeze before the first inference");

    std::vector<float> observation(83, 0.0f);
    observation[5] = -1.0f;
    const auto first_action = runtime.Step(observation);
    Require(first_action.size() == 24 && runtime.StartupActive(),
            "meta_policy did not start active 50-frame history collection");
    Require(!runtime.SafetyHold(),
            "finite meta_policy cold start unexpectedly entered SAFETY_HOLD");
    for (const float value : first_action) {
        Require(std::isfinite(value), "Flat cold-start action is not finite");
    }

    observation[0] = std::numeric_limits<float>::quiet_NaN();
    const auto held_action = runtime.Step(observation);
    Require(runtime.SafetyHold(), "invalid meta_policy observation must hold");
    RequireSameAction(held_action, first_action);
    for (int step = 1; step < 100; ++step) {
        RequireSameAction(runtime.Step(observation), held_action);
    }
    Require(runtime.FaultLatched(),
            "prolonged invalid meta_policy input must latch the fault");

    runtime.Reset();
    Require(!runtime.SafetyHold() && !runtime.FaultLatched(),
            "meta_policy reset did not restore active Flat fallback");
    return 0;
}
