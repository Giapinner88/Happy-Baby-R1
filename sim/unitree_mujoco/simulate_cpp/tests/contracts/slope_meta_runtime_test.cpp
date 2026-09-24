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
                "SAFETY_HOLD changed the last commanded target");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::invalid_argument("usage: slope_meta_runtime_test PACKAGE_DIR");
    }

    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "slope_meta_runtime_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    r1::slope_meta::SlopeMetaRuntime runtime(
        std::filesystem::path(argv[1]), environment, options);

    Require(runtime.ObservationSize() == 83, "gate actor observation must be 83-D");
    Require(runtime.CommonContract().scale.size() == 24,
            "common action scale must be 24-D");
    Require(runtime.ExpertNames()
                == std::vector<std::string>({"slope_up", "slope_down"}),
            "registry expert order mismatch");
    Require(runtime.SafetyHoldWarningSteps() == 100,
            "registry warning duration mismatch");

    std::vector<float> observation(83, 0.0f);
    observation[5] = -1.0f;
    std::vector<float> startup_action;
    for (int step = 0; step < 50; ++step) {
        startup_action = runtime.Step(observation);
        Require(startup_action.size() == 24, "startup action must be 24-D");
        Require(runtime.StartupActive(), "cold start ended too early");
        for (const float value : startup_action) {
            Require(std::isfinite(value), "startup expert returned non-finite action");
        }
    }
    Require(!runtime.FaultLatched(), "expected cold start must not latch a fault");
    Require(runtime.ConsecutiveSafetyHoldSteps() == 0,
            "active startup balance must not count as a post-startup hold");

    const auto released_action = runtime.Step(observation);
    Require(!runtime.StartupActive(), "cold start did not finish at frame 51");
    Require(!runtime.SafetyHold(), "confident NEUTRAL did not release cold start");
    Require(runtime.SelectedClass() == "NEUTRAL",
            "upright startup should remain NEUTRAL");
    Require(runtime.Weights().size() == 2
                && runtime.Weights()[0] == 0.5f
                && runtime.Weights()[1] == 0.5f,
            "NEUTRAL startup must keep the unbiased two-expert probe blend");
    Require(released_action.size() == 24, "released action must remain 24-D");

    observation[0] = std::numeric_limits<float>::quiet_NaN();
    for (int step = 0; step < 100; ++step) {
        RequireSameAction(runtime.Step(observation), released_action);
    }
    Require(runtime.FaultLatched(), "prolonged invalid input must latch hold fault");
    observation[0] = 0.0f;
    RequireSameAction(runtime.Step(observation), released_action);
    Require(runtime.FaultLatched(), "latched fault must persist until reset");

    runtime.Reset();
    Require(!runtime.FaultLatched(), "reset did not clear latched fault");
    const auto reset_action = runtime.Step(observation);
    Require(reset_action.size() == 24 && runtime.StartupActive(),
            "reset did not restart active balance cold start");
    for (const float value : reset_action) {
        Require(std::isfinite(value), "reset startup action is not finite");
    }
    return 0;
}
