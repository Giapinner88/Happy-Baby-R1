#include "controllers/rma_meta/RmaMetaRuntime.hpp"

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

void RequireFiniteAction(const std::vector<float>& action) {
    Require(action.size() == 24, "RMA-meta action must be 24-D");
    for (const float value : action) {
        Require(std::isfinite(value), "RMA-meta action must stay finite");
    }
}

void RequireSameAction(
    const std::vector<float>& action,
    const std::vector<float>& expected) {
    Require(action.size() == expected.size(), "held RMA-meta action size changed");
    for (std::size_t index = 0; index < action.size(); ++index) {
        Require(
            std::isfinite(action[index]) && action[index] == expected[index],
            "RMA-meta safety hold changed the last finite target");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::invalid_argument("usage: rma_meta_runtime_test PACKAGE_DIR");
    }

    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "rma_meta_runtime_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    r1::rma_meta::RmaMetaRuntime runtime(
        std::filesystem::path(argv[1]), environment, options);

    Require(runtime.ObservationSize() == 83, "RMA-meta observation must be 83-D");
    Require(runtime.HistorySteps() == 50, "RMA-meta history must be 50 frames");
    Require(runtime.MetaDecimation() == 5, "RMA-meta selector must run at 10 Hz");
    Require(
        runtime.ExpertNames()
            == std::vector<std::string>({"slope_up", "slope_down", "flat"}),
        "RMA-meta expert registry order differs");
    Require(runtime.SelectedClass() == "flat", "RMA-meta must reset to Flat");
    Require(!runtime.SafetyHold(), "finite RMA-meta reset must not safety-hold");

    std::vector<float> observation(83, 0.0f);
    observation[5] = -1.0f;
    std::vector<float> last_action = runtime.Step(observation);
    RequireFiniteAction(last_action);
    const std::vector<float> expected_first_latent = {
        -0.50949657f, 0.25334689f, -0.09345219f, -0.48717579f,
        -0.36004493f, 0.16065988f, 0.60474229f, -0.62863600f};
    const std::vector<float> expected_first_probabilities = {
        0.98225176f, 1.00143938e-08f, 3.16870299e-08f, 0.01774821f};
    for (std::size_t index = 0; index < expected_first_latent.size(); ++index) {
        Require(
            std::abs(runtime.Latent()[index] - expected_first_latent[index]) < 2.0e-5f,
            "RMA-meta C++ adapter differs from Python ONNX Runtime");
    }
    for (std::size_t index = 0;
         index < expected_first_probabilities.size();
         ++index) {
        Require(
            std::abs(runtime.Probabilities()[index]
                     - expected_first_probabilities[index]) < 2.0e-5f,
            "RMA-meta C++ selector differs from Python ONNX Runtime");
    }
    for (int step = 1; step < 60; ++step) {
        last_action = runtime.Step(observation);
        RequireFiniteAction(last_action);
    }
    Require(!runtime.FaultLatched(), "finite RMA-meta inference latched a fault");
    Require(runtime.HistoryValidSteps() == 50, "RMA-meta history did not fill");
    Require(runtime.MetaDecisionCount() == 12, "RMA-meta 10 Hz decision count differs");
    Require(runtime.Probabilities().size() == 4, "RMA-meta must output four classes");
    Require(runtime.Latent().size() == 8, "RMA-meta latent must be 8-D");

    observation[0] = std::numeric_limits<float>::quiet_NaN();
    const auto held_action = runtime.Step(observation);
    Require(runtime.SafetyHold(), "invalid RMA-meta input must safety-hold");
    RequireSameAction(held_action, last_action);
    for (int step = 1; step < runtime.SafetyHoldWarningSteps(); ++step) {
        RequireSameAction(runtime.Step(observation), held_action);
    }
    Require(runtime.FaultLatched(), "persistent invalid RMA-meta input must latch");

    runtime.Reset();
    Require(
        !runtime.SafetyHold() && !runtime.FaultLatched()
            && runtime.SelectedClass() == "flat"
            && runtime.HistoryValidSteps() == 0,
        "RMA-meta reset did not restore the Flat clean state");
    return 0;
}
