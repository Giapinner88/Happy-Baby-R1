#include "controllers/arma/ArmaRuntime.hpp"

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

void RequireFinite(const std::vector<float>& values, const char* label) {
    for (const float value : values) {
        Require(std::isfinite(value), std::string(label) + " must stay finite");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::invalid_argument("usage: arma_runtime_test PACKAGE_DIR");
    }

    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "arma_runtime_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    r1::arma::ArmaRuntime runtime(
        std::filesystem::path(argv[1]), environment, options);

    Require(runtime.BundleContract() == "flat_plus_arma_o83_k50_z8_v1",
            "unexpected A-RMA bundle contract");
    Require(runtime.ActorView() == "base83", "o83 bundle must use base83 view");
    Require(runtime.HistorySteps() == 50, "adapter history must be K=50");
    Require(runtime.LatentSize() == 8, "latent must be 8-D");
    Require(runtime.ActorInputSize() == 91, "actor input must be 91-D");

    std::vector<float> frame(83, 0.0f);
    for (int step = 0; step < 60; ++step) {
        const auto latent = runtime.Adapt(frame);
        Require(latent.size() == 8, "adapter latent dimension changed");
        RequireFinite(latent, "adapter latent");

        std::vector<float> actor_input = frame;
        actor_input.insert(actor_input.end(), latent.begin(), latent.end());
        const auto action = runtime.RunActor(actor_input);
        Require(action.size() == 24, "actor action dimension changed");
        RequireFinite(action, "actor action");
    }
    Require(!runtime.SafetyHold(), "finite inference entered safety hold");
    Require(runtime.LatentAgeSteps() <= runtime.MaxLatentAgeSteps(),
            "latent age exceeded configured bound");

    frame[0] = std::numeric_limits<float>::quiet_NaN();
    const auto held_latent = runtime.Adapt(frame);
    Require(runtime.SafetyHold(), "non-finite frame did not safety-hold");
    RequireFinite(held_latent, "held latent");

    runtime.Reset();
    Require(!runtime.SafetyHold(), "reset did not clear safety hold");
    Require(runtime.LatentAgeSteps() == 0, "reset did not clear latent age");
    return 0;
}
