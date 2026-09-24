#include "controllers/slope_meta/SlopeMetaRuntime.hpp"
#include "onnx/OnnxModel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

double Percentile(const std::vector<double>& sorted, double probability) {
    const auto index = static_cast<std::size_t>(
        std::ceil(probability * static_cast<double>(sorted.size())) - 1.0);
    return sorted[std::min(index, sorted.size() - 1)];
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        throw std::invalid_argument(
            "usage: slope_meta_preflight PACKAGE_DIR [ITERATIONS]");
    }
    const std::filesystem::path package = std::filesystem::absolute(argv[1]);
    const int iterations = argc == 3 ? std::stoi(argv[2]) : 200;
    if (iterations <= 0) throw std::invalid_argument("iterations must be positive");

    Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "slope_meta_preflight");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // Constructor is the contract gate: registry, dimensions, class order,
    // metadata, action scales, offsets and PD gains must all agree.
    r1::slope_meta::SlopeMetaRuntime runtime(package, environment, options);
    const YAML::Node deploy = YAML::LoadFile((package / "params/deploy.yaml").string());
    const YAML::Node meta = deploy["meta_policy"];
    const std::size_t history_steps = meta["history_steps"].as<std::size_t>();
    const double budget_ms =
        1000.0 / meta["selector"]["policy_hz"].as<double>();

    r1::policy::OnnxModel gate(
        environment,
        package / meta["gate_model"].as<std::string>(),
        options);
    std::vector<std::unique_ptr<r1::policy::OnnxModel>> experts;
    for (const auto& expert : meta["experts"]) {
        experts.push_back(std::make_unique<r1::policy::OnnxModel>(
            environment,
            package / expert["model"].as<std::string>(),
            options));
    }

    std::vector<float> observation(gate.InputSize(0), 0.0f);
    observation[5] = -1.0f;

    int release_step = -1;
    float maximum_release_action = 0.0f;
    float maximum_release_leg_target_delta = 0.0f;
    float maximum_step_leg_target_delta = 0.0f;
    std::vector<float> previous_action(24, 0.0f);
    std::string upright_selected_class = "SAFETY_HOLD";
    std::vector<float> upright_probabilities;
    for (int step = 1; step <= 75; ++step) {
        const auto action = runtime.Step(observation);
        for (std::size_t joint = 0; joint < 12; ++joint) {
            maximum_step_leg_target_delta = std::max(
                maximum_step_leg_target_delta,
                std::abs((action[joint] - previous_action[joint])
                         * runtime.CommonContract().scale[joint]));
        }
        previous_action = action;
        if (!runtime.SafetyHold() && release_step < 0) {
            release_step = step;
        }
        if (release_step >= 0) {
            upright_selected_class = runtime.SelectedClass();
            upright_probabilities = runtime.Probabilities();
            for (const float value : action) {
                maximum_release_action = std::max(
                    maximum_release_action, std::abs(value));
            }
            for (std::size_t joint = 0; joint < 12; ++joint) {
                maximum_release_leg_target_delta = std::max(
                    maximum_release_leg_target_delta,
                    std::abs(action[joint] * runtime.CommonContract().scale[joint]));
            }
        }
    }
    runtime.Reset();
    auto run_once = [&]() {
        std::vector<float> hidden(gate.InputSize(1), 0.0f);
        for (std::size_t step = 0; step < history_steps; ++step) {
            auto output = gate.Run({observation, hidden});
            hidden = std::move(output.at(1));
        }
        for (const auto& expert : experts) {
            (void)expert->RunSingle(observation);
        }
    };

    for (int warmup = 0; warmup < 30; ++warmup) run_once();
    std::vector<double> timings_ms;
    timings_ms.reserve(iterations);
    for (int iteration = 0; iteration < iterations; ++iteration) {
        const auto started = std::chrono::steady_clock::now();
        run_once();
        timings_ms.push_back(std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started).count());
    }
    std::sort(timings_ms.begin(), timings_ms.end());
    const double mean = std::accumulate(
        timings_ms.begin(), timings_ms.end(), 0.0) / timings_ms.size();
    const double p99 = Percentile(timings_ms, 0.99);

    std::cout << "slope_meta_preflight=PASS\n"
              << "package=" << package << '\n'
              << "actor_observation=" << runtime.ObservationSize() << '\n'
              << "experts=" << experts.size() << '\n'
              << "history_steps=" << history_steps << '\n'
              << "effective_history_steps=" << runtime.EffectiveHistorySteps() << '\n'
              << "blend_space=" << runtime.BlendSpace() << '\n'
              << "outer_crossfade_enabled="
              << (runtime.OuterCrossfadeEnabled() ? "true" : "false") << '\n'
              << "upright_release_step=" << release_step << '\n'
              << "upright_selected_class=" << upright_selected_class << '\n'
              << "upright_probabilities=";
    for (std::size_t index = 0; index < upright_probabilities.size(); ++index) {
        if (index) std::cout << ',';
        std::cout << upright_probabilities[index];
    }
    std::cout << '\n'
              << "upright_max_abs_raw_action=" << maximum_release_action << '\n'
              << "upright_max_abs_leg_target_delta_rad="
              << maximum_release_leg_target_delta << '\n'
              << "upright_max_step_leg_target_delta_rad="
              << maximum_step_leg_target_delta << '\n'
              << std::fixed << std::setprecision(6)
              << "mean_ms=" << mean << '\n'
              << "p99_ms=" << p99 << '\n'
              << "budget_ms=" << budget_ms << '\n';
    if (p99 >= budget_ms) {
        std::cerr << "slope_meta_preflight=FAIL: p99 exceeds the 50 Hz budget\n";
        return 2;
    }
    return 0;
}
