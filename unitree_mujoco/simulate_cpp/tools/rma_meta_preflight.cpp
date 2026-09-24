#include "controllers/rma_meta/RmaMetaRuntime.hpp"

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

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::cerr << "usage: rma_meta_preflight PACKAGE_DIR [iterations]\n";
        return 2;
    }
    const int iterations = argc == 3 ? std::stoi(argv[2]) : 200;
    if (iterations < 10) {
        std::cerr << "iterations must be at least 10\n";
        return 2;
    }

    try {
        Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "rma_meta_preflight");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        r1::rma_meta::RmaMetaRuntime runtime(
            std::filesystem::path(argv[1]), environment, options);
        std::vector<float> observation(runtime.ObservationSize(), 0.0f);
        observation[5] = -1.0f;
        for (int step = 0; step < 60; ++step) runtime.Step(observation);

        std::vector<double> durations;
        durations.reserve(static_cast<std::size_t>(iterations));
        std::vector<float> action;
        for (int step = 0; step < iterations; ++step) {
            const auto started = std::chrono::steady_clock::now();
            action = runtime.Step(observation);
            durations.push_back(std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count());
            if (action.size() != 24
                || !std::all_of(action.begin(), action.end(), [](float value) {
                    return std::isfinite(value);
                })) {
                throw std::runtime_error("RMA-meta produced an invalid action");
            }
        }
        if (runtime.SafetyHold() || runtime.FaultLatched()) {
            throw std::runtime_error("RMA-meta held or latched on finite input");
        }
        std::sort(durations.begin(), durations.end());
        const double mean = std::accumulate(
            durations.begin(), durations.end(), 0.0) / durations.size();
        const std::size_t p99_index = std::min(
            durations.size() - 1,
            static_cast<std::size_t>(std::ceil(0.99 * durations.size())) - 1);
        const double p99 = durations[p99_index];
        const double maximum_action = std::accumulate(
            action.begin(), action.end(), 0.0,
            [](double current, float value) {
                return std::max(current, static_cast<double>(std::abs(value)));
            });

        std::cout << std::fixed << std::setprecision(6)
                  << "rma_meta_preflight=" << (p99 <= 20.0 ? "PASS" : "FAIL") << '\n'
                  << "package=\"" << std::filesystem::absolute(argv[1]).string()
                  << "\"\n"
                  << "actor_observation=" << runtime.ObservationSize() << '\n'
                  << "experts=" << runtime.ExpertNames().size() << '\n'
                  << "history_steps=" << runtime.HistorySteps() << '\n'
                  << "latent_dim=" << runtime.Latent().size() << '\n'
                  << "meta_decimation=" << runtime.MetaDecimation() << '\n'
                  << "selected_class=" << runtime.SelectedClass() << '\n'
                  << "probabilities=";
        const auto& probabilities = runtime.Probabilities();
        for (std::size_t index = 0; index < probabilities.size(); ++index) {
            if (index) std::cout << ',';
            std::cout << probabilities[index];
        }
        std::cout << '\n'
                  << "max_abs_raw_action=" << maximum_action << '\n'
                  << "mean_ms=" << mean << '\n'
                  << "p99_ms=" << p99 << '\n'
                  << "budget_ms=20.000000\n";
        return p99 <= 20.0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "rma_meta_preflight=FAIL\nreason=" << error.what() << '\n';
        return 1;
    }
}
