#include "controllers/arma/ArmaRuntime.hpp"

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: arma_preflight PACKAGE_DIR\n";
        return 2;
    }
    try {
        Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "arma_preflight");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        r1::arma::ArmaRuntime runtime(argv[1], environment, options);
        std::cout << "arma_preflight: PASS bundle=" << runtime.BundleContract()
                  << " history=" << runtime.HistorySteps()
                  << " latent=" << runtime.LatentSize()
                  << " actor_input=" << runtime.ActorInputSize() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "arma_preflight: FAIL " << error.what() << '\n';
        return 1;
    }
}
