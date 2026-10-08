#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "config/RobotSpec.hpp"
#include "policy/OnnxPolicy.hpp"

namespace {

void Require(bool condition) {
    if (!condition) std::abort();
}

}  // namespace

// Regression: the configured Mimic checkpoint must retain the standard
// one-observation contract and produce a finite 24-D action.
// Pinned to doremon_v8 (dance_3 in config/dance.yaml) since 2026-09-05; the
// previous doremon_v7 folder was emptied. Repoint this and HB_DOREMON_MODEL
// together whenever the configured dance model changes.
int main() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_input_contract_test");
    Ort::SessionOptions options;
    OnnxPolicy policy;

    policy.Load(HB_DOREMON_MODEL, env, options, spec::kMimicObsSize);
    std::vector<float> obs(spec::kMimicObsSize, 0.0f);
    const auto action = policy.Infer(obs);
    Require(action.size() == spec::kNumJoints);
    for (float value : action) Require(std::isfinite(value));
    return 0;
}
