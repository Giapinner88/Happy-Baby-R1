// Drive the real controller/ONNX/history path against native Python phase.
#include "../src/policy/FlatPlusGaitController.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
    if (argc != 3) throw std::invalid_argument("usage: armfree_native_phase_inference_test ROOT TRACE");
    const std::array<std::string, 4> bundles = {
        "armfree/nomirror_d20",
        "armfree/nomirror_d25",
        "armfree/speed20_oldaction_d20",
        "armfree/speed20_nominal_d20"};
    Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "native-phase-test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    for (const auto& bundle : bundles) {
        FlatPlusGaitController policy;
        GaitModeConfig gait;gait.settle_dwell_s = .5f;
        policy.ConfigureGait(gait, .6f);
        Tuning tuning;
        policy.Init(std::string(argv[1]) + "/policies/locomotion/" + bundle + "/policy.onnx", environment, options, tuning);
        RobotState state;
        state.q = policy.default_q();
        policy.Reset(state);
        std::ifstream input(argv[2]);
        if (!input) throw std::runtime_error("cannot open phase trace");
        std::string line;
        int rows = 0, stand = 0, w2s = 0;
        float maximum = 0;
        while (std::getline(input, line)) {
            if (line.empty() || line[0] == '#') continue;
            int reset;
            float vx, vy, yaw, sine, cosine;
            std::istringstream row(line);
            if (!(row >> reset >> vx >> vy >> yaw >> sine >> cosine))
                throw std::runtime_error("bad native phase row");
            if (reset) policy.Reset(state);
            // Moving arms must not prevent the legs12 FSM from settling.
            for (int j = 12; j < 24; ++j) state.dq_raw[j] = 2.0f;
            ControlContext ctx{state};ctx.cmd_vx = vx;ctx.cmd_vy = vy;ctx.cmd_yaw = yaw;
            const auto& target = policy.Step(ctx);
            for (float q : target)
                if (!std::isfinite(q)) throw std::runtime_error("nonfinite native policy target");
            std::vector<float> observation;
            policy.history().PackTermMajor(observation);
            // H4 term-major: gyro12, gravity12, command12, phase8.
            const float error = std::max(std::abs(observation.at(42) - sine),
                                         std::abs(observation.at(43) - cosine));
            if (error > 5e-6f) throw std::runtime_error("controller native phase mismatch at row " + std::to_string(rows));
            maximum = std::max(maximum, error);
            stand += policy.gait().mode() == GaitMode::Stand;
            w2s += policy.gait().mode() == GaitMode::W2S;
            ++rows;
        }
        if (rows < 1000 || stand == 0 || w2s == 0)
            throw std::runtime_error("native inference trace did not cover stop and restart");
        std::cout << bundle << " NATIVE_INFERENCE_OK rows=" << rows
                  << " phase_max_abs_error=" << maximum << " stand=" << stand << " w2s=" << w2s << '\n';
    }
}
