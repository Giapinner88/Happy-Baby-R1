// Cross-implementation parity for contract r1_wb_legs_v1_blind.
//
// Reads tests/data/wb_legs_golden_trace.txt, produced by
// scripts/make_wb_legs_golden.py from the Isaac Lab term definitions of the
// training env plus onnxruntime, and checks that WholeBodyLegsController builds
// the same 71-D observation (carrying its own 12-D last action between steps),
// gets the same raw action and emits the same 24 joint targets (legs from the
// policy, waist and arms held at default_joint_pos).
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../src/policy/WholeBodyLegsController.hpp"

namespace {

int failures = 0;

struct Probe : WholeBodyLegsController {
    void Obs(const ControlContext& ctx, std::vector<float>& obs) { BuildObservation(ctx, obs); }
};

void Near(const std::vector<float>& want, const float* got, int n, float tol,
          const std::string& what) {
    for (int i = 0; i < n; ++i) {
        if (!(std::abs(want[i] - got[i]) <= tol)) {
            std::cerr << "FAIL " << what << "[" << i << "] want " << want[i] << " got " << got[i]
                      << "\n";
            ++failures;
            return;
        }
    }
}

}  // namespace

int main() {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "wb_legs_golden"};
    Ort::SessionOptions opts;
    Tuning tuning;
    Probe ctrl;
    ctrl.Init(HB_WB_LEGS_MODEL, env, opts, tuning);
    RobotState rs;
    ctrl.Reset(rs);

    std::ifstream in(HB_WB_LEGS_TRACE);
    if (!in) { std::cerr << "missing trace\n"; return 1; }
    std::string line;
    std::map<std::string, std::vector<float>> row;
    int steps = 0;
    auto run_step = [&]() {
        if (row.empty()) return;
        for (int i = 0; i < spec::kNumJoints; ++i) { rs.q[i] = row["q"][i]; rs.dq[i] = row["dq"][i]; }
        rs.gyro = {row["gyro"][0], row["gyro"][1], row["gyro"][2]};
        rs.projected_gravity = {row["gravity"][0], row["gravity"][1], row["gravity"][2]};
        ControlContext ctx{rs, row["command"][0], row["command"][1], row["command"][2],
                           {row["phase"][0], row["phase"][1]}};
        // Overlay tay không được che obs với contract blind.
        ctx.arm_override_active = true;
        ctx.arm_mask_keep = 0.0f;
        std::vector<float> obs(WholeBodyLegsController::kObsDim, 0.0f);
        ctrl.Obs(ctx, obs);
        Near(row["obs"], obs.data(), WholeBodyLegsController::kObsDim, 1e-6f,
             "step " + std::to_string(steps) + " obs");
        const auto& target = ctrl.Step(ctx);
        Near(row["action"], ctrl.last_action().data(), WholeBodyLegsController::kActionDim, 1e-4f,
             "step " + std::to_string(steps) + " action");
        Near(row["target"], target.data(), spec::kNumJoints, 1e-4f,
             "step " + std::to_string(steps) + " target");
        for (int i = WholeBodyLegsController::kActionDim; i < spec::kNumJoints; ++i)
            if (ctrl.last_action()[i] != 0.0f) { std::cerr << "FAIL last_action tail\n"; ++failures; }
        ++steps;
        row.clear();
    };
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tag;
        ss >> tag;
        if (tag == "step") { run_step(); continue; }
        std::vector<float> values;
        float v;
        while (ss >> v) values.push_back(v);
        row[tag] = values;
    }
    run_step();
    if (steps != 8) { std::cerr << "FAIL expected 8 steps, got " << steps << "\n"; ++failures; }
    if (failures) return 1;
    std::cout << "wb_legs_golden_test: PASS (" << steps << " steps)\n";
    return 0;
}
