// Cross-implementation parity for contracts r1_wb_legs_gait_h4_v1_blind (U4, 12
// actions), r1_wb_legs_waist_gait_h4_v1_blind (U7, 14 actions incl. waist) and
// r1_wb_legs_armres_gait_h4_v1_ref (U10, 22 actions: legs + arm residuals around a
// filtered arm reference, plus the arm authority g), selected by
// HB_WB_LEGS_GAIT_ACTION_DIM.
//
// Reads the trace produced by scripts/make_wb_legs_gait_golden.py from the loco
// FSM (loco/r1/gait_fsm.py), an independent Isaac Lab history buffer and
// onnxruntime, and checks that WholeBodyLegsGaitController reaches the same
// STAND/WALK/W2S mode, builds the same observation (4 x frame term-major +
// one-hot: 287 or 295), gets the same raw action and emits the same 24 joint
// targets (waist from the policy for U7), over 130 consecutive policy steps.
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../src/policy/WholeBodyLegsGaitController.hpp"

namespace {

int failures = 0;

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
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "wb_legs_gait_golden"};
    Ort::SessionOptions opts;
    Tuning tuning;
    constexpr int kActionDim = HB_WB_LEGS_GAIT_ACTION_DIM;
    WholeBodyLegsGaitController ctrl(kActionDim);
    ctrl.Init(HB_WB_LEGS_GAIT_MODEL, env, opts, tuning);
    const int obs_dim = ctrl.ObsSize();
    const int arm_ref = kActionDim == 22 ? 20 : 0;  // arm_q_ref[10], arm_dq_ref[10] per frame
    const int authority = kActionDim == 22 ? 1 : 0;
    if (obs_dim != 4 * (59 + kActionDim + arm_ref) + 3 + authority ||
        ctrl.ActionSize() != kActionDim) {
        std::cerr << "FAIL obs/action size " << obs_dim << "/" << ctrl.ActionSize() << "\n";
        return 1;
    }
    RobotState rs;
    ctrl.Reset(rs);

    std::ifstream in(HB_WB_LEGS_GAIT_TRACE);
    if (!in) { std::cerr << "missing trace\n"; return 1; }
    std::string line;
    std::map<std::string, std::vector<float>> row, first;
    int steps = 0;
    auto run_step = [&]() {
        if (row.empty()) return;
        for (int i = 0; i < spec::kNumJoints; ++i) {
            rs.q[i] = row["q"][i];
            rs.dq[i] = rs.dq_raw[i] = row["dq"][i];
        }
        rs.gyro = rs.gyro_raw = {row["gyro"][0], row["gyro"][1], row["gyro"][2]};
        rs.projected_gravity = {row["gravity"][0], row["gravity"][1], row["gravity"][2]};
        ControlContext ctx{rs, row["command"][0], row["command"][1], row["command"][2],
                           {row["phase"][0], row["phase"][1]}};
        // Contract blind: overlay tay không được che obs.
        ctx.arm_override_active = true;
        ctx.arm_mask_keep = 0.0f;
        if (row.count("arm_desired")) {
            std::copy(row["arm_desired"].begin(), row["arm_desired"].end(), ctx.arm_desired.begin());
            ctx.arm_desired_valid = true;
            ctx.arm_authority = row["authority"][0];
        }
        const std::string at = "step " + std::to_string(steps) + " ";
        const auto& target = ctrl.Step(ctx);
        if (static_cast<int>(ctrl.gait().mode()) != static_cast<int>(row["mode"][0])) {
            std::cerr << "FAIL " << at << "mode want " << row["mode"][0] << " got "
                      << static_cast<int>(ctrl.gait().mode()) << "\n";
            ++failures;
        }
        // Block actions (4 x A, cuối history) mang action ONNX của các bước
        // trước, nên dùng dung sai onnxruntime như `action`; phần còn lại phải gần exact.
        constexpr int kActionsBlock = 4 * 59;
        const float* obs = ctrl.last_obs().data();
        const std::vector<float>& want = row["obs"];
        if (static_cast<int>(want.size()) != obs_dim) {
            std::cerr << "FAIL " << at << "trace obs size " << want.size() << "\n";
            ++failures;
            return;
        }
        Near(want, obs, kActionsBlock, 1e-6f, at + "obs");
        Near({want.begin() + kActionsBlock, want.end()}, obs + kActionsBlock,
             4 * kActionDim, 1e-4f, at + "obs.actions");
        // Tham chiếu tay: filter float32 (torch) với float C++, sai khác vài ulp tích luỹ.
        const int ref_at = kActionsBlock + 4 * kActionDim;
        Near({want.begin() + ref_at, want.end()}, obs + ref_at, 4 * arm_ref, 1e-5f,
             at + "obs.arm_ref");
        const int one_hot_at = obs_dim - 3 - authority;
        Near({want.begin() + one_hot_at, want.end()}, obs + one_hot_at, 3 + authority, 0.0f,
             at + "obs.one_hot+authority");
        Near(row["action"], ctrl.last_action().data(), kActionDim, 1e-4f, at + "action");
        Near(row["target"], target.data(), spec::kNumJoints, 1e-4f, at + "target");
        if (steps == 0) first = row;
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
    if (steps != 130) { std::cerr << "FAIL expected 130 steps, got " << steps << "\n"; ++failures; }

    // Reset phải xoá history, last_action và FSM: replay bước 0 phải ra đúng bước 0.
    ctrl.Reset(rs);
    row = first;
    steps = 0;
    run_step();
    if (failures) return 1;
    std::cout << "wb_legs_gait_golden_test: PASS " << ctrl.Name() << " obs " << obs_dim
              << " (130 steps + reset replay)\n";
    return 0;
}
