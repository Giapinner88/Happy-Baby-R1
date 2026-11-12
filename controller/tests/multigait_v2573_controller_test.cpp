// MultiGaitV2573Controller on the shipped bundle: identity gate, 355-D layout,
// raw IMU/joint velocity, FSM command/phase terms (never the HB GaitScheduler),
// timing clock driven by action 24, reset determinism and recovery latch.
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../src/policy/MultiGaitV2573Controller.hpp"

namespace {
void Need(bool ok, const std::string& why) {
    if (!ok) throw std::runtime_error("FAIL: " + why);
}
// H4 term-major (oldest -> newest): newest frame of a term block starts at
// block_start + 3 * dim.
constexpr int kGyroNewest = 0 + 3 * 3;          // 9..11
constexpr int kCommandNewest = 24 + 3 * 3;      // 33..35
constexpr int kPhaseBlock = 36;                 // 36..43
constexpr int kJointVelNewest = 140 + 3 * 24;   // 212..235
constexpr int kActionsNewest = 236 + 3 * 24;    // 308..331
constexpr int kCond = 332;
}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) throw std::invalid_argument("usage: multigait_v2573_controller_test CONTROLLER_ROOT");
    const std::string root = argv[1];
    const std::string model =
        root + "/policies/locomotion/multigait/v2574_armtight_c_iter24900/policy.onnx";
    Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "multigait-v2573-test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    Tuning tuning;

    MultiGaitV2573Controller c;
    c.Init(model, env, options, tuning);
    Need(c.ObsSize() == 355 && c.ActionSize() == 24 && c.OutputSize() == 25, "shape 355 -> 24 + timing");

    RobotState s;
    s.q = c.default_q();
    s.gyro = Eigen::Vector3f(0.5f, 0.5f, 0.5f);       // filtered: must NOT reach obs
    s.gyro_raw = Eigen::Vector3f(0.01f, 0.02f, 0.03f);
    s.dq[0] = 0.9f;                                    // filtered: must NOT reach obs
    s.dq_raw[0] = 0.04f;
    c.Reset(s);
    auto step = [&](float vx, float vy, float yaw) {
        ControlContext ctx{s};
        ctx.cmd_vx = vx;
        ctx.cmd_vy = vy;
        ctx.cmd_yaw = yaw;
        ctx.gait_phase = {0.7f, 0.7f};  // shared HB phase: must be ignored
        const auto& target = c.Step(ctx);
        for (float q : target) Need(std::isfinite(q), "finite target");
        Need(!c.infer_fault(), "no inference fault");
        return c.last_observation();
    };

    // 1. Quiet STAND: canonical W2S_GATHER code, zero phase, raw sensors.
    const std::vector<float> first = step(0, 0, 0);
    Need(first.size() == 355, "obs size");
    Need(c.runtime().mode == MultiGaitV257Runtime::STAND && c.runtime().hold, "starts in STAND hold");
    Need(first[kCond + 0] == 0 && first[kCond + 4] == 1 && first[kCond + 9] == 1 && first[kCond + 15] == 1,
         "quiet conditioning uses the W2S_GATHER code");
    for (int i = 0; i < 8; ++i) Need(first[kPhaseBlock + i] == 0, "phase is zero in hold, HB phase ignored");
    for (int i = 0; i < 3; ++i) Need(first[kGyroNewest + i] == s.gyro_raw[i], "raw gyro in obs");
    Need(first[kJointVelNewest] == s.dq_raw[0], "raw joint velocity in obs");

    // 2. Previous raw joint actions enter history; the timing output does not.
    const auto previous = c.last_action();
    const std::vector<float> second = step(0, 0, 0);
    for (int i = 0; i < 24; ++i) Need(second[kActionsNewest + i] == previous[i], "last joint action history");

    // 3. Reset is deterministic.
    c.Reset(s);
    Need(step(0, 0, 0) == first, "reset deterministic");

    // 4. Walk request: FSM leaves STAND, command term is the accel-limited
    //    schedule, and action 24 advances the policy clock.
    s.gyro_raw = Eigen::Vector3f(0, 0, 0);
    s.dq_raw[0] = 0;
    c.Reset(s);
    std::vector<float> walking = step(0.6f, 0, 0);
    Need(c.runtime().mode == MultiGaitV257Runtime::S2W && !c.runtime().hold, "S2W on walk request");
    Need(walking[kCommandNewest] == c.runtime().scheduled[0] && walking[kCommandNewest] < 0.6f,
         "command term is the FSM schedule, not the raw stick");
    for (int i = 0; i < 50; ++i) walking = step(0.6f, 0, 0);
    Need(c.runtime().cycles > 0.1f, "timing clock advanced by action 24");
    float phase_energy = 0;
    for (int i = 0; i < 8; ++i) phase_energy += std::abs(walking[kPhaseBlock + i]);
    Need(phase_energy > 0.1f, "phase active while stepping");
    Need(!c.StandReady(), "not stand-ready while walking");

    // 5. A tilted stop latches recovery and blocks policy swaps.
    c.Reset(s);
    s.projected_gravity = Eigen::Vector3f(std::sin(0.3f), 0.0f, -std::cos(0.3f));
    std::vector<float> tilted;
    for (int i = 0; i < 100; ++i) tilted = step(0, 0, 0);
    Need(c.runtime().recovery_latched, "recovery latches on a tilted stop");
    Need(!c.StandReady(), "no swap during recovery");
    Need(tilted[kCond + 17] >= 0.5f, "latched recovery context input");

    // 6. Identity gate: another 355/25 multigait export (V2 stub bundle) is refused.
    bool refused = false;
    try {
        MultiGaitV2573Controller other;
        other.Init(root + "/policies/locomotion/multigait_v2_run25_iter8400/policy.onnx", env, options,
                   tuning);
    } catch (const std::exception& e) {
        refused = std::string(e.what()).find("ONNX metadata mismatch") != std::string::npos;
    }
    Need(refused, "multigait V2 export refused by the v2573 identity gate");

    std::cout << "MULTIGAIT_V2573_CONTROLLER_OK identity, layout, raw sensors, FSM command/phase, "
                 "timing clock, reset, recovery\n";
    return 0;
}
