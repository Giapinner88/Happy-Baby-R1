// Replays a Python v2573_push env trace through MultiGaitV257Runtime (the FSM
// used by MultiGaitV2573Controller) and requires per-tick parity of the 23-D
// conditioning, scheduled command, clock phase, hold/mode/substate and the
// IMU/FK speed estimate. v2574 resumes from v2573_push with the same runtime.
//
// Trace: tests/data/multigait_v2573_push_runtime_trace.txt.gz, produced by
// simulate_cpp/tests/fixtures/multigait_v257/generate_trace.py with
// AUDIT_NS=armfree0917_multigait_run25_v2573_push_4096 (model_27999); see the
// header lines of the trace for the env subset.
// Row: env step request raw[3] timing gyro[3] gravity[3] q[24] dq[24] |
//      cond[23] scheduled[3] phase hold mode substate speed confident.
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../src/gait/MultiGaitV257Runtime.hpp"

int main(int argc, char** argv) {
    if (argc < 2) throw std::runtime_error("usage: multigait_v2573_runtime_parity_test trace.txt.gz");
    std::stringstream buffer;
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(
        popen(("gzip -dc '" + std::string(argv[1]) + "'").c_str(), "r"), pclose);
    if (!pipe) throw std::runtime_error("gzip failed");
    char chunk[1 << 16];
    size_t k;
    while ((k = fread(chunk, 1, sizeof chunk, pipe.get())) > 0) buffer.write(chunk, k);

    MultiGaitV257Runtime::Variant variant;
    variant.v2573 = true;
    variant.anisotropic = true;
    LegFK fk;
    fk.UseNarrowStance190();
    std::map<int, MultiGaitV257Runtime> envs;
    std::set<int> modes;
    std::string line;
    long n = 0;
    float worst = 0, worst_speed = 0;
    auto fail = [&](const std::string& what, float a, float b) {
        throw std::runtime_error("mismatch row=" + std::to_string(n) + " " + what +
                                 " cpp=" + std::to_string(a) + " python=" + std::to_string(b));
    };
    while (std::getline(buffer, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream s(line);
        std::vector<float> x;
        float v;
        while (s >> v) x.push_back(v);
        if (x.size() != 93) throw std::runtime_error("bad row size " + std::to_string(x.size()));
        auto& m = envs[int(x[0])];
        m.gate = MultiGaitV257Runtime::StopGate::StopFix();
        m.variant = variant;
        MultiGaitV257Runtime::Inputs in;
        in.request = int(x[2]);
        for (int i = 0; i < 3; ++i) {
            in.raw[i] = x[3 + i];
            in.gyro[i] = x[7 + i];
            in.gravity[i] = x[10 + i];
        }
        for (int i = 0; i < 24; ++i) {
            in.q[i] = x[13 + i];
            in.dq[i] = x[37 + i];
        }
        m.AdvanceAction(x[6]);
        m.Step(in, fk);
        const float* e = &x[61];
        const auto o = m.Conditioning();
        for (int i = 0; i < 23; ++i) {
            const float d = std::abs(o[i] - e[i]);
            worst = std::max(worst, d);
            if (d > 3e-4f) fail("cond[" + std::to_string(i) + "]", o[i], e[i]);
        }
        for (int i = 0; i < 3; ++i)
            if (std::abs(m.scheduled[i] - e[23 + i]) > 3e-4f) fail("scheduled", m.scheduled[i], e[23 + i]);
        if (std::abs(m.phase - e[26]) > 3e-4f) fail("phase", m.phase, e[26]);
        if (int(m.hold) != int(e[27]) || m.mode != int(e[28]) || m.substate != int(e[29]))
            fail("hold/mode/substate", m.mode * 100 + m.substate * 10 + m.hold,
                 e[28] * 100 + e[29] * 10 + e[27]);
        const float ds = std::abs(m.balance.speed - e[30]);
        worst_speed = std::max(worst_speed, ds);
        if (ds > 2e-3f) fail("balance.speed", m.balance.speed, e[30]);
        if (int(m.balance.confident) != int(e[31])) fail("balance.confident", m.balance.confident, e[31]);
        modes.insert(m.mode);
        ++n;
    }
    if (n < 5000 || modes.size() < 8)
        throw std::runtime_error("insufficient coverage rows=" + std::to_string(n) +
                                 " modes=" + std::to_string(modes.size()));
    std::cout << "MULTIGAIT_V2573_PARITY_OK " << n << " ticks, " << envs.size() << " envs, "
              << modes.size() << " modes, max_cond_abs=" << worst
              << " max_speed_abs=" << worst_speed << "\n";
    return 0;
}
