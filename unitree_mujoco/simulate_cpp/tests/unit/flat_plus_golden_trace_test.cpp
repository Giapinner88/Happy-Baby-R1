// Phase 6 -- cross-runtime packing parity.
//
// Reads the trace produced by
//   unitree_rl_mjlab_meta/scripts/make_flat_plus_golden_trace.py
// and checks that this runtime's history bank reproduces the packed 332-D
// vector element by element, including across a mid-trace reset.
//
// HB/high_level_2/tests/flat_plus_golden_trace_test.cpp runs the same file. If
// the two runtimes ever disagree, one of them is packing differently from
// training, and no dimension check would have caught it.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "controllers/locomotion/BaseObservationHistory.hpp"

namespace hist = r1::history;

namespace {

int failures = 0;

void Check(bool condition, const std::string& what) {
    if (!condition) {
        std::cerr << "FAIL: " << what << std::endl;
        ++failures;
    }
}

std::vector<float> ReadValues(std::istringstream& line, int count,
                              const std::string& what) {
    std::vector<float> values;
    values.reserve(static_cast<std::size_t>(count));
    float value = 0.0f;
    while (line >> value) values.push_back(value);
    if (static_cast<int>(values.size()) != count) {
        std::cerr << "FAIL: " << what << " expected " << count << " values, got "
                  << values.size() << std::endl;
        ++failures;
    }
    return values;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: flat_plus_golden_trace_test <trace.txt>\n";
        return 2;
    }
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        std::cerr << "cannot open trace " << argv[1] << std::endl;
        return 2;
    }

    int base_dim = 0, history = 0, actor_dim = 0, steps = 0;
    std::string order, padding, contract;

    hist::BaseObservationHistory bank;
    std::vector<float> base, packed, expected_packed;
    bool have_step = false;
    int checked_steps = 0;

    std::string raw;
    while (std::getline(file, raw)) {
        if (raw.empty() || raw[0] == '#') continue;
        std::istringstream line(raw);
        std::string tag;
        line >> tag;

        if (tag == "contract") { line >> contract; continue; }
        if (tag == "base_dim") { line >> base_dim; continue; }
        if (tag == "history") { line >> history; continue; }
        if (tag == "actor_dim") { line >> actor_dim; continue; }
        if (tag == "order") { line >> order; continue; }
        if (tag == "padding") { line >> padding; continue; }
        if (tag == "steps") { line >> steps; continue; }
        if (tag == "base_schema") { continue; }

        if (tag == "STEP") {
            int index = 0, reset = 0;
            std::string reset_tag;
            line >> index >> reset_tag >> reset;
            if (reset) bank.Reset();
            have_step = true;
            continue;
        }
        if (!have_step) continue;

        if (tag == "BASE") {
            base = ReadValues(line, base_dim, "BASE");
            // Feed the trace's own base frame: this test isolates the history
            // bank and the packer. The 83-D builder itself is covered by
            // flat_plus_history_test / the legacy Flat regression.
            bank.Append(base);
            continue;
        }
        if (tag == "PACKED") {
            expected_packed = ReadValues(line, actor_dim, "PACKED");
            bank.PackTermMajorH4(packed);
            Check(packed.size() == expected_packed.size(), "packed size");
            for (std::size_t i = 0; i < expected_packed.size() && i < packed.size(); ++i) {
                if (std::fabs(packed[i] - expected_packed[i]) > 1e-5f) {
                    std::cerr << "FAIL: element " << i << " expected "
                              << expected_packed[i] << " got " << packed[i]
                              << std::endl;
                    ++failures;
                    break;
                }
            }
            ++checked_steps;
            continue;
        }
        // GYRO/GRAV/CMD/PHASE/Q/DQ/PREV are the raw inputs; the base frame is
        // derived from them by the training-side contract and checked above.
    }

    // The header must agree with this runtime's compiled-in contract, or the
    // trace is describing a different policy than the one we would run.
    Check(contract == hist::kFlatPlusContract, "trace contract matches runtime");
    Check(base_dim == hist::kBaseDim, "trace base_dim matches runtime");
    Check(history == hist::kH4Steps, "trace history length matches runtime");
    Check(actor_dim == hist::kH4ActorDim, "trace actor_dim matches runtime");
    Check(order == hist::kH4Order, "trace order matches runtime");
    Check(padding == hist::kH4Padding, "trace padding matches runtime");
    Check(checked_steps == steps, "every step in the trace was checked");
    Check(steps > 0, "trace is not empty");

    if (failures != 0) {
        std::cerr << "flat_plus_golden_trace_test: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "flat_plus_golden_trace_test: PASS (" << checked_steps
              << " steps)\n";
    return 0;
}
