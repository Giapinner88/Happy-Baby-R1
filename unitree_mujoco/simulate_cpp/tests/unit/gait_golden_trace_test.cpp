#include "controllers/locomotion/GaitModeScheduler.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void Check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

bool Close(float actual, float expected, float tolerance = 5.0e-5f) {
    return std::abs(actual - expected) <= tolerance;
}

template <typename Container>
void CheckVector(const Container& actual, const Container& expected,
                const std::string& what, float tolerance = 5.0e-5f) {
    for (std::size_t index = 0; index < actual.size(); ++index) {
        if (!Close(actual[index], expected[index], tolerance)) {
            throw std::runtime_error(
                what + " mismatch at index " + std::to_string(index)
                + ": actual=" + std::to_string(actual[index])
                + " expected=" + std::to_string(expected[index]));
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) throw std::invalid_argument("usage: gait_golden_trace_test TRACE");
    try {
        std::ifstream input(argv[1]);
        if (!input.is_open()) throw std::runtime_error("cannot open golden trace");

        GaitModeScheduler scheduler;
        scheduler.Configure({0.10f, 0.10f, 0.15f, 0.15f,
                             0.25f, 0.60f, 1.50f, 5.00f, 0.20f});

        int steps = 0;
        int current_step = -1;
        bool reset = false;
        std::array<float, 3> command{};
        std::array<float, 3> gyro{};
        std::array<float, GaitModeScheduler::kNumJoints> joint_velocity{};
        std::array<float, 3> expected_filtered_gyro{};
        std::array<float, GaitModeScheduler::kNumJoints> expected_filtered_joint{};
        int expected_mode = -1;
        float expected_settle = 0.0f;
        float expected_stop_elapsed = 0.0f;
        float expected_w2s = 0.0f;
        std::array<float, 3> expected_one_hot{};

        std::string line;
        while (std::getline(input, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream row(line);
            std::string tag;
            row >> tag;
            if (tag == "STEP") {
                row >> current_step >> tag;
                int reset_flag = 0;
                row >> reset_flag;
                reset = reset_flag != 0;
            } else if (tag == "CMD") {
                row >> command[0] >> command[1] >> command[2];
            } else if (tag == "W") {
                row >> gyro[0] >> gyro[1] >> gyro[2];
            } else if (tag == "DQ") {
                for (float& value : joint_velocity) row >> value;
            } else if (tag == "FILT_W") {
                for (float& value : expected_filtered_gyro) row >> value;
            } else if (tag == "FILT_DQ") {
                for (float& value : expected_filtered_joint) row >> value;
            } else if (tag == "FLAGS") {
                // Mode/state/filter checks below are the discriminating parts;
                // the trace's stop/stable flags are implied by those states.
                int ignored = 0;
                row >> ignored >> ignored >> ignored >> ignored;
            } else if (tag == "STATE") {
                row >> expected_mode >> expected_settle
                    >> expected_stop_elapsed >> expected_w2s;
            } else if (tag == "GAIT") {
                row >> expected_one_hot[0] >> expected_one_hot[1]
                    >> expected_one_hot[2];
                if (reset) scheduler.Reset();
                scheduler.Update(
                    std::hypot(command[0], command[1]), std::abs(command[2]),
                    gyro, joint_velocity, 0.02f);
                Check(current_step == steps,
                      "trace step index mismatch at " + std::to_string(steps));
                Check(static_cast<int>(scheduler.mode()) == expected_mode,
                      "mode mismatch at step " + std::to_string(current_step));
                Check(Close(scheduler.settle_time_s(), expected_settle),
                      "settle timer mismatch at step " + std::to_string(current_step));
                Check(Close(scheduler.stop_elapsed_s(), expected_stop_elapsed),
                      "stop timer mismatch at step " + std::to_string(current_step));
                Check(Close(scheduler.w2s_time_s(), expected_w2s),
                      "W2S timer mismatch at step " + std::to_string(current_step));
                CheckVector(scheduler.filtered_gyro(), expected_filtered_gyro,
                            "filtered gyro at step " + std::to_string(current_step));
                CheckVector(scheduler.filtered_joint_velocity(), expected_filtered_joint,
                            "filtered joint velocity at step " + std::to_string(current_step));
                CheckVector(scheduler.OneHot(), expected_one_hot,
                            "gait one-hot at step " + std::to_string(current_step));
                ++steps;
            }
        }
        Check(steps == 640, "expected 640 golden-trace steps, got " + std::to_string(steps));
        std::cout << "gait_golden_trace_test: PASS steps=" << steps << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "gait_golden_trace_test: FAIL " << error.what() << '\n';
        return 1;
    }
}
