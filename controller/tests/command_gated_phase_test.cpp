#include "../src/gait/CommandGatedPhase.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    if (argc != 2) throw std::invalid_argument("usage: command_gated_phase_test TRACE");
    std::ifstream input(argv[1]);
    if (!input) throw std::runtime_error("cannot open native Torch phase trace");
    CommandGatedPhase phase;
    std::string line;
    int rows = 0;
    float maximum = 0;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;
        int reset;
        float vx, vy, yaw, expected_sin, expected_cos;
        std::istringstream row(line);
        if (!(row >> reset >> vx >> vy >> yaw >> expected_sin >> expected_cos))
            throw std::runtime_error("invalid native phase row");
        if (reset) phase.Reset();
        const auto actual = phase.Next(vx, vy, yaw);
        const float error = std::max(std::abs(actual[0] - expected_sin),
                                     std::abs(actual[1] - expected_cos));
        if (!std::isfinite(error) || error > 5e-6f)
            throw std::runtime_error("native phase mismatch at row " + std::to_string(rows)
                + " actual=" + std::to_string(actual[0]) + "," + std::to_string(actual[1])
                + " expected=" + std::to_string(expected_sin) + "," + std::to_string(expected_cos));
        maximum = std::max(maximum, error);
        ++rows;
    }
    if (rows < 1000) throw std::runtime_error("native phase trace is incomplete");
    std::cout << "COMMAND_GATED_PHASE_OK rows=" << rows << " max_abs_error=" << maximum << '\n';
}
