#include "controllers/slope_meta/SlopeMetaCommandGovernor.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool Near(float left, float right) {
    return std::abs(left - right) < 1.0e-6f;
}

}  // namespace

int main() {
    r1::slope_meta::SlopeMetaCommandGovernor governor;
    governor.Configure({{
        {-0.5f, 1.0f, 0.5f},
        {-0.5f, 0.5f, 0.5f},
        {-1.0f, 1.0f, 1.0f},
    }});

    auto command = governor.Update({1.5f, -0.8f, 2.0f}, false, 0.02f);
    Require(command == std::array<float, 3>{0.0f, 0.0f, 0.0f},
            "SAFETY_HOLD must force command to zero");

    command = governor.Update({1.5f, -0.8f, 2.0f}, true, 0.02f);
    Require(Near(command[0], 0.01f) && Near(command[1], -0.01f)
                && Near(command[2], 0.02f),
            "command release did not use configured slew rates");

    for (int step = 0; step < 200; ++step) {
        command = governor.Update({1.5f, -0.8f, 2.0f}, true, 0.02f);
    }
    Require(Near(command[0], 1.0f) && Near(command[1], -0.5f)
                && Near(command[2], 1.0f),
            "commands were not clamped to expert training ranges");

    command = governor.Update({1.0f, 0.5f, 1.0f}, false, 0.02f);
    Require(command == std::array<float, 3>{0.0f, 0.0f, 0.0f},
            "re-entering SAFETY_HOLD must clear accumulated commands");
    return 0;
}
