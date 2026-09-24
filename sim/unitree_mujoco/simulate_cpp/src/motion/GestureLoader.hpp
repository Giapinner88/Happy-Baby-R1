#pragma once

#include <array>
#include <string>
#include <vector>

std::vector<std::array<float, 10>> LoadGestureNpz(
    const std::string& path,
    float& frames_per_second,
    bool& loaded);
