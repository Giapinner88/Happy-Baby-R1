#pragma once
// Helpers shared by the Application translation units (src/app/Application*.cpp).
// Not part of the Application interface; include only from those files.

#include <chrono>
#include <cmath>
#include <cstdint>

#include "../config/RobotSpec.hpp"
#include "../estimation/StateEstimator.hpp"

namespace app_internal {

inline int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

// max|dq| trên toàn bộ khớp + chỉ số khớp nhanh nhất (cho lớp an toàn tốc độ khớp).
inline float MaxJointSpeed(const RobotState& rs, int& worst_joint) {
    float m = 0.0f;
    worst_joint = -1;
    for (int i = 0; i < spec::kNumJoints; ++i) {
        float a = std::fabs(rs.dq[i]);
        if (a > m) { m = a; worst_joint = i; }
    }
    return m;
}

}  // namespace app_internal
