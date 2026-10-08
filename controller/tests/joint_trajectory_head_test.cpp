#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include <cnpy.h>

#include "../src/motion/JointTrajectory.hpp"

namespace {
void Require(bool condition) {
    if (!condition) std::abort();
}

void ExpectNear(float actual, float expected) {
    Require(std::fabs(actual - expected) < 1e-6f);
}
}  // namespace

int main() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = std::filesystem::temp_directory_path() /
                      ("hb_joint_trajectory_head_" + std::to_string(stamp) + ".npz");

    std::vector<float> joints(2 * spec::kNumJoints, 0.0f);
    std::vector<float> head = {0.10f, -0.20f, 0.30f, -0.40f};
    const double fps = 50.0;
    cnpy::npz_save(path.string(), "joint_pos", joints.data(),
                   {2, static_cast<size_t>(spec::kNumJoints)}, "w");
    cnpy::npz_save(path.string(), "head_pos", head.data(), {2, 2}, "a");
    cnpy::npz_save(path.string(), "fps", &fps, {1}, "a");

    JointTrajectory trajectory;
    trajectory.Load(path.string());
    Require(trajectory.has_head());
    ExpectNear(trajectory.head_frame(0)[0], 0.10f);
    ExpectNear(trajectory.head_frame(0)[1], -0.20f);
    ExpectNear(trajectory.head_frame(1)[0], 0.30f);
    ExpectNear(trajectory.head_frame(1)[1], -0.40f);

    std::filesystem::remove(path);
    return 0;
}
