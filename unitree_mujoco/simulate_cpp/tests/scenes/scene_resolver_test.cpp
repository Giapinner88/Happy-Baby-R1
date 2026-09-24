#include "simulator/SimulatorSceneResolver.hpp"

#include <iostream>

namespace {
int failures = 0;

void Check(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}
}  // namespace

int main() {
    using SimulatorSceneResolver::ParseArgs;
    using SimulatorSceneResolver::ResolveScenePath;

    const auto short_args = ParseArgs(
        {"unitree_mujoco", "-r", "r1", "-n", "lo", "-s", "scene_slope_15.xml"});
    Check(short_args.robot == "r1", "parse short robot option");
    Check(short_args.scene == "scene_slope_15.xml", "parse short scene option");
    Check(short_args.scene_from_command_line, "mark short scene as explicit");

    const auto long_args = ParseArgs(
        {"unitree_mujoco", "--robot=r1", "--scene=/tmp/custom.xml"});
    Check(long_args.robot == "r1", "parse long robot option");
    Check(long_args.scene == "/tmp/custom.xml", "parse long scene option");
    Check(long_args.scene_from_command_line, "mark long scene as explicit");

    const auto defaults = ParseArgs({"unitree_mujoco"}, "r1", "scene.xml");
    Check(defaults.robot == "r1", "use default robot");
    Check(defaults.scene == "scene.xml", "use default scene");
    Check(!defaults.scene_from_command_line, "mark config scene as default");

    const auto resolved = ResolveScenePath(
        "/workspace/unitree_mujoco/simulate/build/unitree_mujoco",
        "r1", "scene_slope_30.xml");
    Check(resolved ==
              "/workspace/unitree_mujoco/unitree_robots/r1/scene_slope_30.xml",
          "resolve relative scene like simulator");

    const auto explicit_relative = ResolveScenePath(
        "/workspace/unitree_mujoco/simulate/build/unitree_mujoco",
        "r1", "../unitree_robots/r1/scene.xml", true,
        "/workspace/unitree_mujoco/simulate_cpp");
    Check(explicit_relative ==
              "/workspace/unitree_mujoco/unitree_robots/r1/scene.xml",
          "resolve explicit relative scene from simulator cwd");

    if (failures != 0) return 1;
    std::cout << "scene_resolver_test: PASS\n";
    return 0;
}
