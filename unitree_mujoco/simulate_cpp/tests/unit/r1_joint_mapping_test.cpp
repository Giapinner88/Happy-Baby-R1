#include "runtime/R1Config.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <set>
#include <string>

namespace {

bool Require(bool condition, const std::string& message) {
    if (!condition) std::cerr << "r1_joint_mapping_test: FAIL: " << message << '\n';
    return condition;
}

}  // namespace

int main() {
    constexpr std::array<int, R1Config::NUM_JOINTS> expected_policy_to_idl = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12, 13,
        15, 16, 17, 18, 19,
        22, 23, 24, 25, 26,
    };

    bool ok = true;
    std::set<int> used_idl_slots;
    for (int policy_index = 0; policy_index < R1Config::NUM_JOINTS; ++policy_index) {
        const int idl_index = R1Config::PolicyToIdl(policy_index);
        ok &= Require(idl_index == expected_policy_to_idl[policy_index],
                      "policy index " + std::to_string(policy_index) +
                      " maps to IDL " + std::to_string(idl_index) +
                      ", expected " + std::to_string(expected_policy_to_idl[policy_index]));
        ok &= Require(used_idl_slots.insert(idl_index).second,
                      "duplicate IDL slot " + std::to_string(idl_index));
    }

    ok &= Require(R1Config::PolicyToIdl(R1Config::WAIST_ROLL_POLICY_INDEX) == 12,
                  "waist_roll must route to IDL 12");
    ok &= Require(R1Config::PolicyToIdl(R1Config::WAIST_YAW_POLICY_INDEX) == 13,
                  "waist_yaw must route to IDL 13");
    ok &= Require(
        R1Config::joint_idx_in_idl[R1Config::HEAD_PITCH_LOGICAL_INDEX] == 29,
        "head_pitch must route to IDL 29");
    ok &= Require(
        R1Config::joint_idx_in_idl[R1Config::HEAD_YAW_LOGICAL_INDEX] == 30,
        "head_yaw must route to IDL 30");

    // End-to-end command and feedback sentinel test through policy/IDL/sim order.
    std::array<double, 35> low_command{};
    for (int policy_index = 0; policy_index < R1Config::NUM_JOINTS; ++policy_index) {
        low_command[R1Config::PolicyToIdl(policy_index)] = 1000.0 + policy_index;
    }
    std::array<double, R1Config::NUM_JOINTS> sim_actuator{};
    for (int sim_index = 0; sim_index < R1Config::NUM_JOINTS; ++sim_index) {
        sim_actuator[sim_index] = low_command[R1JointMap::kSimToIdl[sim_index]];
        ok &= Require(sim_actuator[sim_index] == 1000.0 + sim_index,
                      "command sentinel reached wrong sim actuator " + std::to_string(sim_index));
    }

    std::array<double, 35> low_state{};
    for (int sim_index = 0; sim_index < R1Config::NUM_JOINTS; ++sim_index) {
        low_state[R1JointMap::kSimToIdl[sim_index]] = 2000.0 + sim_index;
    }
    for (int policy_index = 0; policy_index < R1Config::NUM_JOINTS; ++policy_index) {
        const double feedback = low_state[R1Config::PolicyToIdl(policy_index)];
        ok &= Require(feedback == 2000.0 + policy_index,
                      "feedback sentinel reached wrong policy observation " +
                      std::to_string(policy_index));
    }

    if (!ok) return 1;
    std::cout << "r1_joint_mapping_test: PASS (waist roll=12, yaw=13)\n";
    return 0;
}
