#pragma once

#include <stdexcept>
#include <string>

// One explicit selector for mutually-exclusive flat-policy contracts. Keeping
// it typed prevents a V10/V11/legacy path from silently falling into another.
enum class FlatPolicyProfile {
    kLegacy83,
    kUnifiedV10,
    kReservedV11Teleop,
};

inline FlatPolicyProfile ParseFlatPolicyProfile(const std::string& value) {
    if (value == "legacy_83") return FlatPolicyProfile::kLegacy83;
    if (value == "r1_unified_v10") return FlatPolicyProfile::kUnifiedV10;
    if (value == "r1_unified_v11_teleop") return FlatPolicyProfile::kReservedV11Teleop;
    throw std::runtime_error("Unknown flat_policy_contract: " + value);
}
