#pragma once

#include <stdexcept>
#include <string>

// One explicit selector for the supported flat-policy contracts. Keeping
// it typed prevents a history policy from silently falling into legacy 83-D.
enum class FlatPolicyProfile {
    kLegacy83,
    kFlatPlusH4V1,
    kFlatPlusH5V1,
    kFlatPlusGaitH4V1,
};

inline FlatPolicyProfile ParseFlatPolicyProfile(const std::string& value) {
    if (value == "legacy_83") return FlatPolicyProfile::kLegacy83;
    if (value == "flat_plus_h4_v1") return FlatPolicyProfile::kFlatPlusH4V1;
    if (value == "flat_plus_h5_v1") return FlatPolicyProfile::kFlatPlusH5V1;
    if (value == "flat_plus_gait_h4_v1") return FlatPolicyProfile::kFlatPlusGaitH4V1;
    throw std::runtime_error("Unknown flat_policy_contract: " + value);
}
