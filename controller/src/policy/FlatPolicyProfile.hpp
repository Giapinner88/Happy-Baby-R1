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
    kWholeBodyLegsBlindV1,   // r1_wb_legs_v1_blind: 71-D obs, 12 leg actions (chỉ route bundle)
    kWholeBodyLegsGaitH4BlindV1,  // r1_wb_legs_gait_h4_v1_blind: 4x71 + one-hot = 287-D, 12 actions
    // r1_armfree0917_multigait_h4_v2: 355-D (332 chuan hoa + 23 conditioning
    // tho), 25 action (24 khop + 1 lenh nhip). CHUA chay duoc: xem
    // MultiGaitH4V2Controller va plans/PHASE_E_V2_CONTRACT_NOTES.md.
    kMultiGaitH4V2,
};

inline FlatPolicyProfile ParseFlatPolicyProfile(const std::string& value) {
    if (value == "legacy_83") return FlatPolicyProfile::kLegacy83;
    if (value == "flat_plus_h4_v1") return FlatPolicyProfile::kFlatPlusH4V1;
    if (value == "flat_plus_h5_v1") return FlatPolicyProfile::kFlatPlusH5V1;
    if (value == "flat_plus_gait_h4_v1") return FlatPolicyProfile::kFlatPlusGaitH4V1;
    if (value == "r1_wb_legs_v1_blind") return FlatPolicyProfile::kWholeBodyLegsBlindV1;
    if (value == "r1_wb_legs_gait_h4_v1_blind")
        return FlatPolicyProfile::kWholeBodyLegsGaitH4BlindV1;
    if (value == "r1_armfree0917_multigait_h4_v2")
        return FlatPolicyProfile::kMultiGaitH4V2;
    throw std::runtime_error("Unknown flat_policy_contract: " + value);
}

// Contract co MOT chu ky gait co dinh luc train, ghi trong policy.yaml cua
// bundle (hoac override trong locomotion.yaml)?
//
// Sai voi r1_armfree0917_multigait_h4_v2: chu ky cua no do chinh policy phat
// ra moi buoc qua action index 24 va chay trong dai 0.35-2.0 s (metadata
// `timing_rule`), nen khong ton tai mot so de ghi. Bat bundle phai khai mot
// so o day la bia dat mot hop dong khong co that; runtime cung khong co
// metadata `gait_period_s` nao de doi chieu lai.
inline bool FlatPolicyProfileHasFixedGaitPeriod(FlatPolicyProfile profile) {
    return profile != FlatPolicyProfile::kMultiGaitH4V2;
}
