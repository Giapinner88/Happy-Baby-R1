#include <cassert>
#include <cstdlib>
#include <stdexcept>

#include "policy/FlatPolicyProfile.hpp"

namespace {
void Require(bool condition) {
    if (!condition) std::abort();
}
}  // namespace

// Standard assert disappears under NDEBUG; CTest also runs in Release.
#undef assert
#define assert(condition) Require(static_cast<bool>(condition))

int main() {
    assert(ParseFlatPolicyProfile("legacy_83") == FlatPolicyProfile::kLegacy83);
    assert(ParseFlatPolicyProfile("flat_plus_h4_v1") ==
           FlatPolicyProfile::kFlatPlusH4V1);
    assert(ParseFlatPolicyProfile("flat_plus_h5_v1") ==
           FlatPolicyProfile::kFlatPlusH5V1);
    assert(ParseFlatPolicyProfile("flat_plus_gait_h4_v1") ==
           FlatPolicyProfile::kFlatPlusGaitH4V1);

    // Contract không thuộc hai route hỗ trợ và tên gần đúng phải bị từ chối.
    for (const char* bad : {"unsupported_flat_contract", "flat_plus",
                            "flat_plus_h4", "flat_plus_h5", "332"}) {
        bool bad_rejected = false;
        try {
            (void)ParseFlatPolicyProfile(bad);
        } catch (const std::runtime_error&) {
            bad_rejected = true;
        }
        assert(bad_rejected);
    }
    return 0;
}
