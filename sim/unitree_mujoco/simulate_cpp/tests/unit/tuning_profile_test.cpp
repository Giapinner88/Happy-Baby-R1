#include "runtime/Tuning.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {
int failures = 0;

void Check(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}

bool Near(float actual, float expected) {
    return std::abs(actual - expected) <= 1e-6f;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    Tuning& tuning = Tuning::Get();
    tuning.Load(argv[1]);

    tuning.locomotion_policy = "rough/policy_r1_rough.onnx";
    const LocomotionProfile& rough = tuning.ResolveLocomotionProfile(270);
    Check(rough.name == "Rough", "270-D policy selects Rough profile");
    Check(Near(rough.gait_period_s, 0.6f), "Rough gait period is 0.6 s");
    Check(Near(rough.fast[0], 2.0f) && Near(rough.fast[1], 0.8f) &&
              Near(rough.fast[2], 1.0f),
          "Rough fast limits are per-axis (2.0, 0.8, 1.0)");

    tuning.locomotion_policy = "slope/policy_up_v2.onnx";
    const LocomotionProfile& slope = tuning.ResolveLocomotionProfile(83);
    Check(slope.name == "Slope", "slope folder selects Slope profile");
    Check(Near(slope.gait_period_s, 0.6f), "Slope gait period is 0.6 s");

    tuning.locomotion_policy = "flat/policy_goc.onnx";
    const LocomotionProfile& flat = tuning.ResolveLocomotionProfile(83);
    Check(flat.name == "Flat", "83-D flat policy selects Flat profile");
    Check(Near(flat.gait_period_s, 0.6f), "Flat gait period is 0.6 s");

    // --- resolver fail-closed theo cặp (contract, dim) ---

    // Model legacy KHÔNG có metadata contract vẫn phải chạy, gồm cả baseline
    // common-PD dùng làm đối chứng.
    tuning.locomotion_policy = "flat/policy_goc.onnx";
    Check(Near(tuning.ResolveLocomotionProfile(83, "").gait_period_s, 0.6f),
          "legacy 83-D without metadata still resolves");
    Check(tuning.ResolveLocomotionProfile(270, "").name == "Rough",
          "legacy 270-D without metadata still resolves");

    // Dimension lạ không được âm thầm rơi về Flat như resolver cũ.
    for (int dim : {84, 105, 271, 332, 999}) {
        bool threw = false;
        try {
            (void)tuning.ResolveLocomotionProfile(dim, "");
        } catch (const std::exception&) {
            threw = true;
        }
        Check(threw, "unknown dimension without contract is rejected");
    }

    // Có contract -> phải khớp đúng cặp.
    Check(Near(tuning.ResolveLocomotionProfile(332, "flat_plus_h4_v1").gait_period_s,
               0.6f),
          "flat_plus_h4_v1 + 332 resolves to the 0.6 s Flat profile");
    Check(Near(tuning.ResolveLocomotionProfile(415, "flat_plus_h5_v1").gait_period_s,
               0.6f),
          "flat_plus_h5_v1 + 415 resolves to the 0.6 s Flat profile");
    Check(Near(tuning.ResolveLocomotionProfile(335, "flat_plus_gait_h4_v1").gait_period_s,
               0.6f),
          "flat_plus_gait_h4_v1 + 335 resolves to the 0.6 s Flat profile");
    for (int dim : {83, 105, 270}) {
        bool threw = false;
        try {
            (void)tuning.ResolveLocomotionProfile(dim, "flat_plus_h4_v1");
        } catch (const std::exception&) {
            threw = true;
        }
        Check(threw, "flat_plus_h4_v1 with the wrong dimension is rejected");
    }
    {
        bool threw = false;
        try {
            (void)tuning.ResolveLocomotionProfile(332, "flat_plus_h5_v1");
        } catch (const std::exception&) {
            threw = true;
        }
        Check(threw, "flat_plus_h5_v1 with the wrong dimension is rejected");
    }
    {
        bool threw = false;
        try {
            (void)tuning.ResolveLocomotionProfile(105, "unsupported_flat_contract");
        } catch (const std::exception&) {
            threw = true;
        }
        Check(threw, "unsupported 105-D contract is rejected");
    }

    if (failures != 0) return 1;
    std::cout << "tuning_profile_test: PASS\n";
    return 0;
}
