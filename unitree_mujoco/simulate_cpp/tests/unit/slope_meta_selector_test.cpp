#include "controllers/slope_meta/SlopeMetaSelector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

int main() {
    const YAML::Node config = YAML::Load(R"(
policy_hz: 50.0
cold_start_s: 0.02
enter_confidence: 0.75
exit_confidence: 0.55
agreement_frames: 2
minimum_dwell_s: 0.02
crossfade_s: 0.04
initial_weights: [0.5, 0.5, 0.0]
)");
    r1::slope_meta::SlopeMetaSelector selector(config, 3);
    const std::vector<float> neutral{0.03f, 0.03f, 0.04f, 0.90f};
    const std::vector<float> up{0.90f, 0.03f, 0.03f, 0.04f};
    const std::vector<float> flat{0.03f, 0.03f, 0.90f, 0.04f};
    const std::vector<float> low{0.26f, 0.25f, 0.25f, 0.24f};

    Require(selector.SafetyHold(), "selector must start in hold");
    selector.Update(neutral);
    Require(selector.SafetyHold(), "cold start hold ended too early");
    selector.Update(neutral);
    Require(!selector.SafetyHold(), "confident neutral should release hold");

    selector.Update(up);
    selector.Update(up);
    Require(selector.Selected() == 0, "up expert was not selected");
    selector.Update(up);
    Require(std::abs(selector.Weights()[0] - 1.0f) < 1.0e-6f,
            "up crossfade did not finish");

    selector.Update(neutral);
    selector.Update(neutral);
    Require(selector.Selected() == selector.NeutralIndex(),
            "neutral was not selected");
    Require(selector.LastExpert() == 0, "last expert was not retained");
    Require(!selector.SafetyHold(), "confident neutral must not be safety hold");

    selector.Update(low);
    Require(selector.SafetyHold(), "low confidence must enter hold");
    selector.Update(flat);
    selector.Update(flat);
    Require(selector.Selected() == 2, "extensible third expert was not selected");

    selector.Update({
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f});
    Require(selector.Selected() == selector.NeutralIndex(),
            "invalid probabilities must reset selection");
    Require(selector.SafetyHold(), "invalid probabilities must hold");

    const YAML::Node neutral_fallback_config = YAML::Load(R"(
policy_hz: 50.0
cold_start_s: 0.02
enter_confidence: 0.75
exit_confidence: 0.55
hold_on_low_confidence: false
agreement_frames: 2
minimum_dwell_s: 0.02
crossfade_s: 0.04
initial_weights: [1.0, 0.0]
)");
    r1::slope_meta::SlopeMetaSelector neutral_fallback(
        neutral_fallback_config, 2);
    neutral_fallback.Update({0.05f, 0.05f, 0.90f});
    neutral_fallback.Update({0.40f, 0.25f, 0.35f});
    Require(!neutral_fallback.SafetyHold(),
            "configured low confidence must use command-preserving NEUTRAL");
    Require(neutral_fallback.Selected() == neutral_fallback.NeutralIndex(),
            "low-confidence fallback must remain NEUTRAL");
    Require(neutral_fallback.Weights()[0] == 1.0f
                && neutral_fallback.Weights()[1] == 0.0f,
            "NEUTRAL fallback must preserve deterministic bootstrap expert");

    const YAML::Node meta_policy_config = YAML::Load(R"(
policy_hz: 50.0
cold_start_s: 0.02
enter_confidence: 0.75
exit_confidence: 0.55
startup_safety_hold: false
hold_on_low_confidence: false
agreement_frames: 2
minimum_dwell_s: 0.02
crossfade_s: 0.04
initial_weights: [0.0, 0.0, 1.0]
)");
    r1::slope_meta::SlopeMetaSelector meta_policy(meta_policy_config, 3);
    Require(!meta_policy.SafetyHold(),
            "meta_policy must actively run Flat immediately after reset");
    Require(meta_policy.Weights()[2] == 1.0f,
            "meta_policy cold-start weights must select Flat");
    meta_policy.Update(neutral);
    Require(!meta_policy.SafetyHold(),
            "finite meta_policy cold start must not enter SAFETY_HOLD");
    meta_policy.EnterSafetyHold();
    Require(meta_policy.SafetyHold(),
            "explicit invalid-signal transition must enter SAFETY_HOLD");

    const YAML::Node no_neutral_config = YAML::Load(R"(
policy_hz: 50.0
cold_start_s: 0.0
enter_confidence: 0.75
exit_confidence: 0.55
startup_safety_hold: false
hold_on_low_confidence: false
allow_neutral: false
agreement_frames: 1
minimum_dwell_s: 0.0
crossfade_s: 0.0
initial_weights: [0.0, 0.0, 1.0]
)");
    r1::slope_meta::SlopeMetaSelector no_neutral(no_neutral_config, 3);
    no_neutral.Update(neutral);
    no_neutral.Update(neutral);
    Require(no_neutral.Selected() == 2 && !no_neutral.SafetyHold(),
            "no-neutral ablation must choose the strongest finite expert");
    return 0;
}
