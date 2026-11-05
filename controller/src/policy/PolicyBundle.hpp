#pragma once

// Locomotion policy bundle: one directory per selectable model.
//
//   policies/locomotion/<name>/policy.onnx
//   policies/locomotion/<name>/policy.yaml   (contract, sha256, gait timing)
//
// config/locomotion.yaml selects bundles by relative name, optionally group/name
// (walk_policy / teleop_policy). Flat names remain supported.
// The bundle is the single place that records which contract an artifact uses
// and its SHA-256; the runtime verifies the hash once at startup, outside the
// 500 Hz loop. ONNX metadata is still checked by PolicyController::Init.
// H4 teleop availability follows the contract (SupportsH4TeleopOverlay), not
// a per-bundle flag; a legacy `teleop_h4:` line is accepted and ignored.
//
// gait_period_s / gait_settle_dwell_s are copies of the ONNX metadata of the
// same name, written here at packaging time so that selecting a bundle is
// enough: locomotion.yaml no longer has to repeat the training period. They are
// a proposal, not the authority. PolicyController::Init still compares them
// against the metadata, so a hand-edited policy.yaml stops startup. Both are
// optional; a bundle without them falls back to the locomotion.yaml value.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

#include "TeleopH4PolicyGate.hpp"

struct PolicyBundle {
    std::string name;
    std::string rel_path;       // relative to proj_dir, e.g. policies/locomotion/x/policy.onnx
    std::string contract;
    std::string sha256;
    std::optional<float> gait_period_s;        // chu kỳ phase obs lúc train
    std::optional<float> gait_settle_dwell_s;  // gait_t_settle_s lúc train
    // "command_gated" hoặc "episode_clock": nguồn của term phase lúc train.
    // Hai bundle cùng gait_fsm_contract có thể khác nhau ở đây, nên FSM không
    // suy ra được; thiếu khoá thì ONNX khuyết metadata phase sẽ lọt.
    std::string gait_phase_source;

    static bool ValidName(const std::string& name) {
        // Same component grammar as hb_valid_policy_bundle_name in the shell
        // reader. Reject absolute paths, empty components and traversal before IO.
        if (name.empty()) return false;
        bool first = true;
        for (const char ch : name) {
            const bool alnum = (ch >= 'a' && ch <= 'z') ||
                               (ch >= 'A' && ch <= 'Z') ||
                               (ch >= '0' && ch <= '9');
            if (ch == '/') {
                if (first) return false;
                first = true;
            } else {
                if (!alnum && (first || (ch != '.' && ch != '_' && ch != '-')))
                    return false;
                first = false;
            }
        }
        return !first;
    }

    // Throws on a missing/incomplete bundle or a hash mismatch: a wrong
    // artifact must stop startup, never fall back to another model.
    static PolicyBundle Load(const std::string& proj_dir, const std::string& name) {
        if (!ValidName(name))
            throw std::runtime_error("policy bundle must be a relative name or group/name: '" + name + "'");
        PolicyBundle b;
        b.name = name;
        const std::string dir_rel = "policies/locomotion/" + name + "/";
        b.rel_path = dir_rel + "policy.onnx";
        const std::string yaml_path = proj_dir + "/" + dir_rel + "policy.yaml";
        std::ifstream file(yaml_path);
        if (!file.is_open())
            throw std::runtime_error("policy bundle '" + name + "' has no " + yaml_path);

        std::string line;
        while (std::getline(file, line)) {
            const size_t hash = line.find('#');
            if (hash != std::string::npos) line = line.substr(0, hash);
            const size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = Trim(line.substr(0, colon));
            std::string value = Trim(line.substr(colon + 1));
            if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
                value = value.substr(1, value.size() - 2);
            if (key == "contract") b.contract = value;
            else if (key == "sha256") b.sha256 = value;
            else if (key == "gait_period_s")
                b.gait_period_s = ParseFloat(name, key, value, 0.3f, 1.5f);
            else if (key == "gait_settle_dwell_s")
                b.gait_settle_dwell_s = ParseFloat(name, key, value, 0.01f, 60.0f);
            else if (key == "gait_phase_source") {
                if (value != "command_gated" && value != "episode_clock")
                    throw std::runtime_error("policy bundle '" + name +
                        "': gait_phase_source must be 'command_gated' or 'episode_clock', got '" +
                        value + "'");
                b.gait_phase_source = value;
            }
        }
        if (b.contract.empty())
            throw std::runtime_error("policy bundle '" + name + "': missing contract");
        if (!teleop_h4::IsLowerSha256(b.sha256))
            throw std::runtime_error("policy bundle '" + name + "': sha256 must be 64 lowercase hex chars");

        const std::string actual = teleop_h4::Sha256File(proj_dir + "/" + b.rel_path);
        if (actual != b.sha256)
            throw std::runtime_error("policy bundle '" + name + "': policy.onnx SHA-256 " +
                                     (actual.empty() ? std::string("unreadable") : actual) +
                                     " differs from policy.yaml " + b.sha256);
        return b;
    }

private:
    // A malformed number stops startup: silently dropping it would fall back to
    // locomotion.yaml and run the policy at the wrong period.
    // Dải giống Tuning::Validate: bundle là đường vào thứ hai cho cùng tham số,
    // nên phải chặn cùng mức. PhaseObs chia cho chu kỳ, số 0 đi thẳng vào obs.
    static float ParseFloat(const std::string& bundle, const std::string& key,
                            const std::string& value, float low, float high) {
        size_t consumed = 0;
        float parsed = 0.0f;
        try {
            parsed = std::stof(value, &consumed);
        } catch (const std::exception&) {
            consumed = 0;
        }
        if (consumed != value.size() || !std::isfinite(parsed))
            throw std::runtime_error("policy bundle '" + bundle + "': " + key +
                                     " is not a finite number: '" + value + "'");
        if (parsed < low || parsed > high)
            throw std::runtime_error("policy bundle '" + bundle + "': " + key + " = " +
                                     value + " is outside [" + std::to_string(low) + ", " +
                                     std::to_string(high) + "]");
        return parsed;
    }

    static std::string Trim(const std::string& s) {
        const auto first = s.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        const auto last = s.find_last_not_of(" \t\r\n");
        return s.substr(first, last - first + 1);
    }
};
