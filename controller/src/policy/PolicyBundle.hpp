#pragma once

// Locomotion policy bundle: one directory per selectable model.
//
//   policies/locomotion/<name>/policy.onnx
//   policies/locomotion/<name>/policy.yaml   (contract, sha256[, waist_kp, waist_kd])
//
// config/locomotion.yaml selects bundles by relative name, optionally group/name
// (walk_policy / teleop_policy). Flat names remain supported.
// The bundle is the single place that records which contract an artifact uses
// and its SHA-256; the runtime verifies the hash once at startup, outside the
// 500 Hz loop. ONNX metadata is still checked by PolicyController::Init.
// H4 teleop availability follows the contract (SupportsH4TeleopOverlay), not
// a per-bundle flag; a legacy `teleop_h4:` line is accepted and ignored.
//
// Optional `waist_kp` / `waist_kd` (both or neither) replace the ONNX metadata
// gains of waist_roll and waist_yaw on the robot, for policies that hold the
// waist by PD (waist not a policy action). This is a hardware-side deviation
// from the training dynamics; record it in the bundle README.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

#include "TeleopH4PolicyGate.hpp"

struct PolicyBundle {
    std::string name;
    std::string rel_path;       // relative to proj_dir, e.g. policies/locomotion/x/policy.onnx
    std::string contract;
    std::string sha256;
    float waist_kp = NAN;       // NaN = keep the ONNX metadata gain
    float waist_kd = NAN;

    bool has_waist_gains() const { return std::isfinite(waist_kp); }

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
            else if (key == "waist_kp") b.waist_kp = ParseGain(name, key, value, 10.0f, 400.0f);
            else if (key == "waist_kd") b.waist_kd = ParseGain(name, key, value, 0.0f, 20.0f);
        }
        if (std::isfinite(b.waist_kp) != std::isfinite(b.waist_kd))
            throw std::runtime_error("policy bundle '" + name + "': waist_kp and waist_kd go together");
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
    static float ParseGain(const std::string& name, const std::string& key,
                           const std::string& value, float lo, float hi) {
        size_t used = 0;
        float f = NAN;
        try { f = std::stof(value, &used); } catch (...) {}
        if (used != value.size() || !std::isfinite(f) || f < lo || f > hi)
            throw std::runtime_error("policy bundle '" + name + "': " + key + " must be a number in [" +
                                     std::to_string(lo) + ", " + std::to_string(hi) + "], got '" +
                                     value + "'");
        return f;
    }

    static std::string Trim(const std::string& s) {
        const auto first = s.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        const auto last = s.find_last_not_of(" \t\r\n");
        return s.substr(first, last - first + 1);
    }
};
