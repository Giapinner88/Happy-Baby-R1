#pragma once

// Locomotion policy bundle: one directory per selectable model.
//
//   policies/locomotion/<name>/policy.onnx
//   policies/locomotion/<name>/policy.yaml   (contract, sha256)
//
// config/locomotion.yaml selects bundles by name (walk_policy / teleop_policy).
// The bundle is the single place that records which contract an artifact uses
// and its SHA-256; the runtime verifies the hash once at startup, outside the
// 500 Hz loop. ONNX metadata is still checked by PolicyController::Init.
// H4 teleop availability follows the contract (SupportsH4TeleopOverlay), not
// a per-bundle flag; a legacy `teleop_h4:` line is accepted and ignored.
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <string>

#include "TeleopH4PolicyGate.hpp"

struct PolicyBundle {
    std::string name;
    std::string rel_path;       // relative to proj_dir, e.g. policies/locomotion/x/policy.onnx
    std::string contract;
    std::string sha256;

    static bool ValidName(const std::string& name) {
        return !name.empty() && name != "." && name != ".." &&
               name.find('/') == std::string::npos &&
               name.find('\\') == std::string::npos;
    }

    // Throws on a missing/incomplete bundle or a hash mismatch: a wrong
    // artifact must stop startup, never fall back to another model.
    static PolicyBundle Load(const std::string& proj_dir, const std::string& name) {
        if (!ValidName(name))
            throw std::runtime_error("policy bundle name must be a directory name: '" + name + "'");
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
    static std::string Trim(const std::string& s) {
        const auto first = s.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        const auto last = s.find_last_not_of(" \t\r\n");
        return s.substr(first, last - first + 1);
    }
};
