#pragma once

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace SimulatorSceneResolver {

namespace fs = std::filesystem;

struct ParsedArgs {
    std::string robot;
    fs::path scene;
    bool scene_from_command_line = false;
};

struct Result {
    bool ok = false;
    fs::path scene;
    std::string message;
};

inline std::string Trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    value = value.substr(first, last - first + 1);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
        value = value.substr(1, value.size() - 2);
    }
    return value;
}

inline std::optional<std::string> ReadSimpleYamlValue(
    const fs::path& path, const std::string& wanted_key) {
    std::ifstream file(path);
    if (!file.is_open()) return std::nullopt;

    std::string line;
    while (std::getline(file, line)) {
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        if (Trim(line.substr(0, colon)) != wanted_key) continue;
        const std::string value = Trim(line.substr(colon + 1));
        if (!value.empty()) return value;
    }
    return std::nullopt;
}

inline ParsedArgs ParseArgs(const std::vector<std::string>& args,
                            std::string default_robot = {},
                            fs::path default_scene = {}) {
    ParsedArgs parsed{std::move(default_robot), std::move(default_scene), false};
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if ((arg == "-r" || arg == "--robot") && i + 1 < args.size()) {
            parsed.robot = args[++i];
        } else if (arg.rfind("--robot=", 0) == 0) {
            parsed.robot = arg.substr(8);
        } else if ((arg == "-s" || arg == "--scene") && i + 1 < args.size()) {
            parsed.scene = args[++i];
            parsed.scene_from_command_line = true;
        } else if (arg.rfind("--scene=", 0) == 0) {
            parsed.scene = arg.substr(8);
            parsed.scene_from_command_line = true;
        }
    }
    return parsed;
}

inline fs::path ResolveScenePath(const fs::path& simulator_executable,
                                 const std::string& robot,
                                 const fs::path& scene,
                                 bool scene_from_command_line = false,
                                 const fs::path& simulator_cwd = {}) {
    if (scene.is_absolute()) return scene.lexically_normal();
    // MuJoCo resolves an explicit relative -s/--scene against the simulator
    // process working directory.  Config's default robot_scene, by contrast,
    // is a filename relative to unitree_robots/<robot>.
    if (scene_from_command_line && !simulator_cwd.empty()) {
        return (simulator_cwd / scene).lexically_normal();
    }
    // .../simulate/build/unitree_mujoco -> .../unitree_mujoco
    const fs::path unitree_mujoco_root = simulator_executable.parent_path()
                                                 .parent_path()
                                                 .parent_path();
    return (unitree_mujoco_root / "unitree_robots" / robot / scene).lexically_normal();
}

inline std::vector<std::string> ReadProcessArgs(const fs::path& cmdline_path) {
    std::ifstream file(cmdline_path, std::ios::binary);
    std::vector<std::string> args;
    if (!file.is_open()) return args;
    std::string arg;
    while (std::getline(file, arg, '\0')) {
        if (!arg.empty()) args.push_back(arg);
    }
    return args;
}

inline bool IsNumeric(const std::string& value) {
    return !value.empty() && std::all_of(value.begin(), value.end(),
        [](unsigned char c) { return std::isdigit(c) != 0; });
}

inline Result DetectActiveR1Scene(const fs::path& proc_root = "/proc") {
    struct Match {
        std::string pid;
        fs::path scene;
    };
    std::vector<Match> matches;

    std::error_code iter_ec;
    for (const auto& entry : fs::directory_iterator(proc_root, iter_ec)) {
        if (iter_ec) break;
        const std::string pid = entry.path().filename().string();
        if (!IsNumeric(pid)) continue;

        std::error_code link_ec;
        const fs::path executable = fs::read_symlink(entry.path() / "exe", link_ec);
        if (link_ec || executable.filename() != "unitree_mujoco") continue;

        const auto args = ReadProcessArgs(entry.path() / "cmdline");
        if (args.empty()) continue;

        const fs::path config_path = executable.parent_path().parent_path() / "config.yaml";
        const std::string default_robot = ReadSimpleYamlValue(config_path, "robot").value_or("");
        const fs::path default_scene = ReadSimpleYamlValue(config_path, "robot_scene").value_or("");
        const ParsedArgs parsed = ParseArgs(args, default_robot, default_scene);
        if (parsed.robot != "r1" || parsed.scene.empty()) continue;

        std::error_code cwd_ec;
        const fs::path process_cwd = fs::read_symlink(entry.path() / "cwd", cwd_ec);
        fs::path resolved = ResolveScenePath(
            executable, parsed.robot, parsed.scene,
            parsed.scene_from_command_line, cwd_ec ? fs::path{} : process_cwd);
        std::error_code canonical_ec;
        const fs::path canonical = fs::weakly_canonical(resolved, canonical_ec);
        if (!canonical_ec) resolved = canonical;
        matches.push_back({pid, resolved});
    }

    if (matches.empty()) {
        return {false, {},
            "chưa thấy tiến trình unitree_mujoco cho R1; hãy mở simulator khi policy đã sẵn sàng"};
    }

    std::map<std::string, std::vector<std::string>> scenes_to_pids;
    for (const auto& match : matches) {
        scenes_to_pids[match.scene.string()].push_back(match.pid);
    }
    if (scenes_to_pids.size() != 1) {
        std::ostringstream msg;
        msg << "phát hiện nhiều simulator R1 dùng scene khác nhau:";
        for (const auto& [scene, pids] : scenes_to_pids) {
            msg << "\n  " << scene << " (PID ";
            for (std::size_t i = 0; i < pids.size(); ++i) {
                if (i != 0) msg << ',';
                msg << pids[i];
            }
            msg << ')';
        }
        msg << "\nHãy chỉ giữ một simulator R1 hoặc đặt rough_scene_path rõ ràng.";
        return {false, {}, msg.str()};
    }

    const fs::path scene = scenes_to_pids.begin()->first;
    std::error_code file_ec;
    if (!fs::is_regular_file(scene, file_ec) || file_ec) {
        return {false, {}, "scene phát hiện từ simulator không tồn tại: " + scene.string()};
    }

    std::ostringstream msg;
    msg << "PID ";
    const auto& pids = scenes_to_pids.begin()->second;
    for (std::size_t i = 0; i < pids.size(); ++i) {
        if (i != 0) msg << ',';
        msg << pids[i];
    }
    return {true, scene, msg.str()};
}

}  // namespace SimulatorSceneResolver
