#pragma once

// Exact-artifact gate evaluated once at startup, outside the 500 Hz loop.
// Uses sha256sum already required by the HB deployment preflight; no shell.
#include <algorithm>
#include <cerrno>
#include <string>

#include <sys/wait.h>
#include <unistd.h>

namespace teleop_h4 {

inline bool IsLowerSha256(const std::string& value) {
    return value.size() == 64 && std::all_of(value.begin(), value.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

inline std::string Sha256File(const std::string& path) {
    int fds[2];
    if (pipe(fds) != 0) return {};
    const pid_t pid = fork();
    if (pid < 0) {
        close(fds[0]);
        close(fds[1]);
        return {};
    }
    if (pid == 0) {
        close(fds[0]);
        if (dup2(fds[1], STDOUT_FILENO) < 0) _exit(127);
        close(fds[1]);
        execlp("sha256sum", "sha256sum", "--", path.c_str(), nullptr);
        _exit(127);
    }
    close(fds[1]);
    std::string output;
    char buffer[256];
    while (true) {
        const ssize_t n = read(fds[0], buffer, sizeof(buffer));
        if (n > 0) output.append(buffer, static_cast<size_t>(n));
        else if (n == 0) break;
        else if (errno != EINTR) break;
    }
    close(fds[0]);
    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) return {};
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0 || output.size() < 66 ||
        output[64] != ' ') return {};
    const std::string hash = output.substr(0, 64);
    return IsLowerSha256(hash) ? hash : std::string{};
}

inline bool Approved(bool enabled, const std::string& contract,
                     const std::string& path, const std::string& expected_hash) {
    return enabled && contract == "flat_plus_gait_h4_v1" &&
           IsLowerSha256(expected_hash) && Sha256File(path) == expected_hash;
}

}  // namespace teleop_h4
