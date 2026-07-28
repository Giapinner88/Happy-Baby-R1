// Entry point for HB R1 High-Level Runner

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include <libgen.h>
#include <unistd.h>

#include "app/Application.hpp"

namespace {

// Thoát êm khi có tín hiệu dừng
void OnStopSignal(int sig) { Application::RequestStop(sig); }

// Get project root directory based on the executable path
std::string GetProjectDir(const char* argv0) {
    const char* configured = std::getenv("HB_PROJECT_DIR");
    if (configured && *configured) return configured;
    char buf[4096];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len > 0) {
        buf[len] = '\0';
        return std::string(dirname(buf)) + "/..";
    }
    return std::string(dirname(const_cast<char*>(argv0))) + "/..";
}

} // namespace

int main(int argc, const char** argv) {
    std::string interface_override;
    bool preflight = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--preflight") preflight = true;
        else interface_override = argv[i];
    }

    std::signal(SIGTERM, OnStopSignal);
    std::signal(SIGINT, OnStopSignal);

    try {
        Application app(GetProjectDir(argv[0]), interface_override);
        return preflight ? app.Preflight() : app.Run();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
