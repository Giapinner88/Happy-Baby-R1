/**
 * main.cpp — Entry point cho HB R1 High-Level Runner.
 *
 * Cách chạy: ./run_r1 [interface]
 */

#include <iostream>
#include <string>

#include <libgen.h>
#include <unistd.h>

#include "app/Application.hpp"

namespace {

std::string GetProjectDir(const char* argv0) {
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
    if (argc >= 2) interface_override = argv[1];

    try {
        Application app(GetProjectDir(argv[0]), interface_override);
        return app.Run();
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }
}
