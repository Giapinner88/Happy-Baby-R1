/**
 * Điểm vào (Entry point) cho ứng dụng HB R1 High-Level Runner.
 * Quản lý khởi tạo, xử lý tín hiệu hệ thống (signal) và vòng lặp chính của robot.
 */
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include <libgen.h>
#include <unistd.h>

#include "app/Application.hpp"

namespace {

/**
 * Thiết lập cờ yêu cầu dừng khi nhận được tín hiệu SIGTERM hoặc SIGINT.
 * Giúp ứng dụng thoát an toàn (thoát êm) và kích hoạt chế độ Damping(), ngăn robot rơi tự do.
 */
void OnStopSignal(int sig) { Application::RequestStop(sig); }

/**
 * Xác định thư mục gốc của dự án (project root) dựa trên đường dẫn của file thực thi.
 * Ưu tiên sử dụng biến môi trường HB_PROJECT_DIR nếu được thiết lập.
 */
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
