/**
 * Điểm vào (Entry point) cho ứng dụng HB R1 High-Level Runner.
 * Quản lý khởi tạo, xử lý tín hiệu hệ thống (signal) và vòng lặp chính của robot.
 */
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include <libgen.h>
#include <unistd.h>

#include <onnxruntime_cxx_api.h>

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

/**
 * In chu kỳ gait và dwell lúc train của một file ONNX, để `hb policy add` dập
 * được chúng vào policy.yaml của bundle mới. Shell không có onnxruntime, mà
 * bundle thiếu hai khoá này thì run_r1 từ chối khởi động.
 */
int PrintPolicyMeta(const std::string& model_path) {
    try {
        Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "hb-policy-meta");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        Ort::Session session(env, model_path.c_str(), options);
        Ort::AllocatorWithDefaultOptions allocator;
        Ort::ModelMetadata metadata = session.GetModelMetadata();
        for (const char* key : {"policy_contract", "gait_period_s", "gait_t_settle_s",
                                "gait_phase_contract"}) {
            Ort::AllocatedStringPtr value =
                metadata.LookupCustomMetadataMapAllocated(key, allocator);
            if (value) std::cout << key << "=" << value.get() << "\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] khong doc duoc metadata cua " << model_path << ": "
                  << e.what() << "\n";
        return 1;
    }
}

} // namespace

int main(int argc, const char** argv) {
    // journald nhận stdout qua pipe => mặc định đệm theo khối, log vận hành có
    // thể xuất hiện muộn hàng giây và làm sai lệch mốc thời gian khi điều tra
    // sự cố. Đệm theo dòng để thứ tự log đúng với thứ tự sự việc.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::string interface_override;
    std::string policy_meta;
    bool preflight = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--preflight") preflight = true;
        // Chỉ shell mới cần: `hb policy add` không có onnxruntime để tự đọc
        // metadata, mà policy.yaml của bundle mới phải khai đúng chu kỳ/dwell.
        else if (arg == "--policy-meta" && i + 1 < argc) policy_meta = argv[++i];
        else interface_override = arg;
    }
    if (!policy_meta.empty()) return PrintPolicyMeta(policy_meta);

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
