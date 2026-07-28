#pragma once

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// Gửi heartbeat/trạng thái audio một chiều tới r1_integration. Không nhận lệnh
// và không liên quan tới đường motor; lỗi socket không làm high-level dừng.
class IntegrationNotifier {
public:
    IntegrationNotifier() {
        const char* configured = std::getenv("HB_INTEGRATION_SOCKET");
        path_ = configured && *configured ? configured : "/run/hb/integration.sock";
        fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    }

    ~IntegrationNotifier() {
        if (fd_ >= 0) close(fd_);
    }

    IntegrationNotifier(const IntegrationNotifier&) = delete;
    IntegrationNotifier& operator=(const IntegrationNotifier&) = delete;

    void Send(bool busy, bool armed, const std::string& state = "") {
        if (fd_ < 0 || path_.size() >= sizeof(((sockaddr_un*)nullptr)->sun_path)) return;
        std::string token = state;
        std::replace_if(token.begin(), token.end(),
                        [](char c) { return c == ' ' || c == '(' || c == ')'; }, '_');
        std::string message = "v=1 busy=" + std::to_string(busy ? 1 : 0) +
                              " armed=" + std::to_string(armed ? 1 : 0);
        if (!token.empty()) message += " state=" + token;

        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
        std::lock_guard<std::mutex> lock(mu_);
        sendto(fd_, message.data(), message.size(), MSG_DONTWAIT,
               reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    }

private:
    int fd_ = -1;
    std::string path_;
    std::mutex mu_;
};
