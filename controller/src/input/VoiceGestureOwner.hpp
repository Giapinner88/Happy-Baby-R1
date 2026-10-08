#pragma once

#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <iostream>
#include <string>
#include <unordered_map>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

// Narrow IPC boundary between the voice process and the sole high-level motor
// owner. The voice process can request a stable gesture ID; it cannot provide
// joint targets, file paths, playback speed, or LowCmd data.
class VoiceGestureOwner {
public:
    struct Request {
        int fd = -1;
        std::string request_id;
        std::string command;
        std::string gesture_id;
        std::chrono::steady_clock::time_point received_at{};
    };

    ~VoiceGestureOwner() { Close(); }

    bool Init(const std::string& path) {
        Close();
        sockaddr_un address{};
        if (path.empty() || path.front() != '/') return Fail("path_must_be_absolute", false);
        if (path.size() >= sizeof(address.sun_path)) return Fail("path_too_long", false);
        path_ = path;

        // A crashed previous runner can leave a socket inode behind. Remove it
        // only when no listener owns it; never steal a live owner's path.
        struct stat existing {};
        if (::lstat(path.c_str(), &existing) == 0) {
            if (!S_ISSOCK(existing.st_mode) || ExistingListener(path))
                return Fail("socket_already_owned_or_not_a_socket");
            if (::unlink(path.c_str()) != 0) return Fail("unlink_stale_socket");
        }

        listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (listen_fd_ < 0) return Fail("socket");
        address.sun_family = AF_UNIX;
        std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
        if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
            return Fail("bind");
        bound_ = true;
        if (::chmod(path.c_str(), 0660) != 0) return Fail("chmod");
        if (::listen(listen_fd_, 4) != 0) return Fail("listen");
        std::cout << "[GestureOwner] listening " << path << "\n";
        return true;
    }

    // Called from the existing 500 Hz high-level loop. All socket operations
    // are nonblocking and each request is bounded to 2 KiB.
    void Poll() {
        if (listen_fd_ < 0) return;
        AcceptOne();
        ReadIncoming();
        const auto now = std::chrono::steady_clock::now();
        if (incoming_fd_ >= 0 && SecondsSince(incoming_at_, now) > kRequestTtlS)
            CloseIncomingWith(false, "expired", "request_ttl_exceeded");
        if (pending_.fd >= 0 && SecondsSince(pending_.received_at, now) > kRequestTtlS)
            Reply(false, "expired", "request_ttl_exceeded");
    }

    bool HasPending() const { return pending_.fd >= 0; }
    const Request& Pending() const { return pending_; }

    void Reply(bool ok, const std::string& status, const std::string& reason = "",
               const std::string& gesture_id = "") {
        if (pending_.fd < 0) return;
        const std::string response = MakeReply(ok, status, reason, gesture_id,
                                               pending_.request_id);
        WriteAndClose(pending_.fd, response);
        CacheReply(pending_.request_id, response);
        pending_ = Request{};
    }

    void Close() {
        if (incoming_fd_ >= 0) ::close(incoming_fd_);
        incoming_fd_ = -1;
        incoming_.clear();
        if (pending_.fd >= 0) ::close(pending_.fd);
        pending_ = Request{};
        if (listen_fd_ >= 0) ::close(listen_fd_);
        listen_fd_ = -1;
        if (bound_ && !path_.empty()) ::unlink(path_.c_str());
        bound_ = false;
        path_.clear();
    }

private:
    static constexpr float kRequestTtlS = 2.0f;
    static constexpr size_t kMaxRequestBytes = 2048;
    static constexpr size_t kReplyCacheSize = 128;

    static float SecondsSince(std::chrono::steady_clock::time_point begin,
                              std::chrono::steady_clock::time_point end) {
        return std::chrono::duration<float>(end - begin).count();
    }

    static bool ExistingListener(const std::string& path) {
        const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0) return true;
        sockaddr_un address{};
        address.sun_family = AF_UNIX;
        std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1);
        const int result = ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
        const int error = errno;
        ::close(fd);
        return result == 0 || error == EINPROGRESS || error == EAGAIN ||
               error == EALREADY || error == EACCES;
    }

    void AcceptOne() {
        for (;;) {
            const int fd = ::accept4(listen_fd_, nullptr, nullptr,
                                     SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (fd < 0) {
                if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                    std::cerr << "[GestureOwner] accept: " << std::strerror(errno) << "\n";
                return;
            }
            if (incoming_fd_ >= 0 || pending_.fd >= 0) {
                WriteAndClose(fd, MakeReply(false, "busy", "request_pending"));
                continue;
            }
            incoming_fd_ = fd;
            incoming_.clear();
            incoming_at_ = std::chrono::steady_clock::now();
            return;
        }
    }

    void ReadIncoming() {
        if (incoming_fd_ < 0) return;
        char buffer[512];
        for (;;) {
            const ssize_t count = ::recv(incoming_fd_, buffer, sizeof(buffer), 0);
            if (count > 0) {
                incoming_.append(buffer, static_cast<size_t>(count));
                if (incoming_.size() > kMaxRequestBytes) {
                    CloseIncomingWith(false, "rejected", "invalid_request");
                    return;
                }
                if (incoming_.find('\n') != std::string::npos) break;
                continue;
            }
            if (count == 0) break;
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            CloseIncomingWith(false, "rejected", "invalid_request");
            return;
        }
        ParseIncoming();
    }

    static bool SafeToken(const std::string& value, size_t max_size) {
        if (value.empty() || value.size() > max_size) return false;
        for (const unsigned char ch : value) {
            if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                  (ch >= '0' && ch <= '9') || ch == '_' || ch == '-'))
                return false;
        }
        return true;
    }

    static bool ReadStringField(const std::string& json, const char* name,
                                std::string& value, bool required) {
        const std::string token = std::string("\"") + name + "\"";
        const size_t key = json.find(token);
        if (key == std::string::npos) return !required;
        if (json.find(token, key + token.size()) != std::string::npos) return false;
        size_t pos = json.find(':', key + token.size());
        if (pos == std::string::npos) return false;
        ++pos;
        while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
        if (pos < json.size() && json.compare(pos, 4, "null") == 0)
            return !required;
        if (pos >= json.size() || json[pos++] != '"') return false;
        const size_t end = json.find('"', pos);
        if (end == std::string::npos) return false;
        // Protocol atoms deliberately exclude JSON escaping and arbitrary text.
        if (json.find('\\', pos) < end) return false;
        value = json.substr(pos, end - pos);
        return SafeToken(value, 128);
    }

    void ParseIncoming() {
        std::string line = incoming_;
        const size_t newline = line.find('\n');
        if (newline == std::string::npos || line.find('\n', newline + 1) != std::string::npos) {
            CloseIncomingWith(false, "rejected", "invalid_request");
            return;
        }
        line.resize(newline);
        Request request;
        request.fd = incoming_fd_;
        request.received_at = std::chrono::steady_clock::now();
        const bool parsed = ReadStringField(line, "request_id", request.request_id, true) &&
            ReadStringField(line, "command", request.command, true) &&
            ReadStringField(line, "gesture_id", request.gesture_id, false);
        incoming_fd_ = -1;
        incoming_.clear();

        const bool command_ok = request.command == "play" || request.command == "cancel" ||
                                request.command == "status";
        if (!parsed || !command_ok || (request.command == "play" && request.gesture_id.empty())) {
            WriteAndClose(request.fd, MakeReply(false, "rejected", "invalid_request"));
            return;
        }
        const auto cached = cached_replies_.find(request.request_id);
        if (cached != cached_replies_.end()) {
            WriteAndClose(request.fd, cached->second);
            return;
        }
        pending_ = std::move(request);
    }

    void CacheReply(const std::string& request_id, const std::string& response) {
        cached_replies_[request_id] = response;
        reply_order_.push_back(request_id);
        while (reply_order_.size() > kReplyCacheSize) {
            cached_replies_.erase(reply_order_.front());
            reply_order_.pop_front();
        }
    }

    void CloseIncomingWith(bool ok, const std::string& status, const std::string& reason) {
        if (incoming_fd_ >= 0)
            WriteAndClose(incoming_fd_, MakeReply(ok, status, reason));
        incoming_fd_ = -1;
        incoming_.clear();
    }

    static std::string MakeReply(bool ok, const std::string& status,
                                 const std::string& reason,
                                 const std::string& gesture_id = "",
                                 const std::string& request_id = "") {
        std::string out = "{\"ok\":" + std::string(ok ? "true" : "false") +
                          ",\"status\":\"" + status + "\"";
        if (!reason.empty()) out += ",\"reason\":\"" + reason + "\"";
        if (!gesture_id.empty()) out += ",\"gesture_id\":\"" + gesture_id + "\"";
        if (!request_id.empty()) out += ",\"request_id\":\"" + request_id + "\"";
        return out + "}\n";
    }

    static void WriteAndClose(int fd, const std::string& response) {
        size_t sent = 0;
        while (sent < response.size()) {
            const ssize_t count = ::send(fd, response.data() + sent,
                                         response.size() - sent, MSG_NOSIGNAL);
            if (count > 0) {
                sent += static_cast<size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) continue;
            break;
        }
        ::close(fd);
    }

    bool Fail(const char* operation, bool include_errno = true) {
        std::cerr << "[GestureOwner] " << operation;
        if (include_errno) std::cerr << ": " << std::strerror(errno);
        std::cerr << "\n";
        Close();
        return false;
    }

    int listen_fd_ = -1;
    int incoming_fd_ = -1;
    std::string path_;
    std::string incoming_;
    std::chrono::steady_clock::time_point incoming_at_{};
    Request pending_;
    std::unordered_map<std::string, std::string> cached_replies_;
    std::deque<std::string> reply_order_;
    bool bound_ = false;
};
