#include "../src/input/VoiceGestureOwner.hpp"

#include <cassert>
#include <cstdio>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

int Connect(const std::string& path) {
    const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    assert(fd >= 0);
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    assert(path.size() < sizeof(address.sun_path));
    std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", path.c_str());
    assert(::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    return fd;
}

void SendLine(int fd, const std::string& line) {
    const std::string request = line + "\n";
    size_t sent = 0;
    while (sent < request.size()) {
        const ssize_t count = ::write(fd, request.data() + sent, request.size() - sent);
        assert(count > 0);
        sent += static_cast<size_t>(count);
    }
}

std::string ReadLine(int fd) {
    std::string result;
    char ch = 0;
    while (::read(fd, &ch, 1) == 1 && ch != '\n') result.push_back(ch);
    return result;
}

void TestPlayAndDuplicate(const std::string& path) {
    VoiceGestureOwner owner;
    assert(owner.Init(path));
    const std::string request =
        R"({"v":1,"request_id":"call_1","command":"play","gesture_id":"wave"})";

    int fd = Connect(path);
    SendLine(fd, request);
    owner.Poll();
    assert(owner.HasPending());
    assert(owner.Pending().command == "play");
    assert(owner.Pending().gesture_id == "wave");
    owner.Reply(true, "accepted", "", "wave");
    const std::string first = ReadLine(fd);
    ::close(fd);
    assert(first.find("\"status\":\"accepted\"") != std::string::npos);

    // Interleave another completed request, then replay the first ID. The
    // owner must still return its prior result instead of queueing it again.
    fd = Connect(path);
    SendLine(fd, R"({"request_id":"call_status","command":"status"})");
    owner.Poll();
    assert(owner.HasPending());
    assert(owner.Pending().command == "status");
    owner.Reply(true, "idle");
    ::close(fd);

    fd = Connect(path);
    SendLine(fd, request);
    owner.Poll();
    const std::string duplicate = ReadLine(fd);
    ::close(fd);
    assert(duplicate == first);

    // A second high-level process must not unlink or replace the live owner.
    VoiceGestureOwner second_owner;
    assert(!second_owner.Init(path));
    fd = Connect(path);
    ::close(fd);
    owner.Close();
}

void TestInvalidCommand(const std::string& path) {
    VoiceGestureOwner owner;
    assert(owner.Init(path));
    const int fd = Connect(path);
    SendLine(fd,
        R"({"v":1,"request_id":"call_2","command":"raw_motor","gesture_id":"wave"})");
    owner.Poll();
    assert(!owner.HasPending());
    const std::string response = ReadLine(fd);
    ::close(fd);
    assert(response.find("\"status\":\"rejected\"") != std::string::npos);
    owner.Close();
}

}  // namespace

int main() {
    const std::string base = "/tmp/hb_voice_gesture_owner_" + std::to_string(::getpid());
    TestPlayAndDuplicate(base + "_play.sock");
    TestInvalidCommand(base + "_invalid.sock");
    return 0;
}
