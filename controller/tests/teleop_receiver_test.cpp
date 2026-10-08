#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

#include "input/TeleopReceiver.hpp"

namespace {

void Require(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Requirement failed at line " << line << ": " << expression << "\n";
        std::abort();
    }
}

int FreeLoopbackPort() {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    socklen_t size = sizeof(addr);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &size) < 0) {
        close(fd);
        return -1;
    }
    const int port = ntohs(addr.sin_port);
    close(fd);
    return port;
}

void SendPacket(int fd, int port, const TeleopReceiver::Packet& packet) {
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port = htons(static_cast<uint16_t>(port));
    const ssize_t sent = sendto(fd, &packet, sizeof(packet), 0,
                                reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    if (sent != static_cast<ssize_t>(sizeof(packet))) std::abort();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

TeleopReceiver::Packet Packet(uint32_t sequence) {
    TeleopReceiver::Packet packet{};
    packet.magic = TeleopReceiver::kMagic;
    packet.seq = sequence;
    packet.enable = 1;
    packet.arm_valid = 1;
    packet.head_valid = 1;
    for (int i = 0; i < TeleopReceiver::kNumArm; ++i)
        packet.arm_q[i] = 0.1f * static_cast<float>(i + 1);
    packet.head_yaw = 0.2f;
    packet.head_pitch = -0.1f;
    return packet;
}

}  // namespace

#define REQUIRE(condition) Require(static_cast<bool>(condition), #condition, __LINE__)

int main() {
    const int port = FreeLoopbackPort();
    REQUIRE(port > 0);
    std::array<float, TeleopReceiver::kNumArm> default_arm{};
    TeleopReceiver receiver;
    receiver.Init(default_arm, true, port,
                  /*timeout_ms=*/30.0f, /*blend_in_s=*/0.05f,
                  /*retract_s=*/0.05f, /*smooth_hz=*/8.0f,
                  /*head_yaw_max=*/1.0f, /*head_pitch_max=*/0.5f);

    const int tx = socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(tx >= 0);

    auto nan_arm = Packet(1);
    nan_arm.arm_q[4] = std::numeric_limits<float>::quiet_NaN();
    SendPacket(tx, port, nan_arm);
    receiver.Update(0.01f, true);
    REQUIRE(!receiver.Active());

    auto huge_arm = Packet(2);
    huge_arm.arm_q[2] = 9.9f;
    SendPacket(tx, port, huge_arm);
    receiver.Update(0.01f, true);
    REQUIRE(!receiver.Active());

    auto bad_yaw = Packet(3);
    bad_yaw.head_yaw = 1.01f;
    SendPacket(tx, port, bad_yaw);
    receiver.Update(0.01f, true);
    REQUIRE(!receiver.Active());

    auto bad_pitch = Packet(4);
    bad_pitch.head_pitch = -0.51f;
    SendPacket(tx, port, bad_pitch);
    receiver.Update(0.01f, true);
    REQUIRE(!receiver.Active());

    // Gói hợp lệ sau các gói bị loại vẫn được nhận bình thường.
    const auto valid = Packet(5);
    SendPacket(tx, port, valid);
    receiver.Update(0.01f, true);
    REQUIRE(receiver.Active());
    REQUIRE(receiver.ArmValid());
    REQUIRE(receiver.HeadValid());
    REQUIRE(receiver.Weight() > 0.0f && receiver.Weight() < 1.0f);
    REQUIRE(receiver.ArmTarget()[0] > 0.0f);

    // Watchdog stale phải xả hết authority tay và đầu.
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    receiver.Update(0.10f, true);
    REQUIRE(!receiver.Active());
    REQUIRE(!receiver.ArmValid());
    REQUIRE(!receiver.HeadValid());

    // Head-only có thể điều khiển đầu nhưng tuyệt đối không cấp authority tay.
    auto head_only = Packet(6);
    head_only.arm_valid = 0;
    SendPacket(tx, port, head_only);
    receiver.Update(0.01f, true);
    REQUIRE(receiver.Active());
    REQUIRE(!receiver.ArmValid());
    REQUIRE(receiver.HeadValid());
    REQUIRE(receiver.Weight() == 0.0f);

    // H4 must never auto-arm from the old pad=0 stream or an enable packet
    // buffered before entering LOCOMOTION. It needs a new explicit STOP edge.
    receiver.BeginH4Session();
    auto h4 = Packet(7);
    h4.pad = 1;
    SendPacket(tx, port, h4);
    REQUIRE(!receiver.H4Authorized());
    auto h4_stop = h4;
    h4_stop.seq = 8;
    h4_stop.enable = 0;
    SendPacket(tx, port, h4_stop);
    REQUIRE(!receiver.H4Authorized());
    h4.seq = 9;
    SendPacket(tx, port, h4);
    REQUIRE(receiver.H4Authorized());
    auto legacy = Packet(10);
    SendPacket(tx, port, legacy);
    REQUIRE(!receiver.H4Authorized());
    h4.seq = 11;
    SendPacket(tx, port, h4);
    REQUIRE(!receiver.H4Authorized());

    // A sidecar process restart may reset sequence numbers only after its
    // previous lease expires, and must begin with a fresh explicit STOP.
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    h4_stop.seq = 0;
    SendPacket(tx, port, h4_stop);
    REQUIRE(!receiver.H4Authorized());
    h4.seq = 1;
    SendPacket(tx, port, h4);
    REQUIRE(receiver.H4Authorized());

    // An advancing packet after a gap still requires a new STOP edge. The
    // application might not call H4Authorized during the missing-packet gap.
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    h4.seq = 2;
    SendPacket(tx, port, h4);
    REQUIRE(!receiver.H4Authorized());
    h4_stop.seq = 3;
    SendPacket(tx, port, h4_stop);
    REQUIRE(!receiver.H4Authorized());
    h4.seq = 4;
    SendPacket(tx, port, h4);
    REQUIRE(receiver.H4Authorized());

    // Authorization and Update run on different ticks/threads. A legacy
    // packet arriving between them must not be consumed as an H4 target.
    legacy.seq = 5;
    SendPacket(tx, port, legacy);
    receiver.Update(0.01f, true, /*require_explicit_deadman=*/true);
    REQUIRE(!receiver.ArmValid());
    REQUIRE(!receiver.HeadValid());

    close(tx);
    receiver.Stop();
    return 0;
}
