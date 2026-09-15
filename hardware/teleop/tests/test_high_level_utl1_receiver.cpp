#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../../high_level/src/input/TeleopReceiver.hpp"

namespace {

constexpr int kTestPort = 45560;

void SendPacket(const TeleopReceiver::Packet& packet) {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(fd >= 0);
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    target.sin_port = htons(kTestPort);
    const ssize_t sent = sendto(fd, &packet, sizeof(packet), 0,
                                reinterpret_cast<const sockaddr*>(&target),
                                sizeof(target));
    assert(sent == static_cast<ssize_t>(sizeof(packet)));
    close(fd);
}

bool Near(float lhs, float rhs, float tolerance = 1e-3f) {
    return std::fabs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    static_assert(sizeof(TeleopReceiver::Packet) == 60, "UTL1 packet must stay 60 bytes");

    std::array<float, TeleopReceiver::kNumArm> neutral{};
    TeleopReceiver receiver;
    receiver.Init(neutral, true, kTestPort,
                  /*timeout_ms=*/100.0f,
                  /*blend_in_s=*/0.01f,
                  /*retract_s=*/0.01f,
                  /*smooth_hz=*/100.0f,
                  /*head_yaw_max=*/1.0f,
                  /*head_pitch_max=*/0.62f);

    TeleopReceiver::Packet command{};
    command.magic = TeleopReceiver::kMagic;
    command.seq = 1;
    command.enable = 1;
    command.arm_valid = 1;
    command.head_valid = 1;
    for (int i = 0; i < TeleopReceiver::kNumArm; ++i)
        command.arm_q[i] = 0.01f * static_cast<float>(i + 1);
    command.head_yaw = 0.25f;
    command.head_pitch = -0.20f;
    SendPacket(command);

    for (int i = 0; i < 100 && (!receiver.Active() || !receiver.HeadValid()); ++i) {
        receiver.Update(0.002f, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(receiver.Active());
    assert(receiver.HeadValid());
    assert(Near(receiver.HeadYaw(), command.head_yaw));
    assert(Near(receiver.HeadPitch(), command.head_pitch));

    for (int i = 0; i < 30; ++i) {
        receiver.Update(0.002f, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto& arm = receiver.ArmTarget();
    for (int i = 0; i < TeleopReceiver::kNumArm; ++i)
        assert(Near(arm[i], command.arm_q[i], 2e-3f));

    TeleopReceiver::Packet stop{};
    stop.magic = TeleopReceiver::kMagic;
    stop.seq = 2;
    stop.enable = 0;
    SendPacket(stop);
    for (int i = 0; i < 100 && receiver.Active(); ++i) {
        receiver.Update(0.002f, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    assert(!receiver.Active());
    assert(!receiver.HeadValid());

    receiver.Stop();
    std::cout << "UTL1 loopback receiver smoke test: PASS\n";
    return 0;
}
