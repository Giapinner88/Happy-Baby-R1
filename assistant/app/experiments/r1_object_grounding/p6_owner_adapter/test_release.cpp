// Pure APV1 lease test: no socket, DDS, policy inference or robot connection.
#include "ApproachReceiver.hpp"
#include <cassert>
#include <iostream>

int main() {
    using namespace approach_owner;
    Lease lease;
    const uint64_t now = 1000000;
    Packet p{kMagic, 1, Hello, 7, 1, now + 100000, 0, 0};
    assert(lease.Receive(p, now, true, false, false, true).status == Ready);
    p.kind = Release; p.sequence = 2;
    assert(lease.Receive(p, now, true, false, false, true).status == Session);
    // Timeout remains latched at zero; it cannot implicitly reactivate search yaw.
    assert(lease.Tick(now + 100001, true, false, false));
    assert(lease.vx() == 0 && lease.wz() == 0);
    p.kind = Stop; p.sequence = 3; p.deadline_us = now + 200000;
    assert(lease.Receive(p, now + 100002, true, false, false, true).status == Stopped);
    p.kind = Release; p.sequence = 4; p.session = 8;
    assert(lease.Receive(p, now + 100003, true, false, false, true).status == Session);
    p.session = 7; p.vx = .01f;
    assert(lease.Receive(p, now + 100003, true, false, false, true).status == Session);
    p.vx = 0;
    assert(lease.Receive(p, now + 100003, true, false, false, true).status == Released);
    assert(!lease.Tick(now + 100004, true, false, false));
    // A released session cannot replay a command or release twice.
    p.kind = Command; p.sequence = 5;
    assert(lease.Receive(p, now + 100004, true, false, false, true).status == Session);
    p.kind = Release; p.sequence = 4;
    assert(lease.Receive(p, now + 100004, true, false, false, true).status == Sequence);
    p.kind = Hello; p.session = 9; p.sequence = 1;
    assert(lease.Receive(p, now + 100004, true, false, false, true).status == Ready);
    std::cout << "APV1_HANDOFF_OK\n";
}
