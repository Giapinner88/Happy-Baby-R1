#include <cassert>
#include <string>
#include <unistd.h>

#include "policy/TeleopH4PolicyGate.hpp"

int main() {
    char path[] = "/tmp/hb_h4_policy_gate_XXXXXX";
    const int fd = mkstemp(path);
    assert(fd >= 0);
    assert(write(fd, "abc", 3) == 3);
    close(fd);
    const std::string hash =
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
    assert(teleop_h4::Sha256File(path) == hash);
    assert(!teleop_h4::Approved(false, "flat_plus_gait_h4_v1", path, hash));
    assert(!teleop_h4::Approved(true, "flat_plus_h4_v1", path, hash));
    assert(!teleop_h4::Approved(true, "flat_plus_gait_h4_v1", path, std::string(64, '0')));
    assert(teleop_h4::Approved(true, "flat_plus_gait_h4_v1", path, hash));
    unlink(path);
    assert(!teleop_h4::Approved(true, "flat_plus_gait_h4_v1", path, hash));
}
