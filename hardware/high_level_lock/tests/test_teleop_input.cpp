#include <cassert>
#include <cmath>
#include <string>

#include "../src/input/TeleopInput.hpp"

int main() {
    TeleopFrame frame;
    bool stop = false;
    assert(TeleopInput::Parse(
        "HBTELEOP1 TARGET 7 0 1 2 3 4 5 6 7 8 9 10 11", frame, stop));
    assert(!stop);
    assert(frame.sequence == 7);
    assert(frame.q[0] == 0.0f && frame.q[11] == 11.0f);

    assert(TeleopInput::Parse("HBTELEOP1 STOP 8", frame, stop));
    assert(stop && frame.sequence == 8);

    assert(!TeleopInput::Parse("HBTELEOP0 STOP 9", frame, stop));
    assert(!TeleopInput::Parse("HBTELEOP1 TARGET 9 0 1", frame, stop));
    assert(!TeleopInput::Parse(
        "HBTELEOP1 TARGET 9 0 1 2 3 4 5 6 7 8 9 10 nan", frame, stop));
    assert(!TeleopInput::Parse("HBTELEOP1 STOP 9 trailing", frame, stop));
    return 0;
}
