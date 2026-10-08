#pragma once
#include <chrono>

// Only observed intervals contribute to operator hold intent. A gap >20 ms
// pauses progress; it neither completes a hold nor erases earlier progress.
// This is input timing tolerance, independent of the PD watchdog.
class SampleProgress {
public:
    using TimePoint = std::chrono::steady_clock::time_point;
    double Update(bool new_sample, TimePoint received) {
        if (!new_sample) return 0.0;
        const double dt = have_ ? std::chrono::duration<double>(received - last_).count() : 0.0;
        last_ = received;
        have_ = true;
        return dt > 0.0 && dt <= 0.020 ? dt : 0.0;
    }
private:
    bool have_ = false;
    TimePoint last_{};
};
