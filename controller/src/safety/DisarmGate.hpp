#pragma once
#include <cmath>

// Operator first enters passive Mode Z with physical support, releases buttons,
// then holds exact R1+R2. Software cannot infer that a stand/spotter supports weight.
class DisarmGate {
public:
    bool Update(bool passive, bool fresh, bool new_sample, bool neutral, bool combo,
                double observed_dt, float max_speed, float gyro_norm, double hold_s) {
        if (!passive || !fresh || !std::isfinite(max_speed) || !std::isfinite(gyro_norm) ||
            max_speed > 0.10f || gyro_norm > 0.15f) {
            Reset();
            return false;
        }
        if (!new_sample) return false;
        if (neutral) released_ = true;
        if (!combo || !released_) { elapsed_ = 0.0; return false; }
        elapsed_ += observed_dt;
        return elapsed_ + 1e-6 >= hold_s;
    }
    void Reset() { released_ = false; elapsed_ = 0.0; }
private:
    bool released_ = false;
    double elapsed_ = 0.0;
};
