#pragma once
/**
 * Filters.hpp — Lọc thông thấp bậc 1.
 */

#include <array>
#include <cmath>

class LowPass1 {
public:
    void Configure(float cutoff_hz, float dt) {
        if (cutoff_hz <= 0.0f) {
            alpha_ = 1.0f; // truyền thẳng
        } else {
            float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_hz);
            alpha_ = dt / (dt + rc);
        }
        initialized_ = false;
    }

    float Update(float x) {
        if (!initialized_) { y_ = x; initialized_ = true; }
        y_ += alpha_ * (x - y_);
        return y_;
    }

    void Reset() { initialized_ = false; }
    float value() const { return y_; }

private:
    float alpha_ = 1.0f;
    float y_ = 0.0f;
    bool initialized_ = false;
};

template <size_t N>
class LowPassVec {
public:
    void Configure(float cutoff_hz, float dt) {
        for (auto& f : filters_) f.Configure(cutoff_hz, dt);
    }
    void Update(const std::array<float, N>& x, std::array<float, N>& y) {
        for (size_t i = 0; i < N; ++i) y[i] = filters_[i].Update(x[i]);
    }
    void Reset() {
        for (auto& f : filters_) f.Reset();
    }

private:
    std::array<LowPass1, N> filters_;
};
