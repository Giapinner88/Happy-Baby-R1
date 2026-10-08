#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <cnpy.h>

namespace motion_array {
inline void Validate(const cnpy::NpyArray& a, const char* key) {
    if (a.type != 'f' || (a.word_size != 4 && a.word_size != 8) || a.fortran_order)
        throw std::runtime_error(std::string(key) + " must be C-order float32/float64");
    if (a.num_vals == 0)
        throw std::runtime_error(std::string(key) + " must not be empty");
    for (size_t i = 0; i < a.num_vals; ++i) {
        const double v = a.word_size == 4 ? a.data<float>()[i] : a.data<double>()[i];
        if (!std::isfinite(v) || std::abs(v) > std::numeric_limits<float>::max())
            throw std::runtime_error(std::string(key) + " contains non-finite/float32-overflow data");
    }
}

inline float Scalar(const cnpy::NpyArray& a, size_t i) {
    return a.word_size == 4 ? a.data<float>()[i] : static_cast<float>(a.data<double>()[i]);
}

inline float Fps(const cnpy::npz_t& npz) {
    auto it = npz.find("fps");
    if (it == npz.end()) return 50.0f;
    Validate(it->second, "fps");
    const float fps = Scalar(it->second, 0);
    if (it->second.num_vals != 1 || fps <= 0.0f)
        throw std::runtime_error("fps must be one finite positive float");
    return fps;
}

inline int Frames(size_t n) {
    if (n < 2 || n > static_cast<size_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("Motion needs 2..INT_MAX frames");
    return static_cast<int>(n);
}
} // namespace motion_array
