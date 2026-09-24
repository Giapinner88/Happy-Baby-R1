#include "motion/GestureLoader.hpp"

#include <cstddef>
#include <exception>
#include <fstream>
#include <iostream>

#include <cnpy.h>

std::vector<std::array<float, 10>> LoadGestureNpz(
    const std::string& path,
    float& frames_per_second,
    bool& loaded) {
    loaded = false;
    frames_per_second = 50.0f;
    std::vector<std::array<float, 10>> frames;
    try {
        cnpy::npz_t archive = cnpy::npz_load(path);
        auto joint_position = archive.find("joint_pos");
        if (joint_position == archive.end()) return frames;
        cnpy::NpyArray& values = joint_position->second;
        if (values.shape.size() != 2 || values.shape[1] < 24) return frames;

        const std::size_t frame_count = values.shape[0];
        const std::size_t column_count = values.shape[1];
        frames.reserve(frame_count);
        const bool is_double = values.word_size == 8;
        const double* doubles = is_double ? values.data<double>() : nullptr;
        const float* floats = is_double ? nullptr : values.data<float>();
        for (std::size_t frame = 0; frame < frame_count; ++frame) {
            std::array<float, 10> arm{};
            for (std::size_t joint = 0; joint < arm.size(); ++joint) {
                const std::size_t index = frame * column_count + 14 + joint;
                arm[joint] = is_double
                    ? static_cast<float>(doubles[index])
                    : floats[index];
            }
            frames.push_back(arm);
        }

        auto fps = archive.find("fps");
        if (fps != archive.end()) {
            if (fps->second.word_size == 8) {
                frames_per_second =
                    static_cast<float>(fps->second.data<double>()[0]);
            } else if (fps->second.word_size == 4) {
                frames_per_second = fps->second.data<float>()[0];
            }
        }
        loaded = true;
    } catch (const std::exception& error) {
        // Trying .loop.npz before .npz is normal. Report only an existing file
        // that could not be decoded.
        std::ifstream probe(path);
        if (probe.good()) {
            std::cerr << "[Gesture] loi nap " << path << ": "
                      << error.what() << '\n';
        }
    }
    return frames;
}
