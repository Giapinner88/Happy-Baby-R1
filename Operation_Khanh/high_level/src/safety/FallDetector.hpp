#pragma once
/**
 * FallDetector.hpp — Phát hiện ngã.
 */

#include <cmath>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/Tuning.hpp"

class FallDetector {
public:
    void Configure(const Tuning& tuning, float dt) {
        enabled_ = tuning.fall_enabled;
        tilt_thresh_      = -std::cos(tuning.fall_tilt_deg * static_cast<float>(M_PI) / 180.0f);
        flip_tilt_thresh_ = -std::cos(tuning.fall_flip_tilt_deg * static_cast<float>(M_PI) / 180.0f);
        flip_gyro_thresh_ = tuning.fall_flip_gyro;
        debounce_ticks_   = std::max(1, static_cast<int>(tuning.fall_debounce_ms / 1000.0f / dt));
        Reset();
    }

    void Reset() {
        is_fallen_ = false;
        tilt_count_ = 0;
        flip_count_ = 0;
    }

    bool Check(const Eigen::Vector3f& projected_gravity,
               const Eigen::Vector3f& gyro,
               std::vector<std::string>& reasons) {
        if (!enabled_) return false;
        if (is_fallen_) return true;

        float gz = projected_gravity.z();

        tilt_count_ = (gz > tilt_thresh_) ? tilt_count_ + 1 : 0;
        bool flip_now = (gz > flip_tilt_thresh_) && (gyro.norm() > flip_gyro_thresh_);
        flip_count_ = flip_now ? flip_count_ + 1 : 0;

        if (tilt_count_ >= debounce_ticks_)
            reasons.push_back("Nghiêng quá mức kéo dài (gz=" + std::to_string(gz) + ")");
        if (flip_count_ >= debounce_ticks_)
            reasons.push_back("Lật nhanh kéo dài (|gyro|=" + std::to_string(gyro.norm()) + ")");

        if (tilt_count_ >= debounce_ticks_ || flip_count_ >= debounce_ticks_) {
            is_fallen_ = true;
            return true;
        }
        return false;
    }

    bool is_fallen() const { return is_fallen_; }

private:
    bool enabled_ = true;
    float tilt_thresh_ = 0.0f;
    float flip_tilt_thresh_ = 0.0f;
    float flip_gyro_thresh_ = 6.0f;
    int debounce_ticks_ = 15;

    bool is_fallen_ = false;
    int tilt_count_ = 0;
    int flip_count_ = 0;
};
