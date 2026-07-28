#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/Tuning.hpp"

// Bộ phát hiện ngã (Fall Detector)
class FallDetector {
public:
    void Configure(const Tuning& tuning, float dt) {
        enabled_ = tuning.fall_enabled;
        tilt_thresh_      = -std::cos(tuning.fall_tilt_deg * static_cast<float>(M_PI) / 180.0f);
        flip_tilt_thresh_ = -std::cos(tuning.fall_flip_tilt_deg * static_cast<float>(M_PI) / 180.0f);
        flip_gyro_thresh_ = tuning.fall_flip_gyro;
        debounce_ticks_   = std::max(1, static_cast<int>(tuning.fall_debounce_ms / 1000.0f / dt));

        // Bảo vệ tốc độ góc khớp
        jspeed_enabled_    = tuning.joint_speed_guard_enabled;
        jspeed_thresh_     = tuning.joint_speed_limit;
        jspeed_ticks_      = std::max(1, static_cast<int>(tuning.joint_speed_debounce_ms / 1000.0f / dt));
        Reset();
    }

    void Reset() {
        is_fallen_ = false;
        tilt_count_ = 0;
        flip_count_ = 0;
        jspeed_count_ = 0;
    }

    // Kiểm tra các điều kiện để phát hiện ngã
    bool Check(const Eigen::Vector3f& projected_gravity,
               const Eigen::Vector3f& gyro,
               float max_joint_speed, int worst_joint,
               std::vector<std::string>& reasons) {
        if (!enabled_) return false;
        if (is_fallen_) return true;

        float gz = projected_gravity.z();

        tilt_count_ = (gz > tilt_thresh_) ? tilt_count_ + 1 : 0;
        bool flip_now = (gz > flip_tilt_thresh_) && (gyro.norm() > flip_gyro_thresh_);
        flip_count_ = flip_now ? flip_count_ + 1 : 0;
        jspeed_count_ = (jspeed_enabled_ && max_joint_speed > jspeed_thresh_)
                            ? jspeed_count_ + 1 : 0;

        if (tilt_count_ >= debounce_ticks_)
            reasons.push_back("Nghiêng quá mức kéo dài (gz=" + std::to_string(gz) + ")");
        if (flip_count_ >= debounce_ticks_)
            reasons.push_back("Lật nhanh kéo dài (|gyro|=" + std::to_string(gyro.norm()) + ")");
        if (jspeed_count_ >= jspeed_ticks_)
            reasons.push_back("Khớp vung quá nhanh (khớp " + std::to_string(worst_joint) +
                              ", |dq|=" + std::to_string(max_joint_speed) + " rad/s)");

        if (tilt_count_ >= debounce_ticks_ || flip_count_ >= debounce_ticks_ ||
            jspeed_count_ >= jspeed_ticks_) {
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

    bool  jspeed_enabled_ = true;
    float jspeed_thresh_ = 25.0f;
    int   jspeed_ticks_ = 15;

    bool is_fallen_ = false;
    int tilt_count_ = 0;
    int flip_count_ = 0;
    int jspeed_count_ = 0;
};
