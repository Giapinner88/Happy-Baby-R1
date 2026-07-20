#pragma once
/**
 * MotionData.hpp — Nạp motion clip từ file NPZ.
 */

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/RobotSpec.hpp"

class MotionData {
public:
    // Nạp file NPZ. Ném ngoại lệ nếu lỗi.
    void Load(const std::string& npz_path);

    int num_frames() const { return num_frames_; }
    float fps() const { return fps_; }

    const std::array<float, spec::kNumJoints>& joint_pos(int frame) const {
        return joint_pos_[static_cast<size_t>(frame)];
    }
    const std::array<float, spec::kNumJoints>& joint_vel(int frame) const {
        return joint_vel_[static_cast<size_t>(frame)];
    }
    const Eigen::Quaternionf& torso_quat(int frame) const {
        return torso_quat_w_[static_cast<size_t>(frame)];
    }

    // Quét tìm frame êm nhất để bắt đầu nhảy
    int FindSmoothStartFrame(int search_frames) const;

private:
    int num_frames_ = 0;
    float fps_ = 50.0f;
    std::vector<std::array<float, spec::kNumJoints>> joint_pos_;
    std::vector<std::array<float, spec::kNumJoints>> joint_vel_;
    std::vector<Eigen::Quaternionf> torso_quat_w_;
};
