#pragma once

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/RobotSpec.hpp"

// Lớp nạp và lưu trữ dữ liệu chuyển động từ file NPZ
class MotionData {
public:
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

    // Tìm frame bắt đầu êm nhất trong N frame đầu (giảm thiểu giật chân/torso)
    int FindSmoothStartFrame(int search_frames) const;

    // Chọn frame vào clip theo trạng thái thực tế ngay trước khi ActivatePolicy.
    // Khác với FindSmoothStartFrame(), hàm này phạt cả q/dq lệch default, vận tốc
    // chân trong vài frame kế tiếp, bất đối xứng và cổ chân không phẳng.
    int FindTransitionStartFrame(
        int search_frames,
        const std::array<float, spec::kNumJoints>& current_q,
        const std::array<float, spec::kNumJoints>& current_dq) const;

private:
    void LoadValidated(const std::string& npz_path);
    int num_frames_ = 0;
    float fps_ = 50.0f;
    std::vector<std::array<float, spec::kNumJoints>> joint_pos_;
    std::vector<std::array<float, spec::kNumJoints>> joint_vel_;
    std::vector<Eigen::Quaternionf> torso_quat_w_;
};
