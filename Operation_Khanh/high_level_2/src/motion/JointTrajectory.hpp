#pragma once

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/RobotSpec.hpp"

// Quỹ đạo khớp phát lại trực tiếp bằng PD (không qua policy RL)
class JointTrajectory {
public:
    // Nạp npz: joint_pos (frames, kNumJoints) và fps
    void Load(const std::string& npz_path);

    int num_frames() const { return num_frames_; }
    float fps() const { return fps_; }
    float duration_s() const {
        return num_frames_ > 1 ? static_cast<float>(num_frames_ - 1) / fps_ : 0.0f;
    }

    // Nội suy tuyến tính tại thời điểm t
    std::array<float, spec::kNumJoints> PoseAt(float t) const;

    // Kiểm tra có dữ liệu torso_quat không
    bool has_torso() const { return has_torso_; }
    // Trọng lực chiếu vào thân tham chiếu tại t
    Eigen::Vector3f RefGravityAt(float t) const;

private:
    int num_frames_ = 0;
    float fps_ = 50.0f;
    bool has_torso_ = false;
    std::vector<std::array<float, spec::kNumJoints>> joint_pos_;
    std::vector<Eigen::Vector3f> ref_grav_;   // projected gravity mỗi frame
};
