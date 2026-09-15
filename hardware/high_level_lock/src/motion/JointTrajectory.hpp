#pragma once

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "../config/RobotSpec.hpp"

// Quỹ đạo khớp thuần (không quat/vel) dùng để phát lại trực tiếp bằng PD, KHÔNG
// qua policy RL — dùng cho đứng lên / nằm xuống (ghi bằng tools/record_motion.cpp).
// Tách riêng khỏi MotionData (dance/mimic) vì không cần body_quat_w/joint_vel cho
// obs của policy ONNX.
class JointTrajectory {
public:
    // NPZ cần key "joint_pos" (frames, kNumJoints); key "fps" (double) tùy chọn,
    // mặc định 50.0 nếu thiếu. "head_pos" (frames, 2: yaw,pitch) là tùy chọn;
    // nó nằm ngoài policy 24-D và chỉ được gesture playback dùng.
    void Load(const std::string& npz_path);

    int num_frames() const { return num_frames_; }
    float fps() const { return fps_; }
    float duration_s() const {
        return num_frames_ > 1 ? static_cast<float>(num_frames_ - 1) / fps_ : 0.0f;
    }

    // Nội suy tuyến tính giữa 2 frame gần nhất theo thời gian t (giây, kể từ frame 0).
    // t ngoài khoảng [0, duration_s()] bị kẹp về 2 đầu.
    std::array<float, spec::kNumJoints> PoseAt(float t) const;

    // Truy cập frame thô (dùng cho GesturePlayer trích cột tay).
    const std::array<float, spec::kNumJoints>& frame(int i) const {
        return joint_pos_[static_cast<size_t>(i)];
    }

    bool has_head() const { return has_head_; }
    const std::array<float, 2>& head_frame(int i) const {
        return head_pos_[static_cast<size_t>(i)];
    }

    // Có dữ liệu hướng thân (torso_quat) để bù cổ chân không (file cũ thì không).
    bool has_torso() const { return has_torso_; }
    // Trọng lực chiếu vào thân (projected gravity) tại thời điểm t — tư thế thân LÚC GHI.
    // Dùng để so với gravity ĐO ĐƯỢC lúc phát lại -> bù cổ chân giữ bàn chân phẳng với sàn.
    Eigen::Vector3f RefGravityAt(float t) const;

private:
    int num_frames_ = 0;
    float fps_ = 50.0f;
    bool has_torso_ = false;
    bool has_head_ = false;
    std::vector<std::array<float, spec::kNumJoints>> joint_pos_;
    std::vector<std::array<float, 2>> head_pos_;
    std::vector<Eigen::Vector3f> ref_grav_;   // projected gravity mỗi frame (từ torso_quat)
};
