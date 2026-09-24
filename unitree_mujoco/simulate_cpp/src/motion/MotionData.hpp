#ifndef MOTION_DATA_HPP
#define MOTION_DATA_HPP

#include <array>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "runtime/R1Config.hpp"


// Đọc dữ liệu chuyển động từ file NPZ (joint_pos, joint_vel, body_quat_w)
// dùng cho DanceController. Hỗ trợ float32 và float64.
class MotionData {
public:
    // Nạp NPZ từ đường dẫn. Ném std::runtime_error nếu file hỏng / thiếu key / sai shape.
    void Load(const std::string& npz_path);

    int num_frames() const { return num_frames_; }
    float fps() const { return fps_; }

    const std::array<float, R1Config::NUM_JOINTS>& joint_pos(int frame) const {
        return joint_pos_[static_cast<size_t>(frame)];
    }
    const std::array<float, R1Config::NUM_JOINTS>& joint_vel(int frame) const {
        return joint_vel_[static_cast<size_t>(frame)];
    }
    const Eigen::Quaternionf& torso_quat(int frame) const {
        return torso_quat_w_[static_cast<size_t>(frame)];
    }

    // Quét `search_frames` frame ĐẦU clip, trả về frame ên nhất (vận tốc nhỏ, chuyển động gần default).
    int FindSmoothStartFrame(int search_frames) const;

    int FindTransitionStartFrame(
        int search_frames,
        const std::array<float, R1Config::NUM_JOINTS>& current_q,
        const std::array<float, R1Config::NUM_JOINTS>& current_dq) const;

private:
    int num_frames_ = 0;
    float fps_ = 50.0f;
    std::vector<std::array<float, R1Config::NUM_JOINTS>> joint_pos_;
    std::vector<std::array<float, R1Config::NUM_JOINTS>> joint_vel_;
    std::vector<Eigen::Quaternionf> torso_quat_w_;
    
    static const int kMotionTorsoIdx = 14;
};

#endif // MOTION_DATA_HPP
