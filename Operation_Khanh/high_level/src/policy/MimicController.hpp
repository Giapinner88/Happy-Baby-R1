#pragma once
/**
 * MimicController.hpp — Policy mimic (dance).
 */

#include <algorithm>
#include <cmath>

#include "../motion/MotionData.hpp"
#include "PolicyController.hpp"

class MimicController : public PolicyController {
public:
    // Khởi tạo
    MimicController(const MotionData& motion, int start_frame, float speed = 1.0f)
        : motion_(motion),
          start_frame_(std::clamp(start_frame, 0, std::max(0, motion.num_frames() - 1))),
          speed_(std::clamp(speed, 0.1f, 2.0f)) {}

    int ObsSize() const override { return spec::kMimicObsSize; }
    std::string Name() const override { return "Mimic"; }
    bool IsFinished() const override { return finished_; }

    void Reset(const RobotState& state) override {
        PolicyController::Reset(state);
        phase_ = static_cast<double>(start_frame_);
        finished_ = (motion_.num_frames() < 2);
        if (finished_) return;

        // Align yaw hệ quy chiếu
        Eigen::Quaternionf robot_yaw = YawOnly(TorsoQuat(state));
        Eigen::Quaternionf ref_yaw = YawOnly(motion_.torso_quat(start_frame_));
        init_quat_ = robot_yaw * ref_yaw.conjugate();
    }

protected:
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const RobotState& s = ctx.state;
        if (finished_) return;

        // Nội suy con trỏ frame
        const int N = motion_.num_frames();
        int f = std::clamp(static_cast<int>(std::floor(phase_)), 0, N - 2);
        float a = static_cast<float>(phase_ - f);

        const auto& mjp0 = motion_.joint_pos(f);
        const auto& mjp1 = motion_.joint_pos(f + 1);
        const auto& mjv0 = motion_.joint_vel(f);
        const auto& mjv1 = motion_.joint_vel(f + 1);
        for (int i = 0; i < spec::kNumJoints; ++i) {
            obs[i] = mjp0[i] + a * (mjp1[i] - mjp0[i]);
            obs[24 + i] = (mjv0[i] + a * (mjv1[i] - mjv0[i])) * speed_;
        }

        // Cập nhật ma trận rot từ quaternion
        Eigen::Quaternionf real_torso = TorsoQuat(s);
        Eigen::Quaternionf ref_torso = motion_.torso_quat(f).slerp(a, motion_.torso_quat(f + 1));
        Eigen::Quaternionf rot_q = (init_quat_ * ref_torso).conjugate() * real_torso;
        Eigen::Matrix3f rot = rot_q.toRotationMatrix().transpose();
        obs[48] = rot(0, 0);
        obs[49] = rot(0, 1);
        obs[50] = rot(1, 0);
        obs[51] = rot(1, 1);
        obs[52] = rot(2, 0);
        obs[53] = rot(2, 1);

        obs[54] = s.gyro.x();
        obs[55] = s.gyro.y();
        obs[56] = s.gyro.z();

        for (int i = 0; i < spec::kNumJoints; ++i) {
            obs[57 + i] = s.q[i] - default_q_[i];
            obs[81 + i] = s.dq[i];
            obs[105 + i] = last_action_[i];
        }

        // Cập nhật frame phase
        phase_ += speed_;
        if (phase_ >= N - 1) finished_ = true;
    }

private:
    Eigen::Quaternionf TorsoQuat(const RobotState& s) const {
        return s.quat *
               Eigen::Quaternionf(Eigen::AngleAxisf(s.waist_roll(), Eigen::Vector3f::UnitX())) *
               Eigen::Quaternionf(Eigen::AngleAxisf(s.waist_yaw(), Eigen::Vector3f::UnitZ()));
    }

    static Eigen::Quaternionf YawOnly(const Eigen::Quaternionf& q) {
        float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                               1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
        return Eigen::Quaternionf(std::cos(yaw * 0.5f), 0.0f, 0.0f,
                                  std::sin(yaw * 0.5f)).normalized();
    }

    const MotionData& motion_;
    int start_frame_ = 0;
    float speed_ = 1.0f;
    double phase_ = 0.0;
    bool finished_ = false;
    Eigen::Quaternionf init_quat_ = Eigen::Quaternionf::Identity();
};
