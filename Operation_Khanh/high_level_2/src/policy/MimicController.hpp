#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "../motion/MotionData.hpp"
#include "PolicyController.hpp"

// Controller nhảy (Mimic / Dance)
class MimicController : public PolicyController {
public:
    MimicController(const MotionData& motion, int start_frame, float speed = 1.0f,
                    float warmup_s = 1.2f)
        : motion_(motion),
          start_frame_(std::clamp(start_frame, 0, std::max(0, motion.num_frames() - 1))),
          speed_(std::clamp(speed, 0.1f, 2.0f)),
          warmup_s_(std::max(0.0f, warmup_s)) {}

    int ObsSize() const override { return spec::kMimicObsSize; }
    std::string Name() const override { return "Mimic"; }
    // Cờ chưa hoàn thành nếu đang cooldown
    bool IsFinished() const override { return finished_ && !cooling_; }

    // Tư thế bắt đầu clip
    const std::array<float, spec::kNumJoints>& start_pose() const {
        return motion_.joint_pos(start_frame_);
    }

    // Soft-start hoàn thành
    bool WarmupDone() const { return warmup_t_ >= warmup_s_; }

    // Tiến độ soft-start [0,1]
    float WarmupProgress() const {
        return warmup_s_ <= 0.0f ? 1.0f
                                 : std::clamp(warmup_t_ / warmup_s_, 0.0f, 1.0f);
    }

    // Dừng mềm (cooldown)
    void BeginCooldown(const std::array<float, spec::kNumJoints>& stand_q,
                       float cooldown_s, const RobotState& state) {
        if (cooling_) return;
        cooling_ = true;
        cool_s_ = std::max(0.05f, cooldown_s);
        cool_t_ = 0.0f;
        // Tránh giật khớp
        cool_q0_    = last_ref_q_;
        cool_quat0_ = last_ref_quat_;
        cool_goal_q_ = stand_q;
        // Mục tiêu torso thẳng đứng
        cool_quat_goal_ = init_quat_.conjugate() * YawOnly(TorsoQuat(state));
    }

    bool CooldownDone() const { return cooling_ && cool_t_ >= cool_s_; }

    // Bàn giao sớm nếu yên
    bool CooldownReady(float min_s) const { return cooling_ && cool_t_ >= min_s; }

    // Telemetry read-only
    double TelemetryPhase() const { return telemetry_ref_phase_; }
    int TelemetryFrame() const {
        return std::clamp(static_cast<int>(std::floor(telemetry_ref_phase_)), 0,
                          std::max(0, motion_.num_frames() - 1));
    }
    int TelemetryStage() const {
        if (cooling_) return 2;
        if (warmup_t_ < warmup_s_) return 0;
        if (finished_) return 3;
        return 1;
    }
    const std::array<float, spec::kNumJoints>& TelemetryReferenceQ() const {
        return last_ref_q_;
    }
    Eigen::Quaternionf TelemetryReferenceTorsoWorld() const {
        return (init_quat_ * last_ref_quat_).normalized();
    }

    // Tránh chuyển trạng thái sớm khi soft-stop
    void Reset(const RobotState& state) override {
        PolicyController::Reset(state);
        phase_ = static_cast<double>(start_frame_);
        telemetry_ref_phase_ = phase_;
        finished_ = (motion_.num_frames() < 2);
        if (finished_) return;

        // Đồng bộ yaw
        Eigen::Quaternionf robot_yaw = YawOnly(TorsoQuat(state));
        Eigen::Quaternionf ref_yaw = YawOnly(motion_.torso_quat(start_frame_));
        init_quat_ = robot_yaw * ref_yaw.conjugate();

        // Khởi động mềm (soft-start)
        warmup_t_ = 0.0f;
        warm_q0_ = state.q;
        // Bù sai số torso lúc t=0
        warm_quat0_ = init_quat_.conjugate() * TorsoQuat(state);

        cooling_ = false;
        cool_t_  = 0.0f;
        last_ref_q_    = warm_q0_;
        last_ref_quat_ = warm_quat0_;
    }

protected:
    // Xây dựng vector quan sát 129 chiều
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const RobotState& s = ctx.state;
        if (finished_ && !cooling_) return;

        const int N = motion_.num_frames();
        Eigen::Quaternionf ref_torso;

        if (cooling_) {
            telemetry_ref_phase_ = phase_;
            // Cooldown: reference trượt về đứng thẳng
            float u  = std::clamp(cool_t_ / cool_s_, 0.0f, 1.0f);
            float w  = u * u * (3.0f - 2.0f * u);
            float dw = 6.0f * u * (1.0f - u) / cool_s_;

            for (int i = 0; i < spec::kNumJoints; ++i) {
                float d = cool_goal_q_[i] - cool_q0_[i];
                obs[i] = cool_q0_[i] + w * d;
                obs[24 + i] = d * dw;
            }
            ref_torso = cool_quat0_.slerp(w, cool_quat_goal_);
            cool_t_ += spec::kPolicyDt;
        } else if (warmup_t_ < warmup_s_) {
            telemetry_ref_phase_ = phase_;
            // Warmup: reference trượt sang mở màn
            float u  = std::clamp(warmup_t_ / warmup_s_, 0.0f, 1.0f);
            float w  = u * u * (3.0f - 2.0f * u);                 // smoothstep 2 đầu
            float dw = 6.0f * u * (1.0f - u) / warmup_s_;         // Đạo hàm thành vận tốc ref

            const auto& goal = motion_.joint_pos(start_frame_);
            for (int i = 0; i < spec::kNumJoints; ++i) {
                float d = goal[i] - warm_q0_[i];
                obs[i] = warm_q0_[i] + w * d;
                obs[24 + i] = d * dw;
            }
            ref_torso = warm_quat0_.slerp(w, motion_.torso_quat(start_frame_));
            warmup_t_ += spec::kPolicyDt;
        } else {
            // Lưu phase trước khi tăng
            telemetry_ref_phase_ = phase_;
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
            ref_torso = motion_.torso_quat(f).slerp(a, motion_.torso_quat(f + 1));

            phase_ += speed_;
            if (phase_ >= N - 1) finished_ = true;
        }

        // Lưu reference để nối tiếp
        for (int i = 0; i < spec::kNumJoints; ++i) last_ref_q_[i] = obs[i];
        last_ref_quat_ = ref_torso;

        // Tính ma trận xoay tương đối
        Eigen::Quaternionf real_torso = TorsoQuat(s);
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
    double telemetry_ref_phase_ = 0.0;
    bool finished_ = false;
    Eigen::Quaternionf init_quat_ = Eigen::Quaternionf::Identity();

    // Soft-start đưa về tư thế bắt đầu
    float warmup_s_ = 1.2f;
    float warmup_t_ = 0.0f;
    std::array<float, spec::kNumJoints> warm_q0_{};
    Eigen::Quaternionf warm_quat0_ = Eigen::Quaternionf::Identity();

    // Soft-stop đưa về đứng thẳng
    bool  cooling_ = false;
    float cool_s_ = 1.0f;
    float cool_t_ = 0.0f;
    std::array<float, spec::kNumJoints> cool_q0_{};
    std::array<float, spec::kNumJoints> cool_goal_q_{};
    Eigen::Quaternionf cool_quat0_     = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf cool_quat_goal_ = Eigen::Quaternionf::Identity();

    // Reference tick trước tránh nhảy bậc
    std::array<float, spec::kNumJoints> last_ref_q_{};
    Eigen::Quaternionf last_ref_quat_ = Eigen::Quaternionf::Identity();
};
