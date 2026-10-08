#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "../motion/MotionData.hpp"
#include "../motion/MimicTransition.hpp"
#include "PolicyController.hpp"

// Controller thực thi động tác nhảy (Mimic / Dance policy)
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
    // Chưa hoàn thành nếu đang soft-stop (cooldown).
    bool IsFinished() const override { return finished_ && !cooling_; }

    // Tư thế khớp của clip tại frame bắt đầu ("tư thế mở màn")
    const std::array<float, spec::kNumJoints>& start_pose() const {
        return motion_.joint_pos(start_frame_);
    }
    int StartFrame() const { return start_frame_; }

    // Soft-start đã xong -> clip bắt đầu chạy (đây là lúc bật nhạc)
    bool WarmupDone() const { return warmup_t_ >= warmup_s_; }

    // Tiến độ soft-start trong [0,1] (1 = đã xong). Dùng để ramp gain lúc bàn giao
    // locomotion -> mimic: giữ chân cứng như lúc đứng rồi nhả dần về gain policy.
    // Khi đã sang cooldown thì warmup không còn nghĩa -> trả 1 để huỷ bài giữa warmup
    // chạy cùng gain với huỷ sau warmup (trước đây wp đóng băng giữa chừng khiến chân
    // ăn stand gain 200 suốt cả pha thu, gấp đôi kp mà policy được train).
    float WarmupProgress() const {
        if (cooling_) return 1.0f;
        return warmup_s_ <= 0.0f ? 1.0f
                                 : std::clamp(warmup_t_ / warmup_s_, 0.0f, 1.0f);
    }

    // During cooldown the quintic reference is the safety target. The dance
    // actor may not reproduce that stand pose from observation alone, so the
    // application commands this reference directly before locomotion takes over.
    const std::array<float, spec::kNumJoints>& CooldownReferencePosition() const {
        return last_ref_q_;
    }

    // Dừng mềm (soft-stop/cooldown): Chuyển dần từ tư thế nhảy hiện tại về tư thế đứng thẳng dưới policy để giữ thăng bằng trước khi giao cho locomotion.
    void BeginCooldown(const std::array<float, spec::kNumJoints>& stand_q,
                       float cooldown_s, const RobotState& state) {
        if (cooling_) return;
        cooling_ = true;
        cool_s_ = std::max(0.05f, cooldown_s);
        cool_t_ = 0.0f;
        // The actor can lag its reference substantially on the final dance
        // frame. Start the physical cooldown from the measured robot state,
        // otherwise a direct reference target can jump away from the robot at
        // handover.
        cool_q0_    = state.q;
        cool_dq0_   = state.dq;
        cool_quat0_ = last_ref_quat_;
        cool_goal_q_ = stand_q;
        cool_goal_dq_.fill(0.0f);
        // Mục tiêu torso: thẳng đứng, giữ nguyên hướng yaw hiện tại.
        cool_quat_goal_ = init_quat_.conjugate() * YawOnly(TorsoQuat(state));
    }

    bool CooldownDone() const { return cooling_ && cool_t_ >= cool_s_; }
    float CooldownElapsedS() const { return cool_t_; }

    // Bàn giao sớm cho locomotion nếu robot đứng thẳng đủ yên và vượt thời gian tối thiểu.
    bool CooldownReady(float min_s) const { return cooling_ && cool_t_ >= min_s; }

    // Opt-in transition v2. Legacy behavior remains selectable by leaving this
    // disabled in Tuning/config.
    void ConfigureTransitionV2(bool enabled, bool auto_entry_selection,
                               int search_frames, float clip_ramp_s) {
        transition_v2_enabled_ = enabled;
        auto_entry_selection_ = enabled && auto_entry_selection;
        entry_search_frames_ = std::max(1, search_frames);
        clip_ramp_s_ = std::max(0.0f, clip_ramp_s);
    }

    bool TransitionV2Enabled() const { return transition_v2_enabled_; }

    // Tilt/gyro alone are insufficient when the torso is quiet in a squat or
    // asymmetric leg posture. v2 additionally requires the real q/dq to reach
    // the cooldown goal before handing control back to locomotion.
    float CooldownMaxPositionError(const RobotState& state) const {
        float max_pos_err = 0.0f;
        for (int i = 0; i < spec::kNumJoints; ++i) {
            max_pos_err = std::max(max_pos_err,
                                   std::fabs(state.q[static_cast<size_t>(i)] -
                                             cool_goal_q_[static_cast<size_t>(i)]));
        }
        return max_pos_err;
    }

    float CooldownMaxJointSpeed(const RobotState& state) const {
        float max_dq = 0.0f;
        for (const float value : state.dq) max_dq = std::max(max_dq, std::fabs(value));
        return max_dq;
    }

    bool CooldownPoseReady(const RobotState& state, float pos_tol, float dq_tol) const {
        if (!transition_v2_enabled_ || !cooling_) return true;
        return CooldownMaxPositionError(state) <= std::max(0.0f, pos_tol) &&
               CooldownMaxJointSpeed(state) <= std::max(0.0f, dq_tol);
    }

    // Chỉ phục vụ telemetry read-only; không thay đổi phase/controller.
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

    // Đang soft-stop thì chưa coi là xong (tránh Application chuyển trạng thái sớm)
    void Reset(const RobotState& state) override {
        PolicyController::Reset(state);
        if (auto_entry_selection_) {
            start_frame_ = motion_.FindTransitionStartFrame(
                entry_search_frames_, state.q, state.dq);
        }
        phase_ = static_cast<double>(start_frame_);
        telemetry_ref_phase_ = phase_;
        finished_ = (motion_.num_frames() < 2);
        warmup_t_ = 0.0f;
        clip_ramp_t_ = 0.0f;
        cooling_ = false;
        cool_t_  = 0.0f;
        warm_q0_ = state.q;
        warm_dq0_ = state.dq;
        last_ref_q_ = state.q;
        last_ref_dq_ = state.dq;
        if (finished_) return;

        // Đồng bộ hướng yaw của robot với frame bắt đầu trong clip
        Eigen::Quaternionf robot_yaw = YawOnly(TorsoQuat(state));
        Eigen::Quaternionf ref_yaw = YawOnly(motion_.torso_quat(start_frame_));
        init_quat_ = robot_yaw * ref_yaw.conjugate();

        // Khởi động mềm (soft-start): Trượt dần reference từ tư thế hiện tại sang tư thế mở màn dưới policy.
        // Quat sao cho sai số torso ở t=0 bằng 0: init_quat_ * ref0 == torso thật
        warm_quat0_ = init_quat_.conjugate() * TorsoQuat(state);

        last_ref_quat_ = warm_quat0_;
    }

protected:
    // Xây dựng vector quan sát 129 chiều (Mimic Obs) với nội suy theo tốc độ phát speed_
    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const RobotState& s = ctx.state;
        if (finished_ && !cooling_) return;

        const int N = motion_.num_frames();
        Eigen::Quaternionf ref_torso;

        if (cooling_) {
            telemetry_ref_phase_ = phase_;
            // Dừng mềm (cooldown): Reference trượt về tư thế đứng thẳng.
            float u  = std::clamp(cool_t_ / cool_s_, 0.0f, 1.0f);
            if (transition_v2_enabled_) {
                std::array<float, spec::kNumJoints> q{};
                std::array<float, spec::kNumJoints> dq{};
                mimic_transition::QuinticBoundary(
                    cool_q0_, cool_dq0_, cool_goal_q_, cool_goal_dq_,
                    cool_s_, cool_t_, q, dq);
                for (int i = 0; i < spec::kNumJoints; ++i) {
                    obs[i] = q[static_cast<size_t>(i)];
                    obs[24 + i] = dq[static_cast<size_t>(i)];
                }
            } else {
                float w  = u * u * (3.0f - 2.0f * u);
                float dw = 6.0f * u * (1.0f - u) / cool_s_;
                for (int i = 0; i < spec::kNumJoints; ++i) {
                    float d = cool_goal_q_[i] - cool_q0_[i];
                    obs[i] = cool_q0_[i] + w * d;
                    obs[24 + i] = d * dw;
                }
            }
            const float w = transition_v2_enabled_ ? mimic_transition::QuinticEase(u)
                                                   : u * u * (3.0f - 2.0f * u);
            ref_torso = cool_quat0_.slerp(w, cool_quat_goal_);
            cool_t_ += spec::kPolicyDt;
        } else if (warmup_t_ < warmup_s_) {
            telemetry_ref_phase_ = phase_;
            // Khởi động mềm (warmup): Reference trượt sang tư thế mở màn.
            float u  = std::clamp(warmup_t_ / warmup_s_, 0.0f, 1.0f);
            const auto& goal = motion_.joint_pos(start_frame_);
            if (transition_v2_enabled_) {
                std::array<float, spec::kNumJoints> zero_dq{};
                std::array<float, spec::kNumJoints> q{};
                std::array<float, spec::kNumJoints> dq{};
                mimic_transition::QuinticBoundary(
                    warm_q0_, warm_dq0_, goal, zero_dq,
                    warmup_s_, warmup_t_, q, dq);
                for (int i = 0; i < spec::kNumJoints; ++i) {
                    obs[i] = q[static_cast<size_t>(i)];
                    obs[24 + i] = dq[static_cast<size_t>(i)];
                }
            } else {
                float w  = u * u * (3.0f - 2.0f * u);          // smoothstep: êm 2 đầu
                float dw = 6.0f * u * (1.0f - u) / warmup_s_;  // đạo hàm -> vận tốc ref
                for (int i = 0; i < spec::kNumJoints; ++i) {
                    float d = goal[i] - warm_q0_[i];
                    obs[i] = warm_q0_[i] + w * d;
                    obs[24 + i] = d * dw;
                }
            }
            const float w = transition_v2_enabled_ ? mimic_transition::QuinticEase(u)
                                                   : u * u * (3.0f - 2.0f * u);
            ref_torso = warm_quat0_.slerp(w, motion_.torso_quat(start_frame_));
            warmup_t_ += spec::kPolicyDt;
        } else {
            // Ghi phase của reference dùng cho inference tick N trước khi phase_ tăng sang N+1.
            telemetry_ref_phase_ = phase_;
            int f = std::clamp(static_cast<int>(std::floor(phase_)), 0, N - 2);
            float a = static_cast<float>(phase_ - f);

            const auto& mjp0 = motion_.joint_pos(f);
            const auto& mjp1 = motion_.joint_pos(f + 1);
            const auto& mjv0 = motion_.joint_vel(f);
            const auto& mjv1 = motion_.joint_vel(f + 1);
            const float speed_scale = transition_v2_enabled_ && clip_ramp_s_ > 0.0f
                                          ? mimic_transition::QuinticEase(clip_ramp_t_ / clip_ramp_s_)
                                          : 1.0f;
            const float phase_speed = speed_ * speed_scale;
            for (int i = 0; i < spec::kNumJoints; ++i) {
                obs[i] = mjp0[i] + a * (mjp1[i] - mjp0[i]);
                obs[24 + i] = (mjv0[i] + a * (mjv1[i] - mjv0[i])) * phase_speed;
            }
            ref_torso = motion_.torso_quat(f).slerp(a, motion_.torso_quat(f + 1));

            phase_ += phase_speed;
            clip_ramp_t_ += spec::kPolicyDt;
            if (phase_ >= N - 1) finished_ = true;
        }

        // Nhớ reference vừa phát ra, để soft-stop có điểm xuất phát liên tục
        for (int i = 0; i < spec::kNumJoints; ++i) {
            last_ref_q_[i] = obs[i];
            last_ref_dq_[i] = obs[24 + i];
        }
        last_ref_quat_ = ref_torso;

        // Tính ma trận xoay tương đối của torso (motion_anchor_ori_b)
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

    // Soft-start: đưa robot vào tư thế mở màn bằng CHÍNH policy (xem Reset)
    float warmup_s_ = 1.2f;
    float warmup_t_ = 0.0f;
    std::array<float, spec::kNumJoints> warm_q0_{};
    std::array<float, spec::kNumJoints> warm_dq0_{};
    Eigen::Quaternionf warm_quat0_ = Eigen::Quaternionf::Identity();

    // Transition v2 is opt-in to preserve the established controller route.
    bool transition_v2_enabled_ = false;
    bool auto_entry_selection_ = false;
    int entry_search_frames_ = 200;
    float clip_ramp_s_ = 0.35f;
    float clip_ramp_t_ = 0.0f;

    // Soft-stop: đưa robot TỪ tư thế nhảy VỀ đứng thẳng, cũng bằng chính policy
    bool  cooling_ = false;
    float cool_s_ = 1.0f;
    float cool_t_ = 0.0f;
    std::array<float, spec::kNumJoints> cool_q0_{};
    std::array<float, spec::kNumJoints> cool_dq0_{};
    std::array<float, spec::kNumJoints> cool_goal_q_{};
    std::array<float, spec::kNumJoints> cool_goal_dq_{};
    Eigen::Quaternionf cool_quat0_     = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf cool_quat_goal_ = Eigen::Quaternionf::Identity();

    // Reference vừa phát ra ở tick trước — để soft-stop nối tiếp không bị nhảy bậc
    std::array<float, spec::kNumJoints> last_ref_q_{};
    std::array<float, spec::kNumJoints> last_ref_dq_{};
    Eigen::Quaternionf last_ref_quat_ = Eigen::Quaternionf::Identity();
};
