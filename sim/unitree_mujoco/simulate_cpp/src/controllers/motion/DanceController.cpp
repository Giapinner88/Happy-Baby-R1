#include "DanceController.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

static Eigen::Quaternionf yawQuaternion(const Eigen::Quaternionf& q) {
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    float half_yaw = yaw * 0.5f;
    return Eigen::Quaternionf(std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)).normalized();
}

static float smoothstep(float value) {
    const float t = std::clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static Eigen::Quaternionf torsoQuaternion(const LowState_& state) {
    const Eigen::Quaternionf root_quat(
        state.imu_state().quaternion()[0],
        state.imu_state().quaternion()[1],
        state.imu_state().quaternion()[2],
        state.imu_state().quaternion()[3]);
    const float waist_roll = state.motor_state()[
        R1Config::PolicyToIdl(R1Config::WAIST_ROLL_POLICY_INDEX)].q();
    const float waist_yaw = state.motor_state()[
        R1Config::PolicyToIdl(R1Config::WAIST_YAW_POLICY_INDEX)].q();
    return root_quat
        * Eigen::AngleAxisf(waist_roll, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(waist_yaw, Eigen::Vector3f::UnitZ());
}

DanceController::DanceController(const MotionData& motion, int start_frame, float speed,
                                 float warmup_s)
    : motion_(motion),
      start_frame_(std::clamp(start_frame, 0, std::max(0, motion.num_frames() - 1))),
      speed_(std::clamp(speed, 0.1f, 2.0f)),
      warmup_s_(std::max(0.0f, warmup_s))
{
    last_action_.fill(0.0f);
    dt_ = 0.02f;
    is_finished_ = (motion_.num_frames() == 0);
}

void DanceController::Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) {
    OpenModel(model_path, env, session_options);
    const auto [input_size, output_size] = SingleModelShape();
    if ((input_size != 129 && input_size != 130)
        || output_size != R1Config::NUM_JOINTS) {
        throw std::runtime_error(
            "dance ONNX contract mismatch; expected one input of 129 or 130 and output=24");
    }
    input_size_ = static_cast<int>(input_size);
    phc_recovery_mode_input_ = input_size_ == 130;
    default_q_ = R1Config::DEFAULT_JOINT_POS;
    action_scale_ = R1Config::ACTION_SCALE;
    joint_stiffness_ = R1Config::KP_ARRAY;
    joint_damping_ = R1Config::KD_ARRAY;
    LoadMetadata();

    std::cout << "[DanceController] Đã nạp thành công mô hình: " << model_path << " (speed=" << speed_ << ")\n";
}

void DanceController::Reset(const LowState_& current_state) {
    if (auto_entry_selection_) {
        std::array<float, R1Config::NUM_JOINTS> current_q{};
        std::array<float, R1Config::NUM_JOINTS> current_dq{};
        for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
            const int idl_idx = R1Config::PolicyToIdl(i);
            current_q[static_cast<size_t>(i)] = current_state.motor_state()[idl_idx].q();
            current_dq[static_cast<size_t>(i)] = current_state.motor_state()[idl_idx].dq();
        }
        start_frame_ = motion_.FindTransitionStartFrame(
            entry_search_frames_, current_q, current_dq);
    }
    phase_ = static_cast<double>(start_frame_);
    is_finished_ = (motion_.num_frames() < 2);
    last_action_.fill(0.0f);
    warmup_elapsed_s_ = 0.0f;
    clip_ramp_elapsed_s_ = 0.0f;
    cooling_down_ = false;
    cooldown_elapsed_s_ = 0.0f;

    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        const int idl_idx = R1Config::PolicyToIdl(i);
        warmup_q0_[i] = current_state.motor_state()[idl_idx].q();
        warmup_dq0_[i] = current_state.motor_state()[idl_idx].dq();
        last_ref_q_[i] = warmup_q0_[i];
        last_ref_dq_[i] = warmup_dq0_[i];
    }

    if (!is_finished_) {
        const Eigen::Quaternionf robot_torso = torsoQuaternion(current_state);
        const Eigen::Quaternionf robot_yaw = yawQuaternion(robot_torso);
        const Eigen::Quaternionf ref_yaw = yawQuaternion(motion_.torso_quat(start_frame_));

        init_quat_ = robot_yaw * ref_yaw.conjugate();
        warmup_quat0_ = init_quat_.conjugate() * robot_torso;
        last_ref_quat_ = warmup_quat0_;
    } else {
        init_quat_ = Eigen::Quaternionf::Identity();
        warmup_quat0_ = Eigen::Quaternionf::Identity();
        last_ref_quat_ = warmup_quat0_;
    }
}

void DanceController::ConfigureTransitionV2(bool enabled, bool auto_entry_selection,
                                             int search_frames, float clip_ramp_s) {
    transition_v2_enabled_ = enabled;
    auto_entry_selection_ = enabled && auto_entry_selection;
    entry_search_frames_ = std::max(1, search_frames);
    clip_ramp_s_ = std::max(0.0f, clip_ramp_s);
}

bool DanceController::CooldownPoseReady(const LowState_& state,
                                        float pos_tol, float dq_tol) const {
    if (!transition_v2_enabled_ || !cooling_down_) return true;
    return CooldownMaxPositionError(state) <= std::max(0.0f, pos_tol) &&
           CooldownMaxJointSpeed(state) <= std::max(0.0f, dq_tol);
}

float DanceController::CooldownMaxPositionError(const LowState_& state) const {
    float max_pos_err = 0.0f;
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        const int idl_idx = R1Config::PolicyToIdl(i);
        max_pos_err = std::max(max_pos_err, std::fabs(
            state.motor_state()[idl_idx].q() - cooldown_goal_q_[static_cast<size_t>(i)]));
    }
    return max_pos_err;
}

float DanceController::CooldownMaxJointSpeed(const LowState_& state) const {
    float max_dq = 0.0f;
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        const int idl_idx = R1Config::PolicyToIdl(i);
        max_dq = std::max(max_dq, std::fabs(state.motor_state()[idl_idx].dq()));
    }
    return max_dq;
}

bool DanceController::IsFinished() const {
    return is_finished_ && !cooling_down_;
}

float DanceController::WarmupProgress() const {
    if (cooling_down_ || warmup_s_ <= 0.0f) return 1.0f;
    return std::clamp(warmup_elapsed_s_ / warmup_s_, 0.0f, 1.0f);
}

void DanceController::BeginCooldown(
    const std::array<float, R1Config::NUM_JOINTS>& stand_q,
    float cooldown_s, const LowState_& state) {
    if (cooling_down_) return;
    cooling_down_ = true;
    cooldown_s_ = std::max(0.05f, cooldown_s);
    cooldown_elapsed_s_ = 0.0f;
    // The actor can lag its reference substantially on the final dance frame.
    // Start the physical cooldown from the measured robot state, otherwise a
    // direct reference target can jump away from the robot at handover.
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        const int idl_idx = R1Config::PolicyToIdl(i);
        cooldown_q0_[i] = state.motor_state()[idl_idx].q();
        cooldown_dq0_[i] = state.motor_state()[idl_idx].dq();
    }
    cooldown_goal_q_ = stand_q;
    cooldown_goal_dq_.fill(0.0f);
    cooldown_quat0_ = last_ref_quat_;
    cooldown_goal_quat_ = init_quat_.conjugate() * yawQuaternion(torsoQuaternion(state));
}

std::vector<float> DanceController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) 
{
    std::vector<float> obs(GetInputSize(), 0.0f);
    if (is_finished_ && !cooling_down_) return obs;

    const int N = motion_.num_frames();
    Eigen::Quaternionf ref_torso = Eigen::Quaternionf::Identity();
    if (cooling_down_) {
        const float u = std::clamp(cooldown_elapsed_s_ / cooldown_s_, 0.0f, 1.0f);
        const float weight = transition_v2_enabled_ ? mimic_transition::QuinticEase(u)
                                                     : smoothstep(u);
        if (transition_v2_enabled_) {
            std::array<float, R1Config::NUM_JOINTS> q{};
            std::array<float, R1Config::NUM_JOINTS> dq{};
            mimic_transition::QuinticBoundary(
                cooldown_q0_, cooldown_dq0_, cooldown_goal_q_, cooldown_goal_dq_,
                cooldown_s_, cooldown_elapsed_s_, q, dq);
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                obs[i] = q[static_cast<size_t>(i)];
                obs[24 + i] = dq[static_cast<size_t>(i)];
            }
        } else {
            const float velocity_scale = 6.0f * u * (1.0f - u) / cooldown_s_;
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                const float delta = cooldown_goal_q_[i] - cooldown_q0_[i];
                obs[i] = cooldown_q0_[i] + weight * delta;
                obs[24 + i] = velocity_scale * delta;
            }
        }
        ref_torso = cooldown_quat0_.slerp(weight, cooldown_goal_quat_);
        cooldown_elapsed_s_ += dt_;
    } else if (warmup_elapsed_s_ < warmup_s_) {
        const float u = warmup_s_ <= 0.0f ? 1.0f
                                           : std::clamp(warmup_elapsed_s_ / warmup_s_, 0.0f, 1.0f);
        const auto& goal = motion_.joint_pos(start_frame_);
        const float weight = transition_v2_enabled_ ? mimic_transition::QuinticEase(u)
                                                     : smoothstep(u);
        if (transition_v2_enabled_) {
            std::array<float, R1Config::NUM_JOINTS> zero_dq{};
            std::array<float, R1Config::NUM_JOINTS> q{};
            std::array<float, R1Config::NUM_JOINTS> dq{};
            mimic_transition::QuinticBoundary(
                warmup_q0_, warmup_dq0_, goal, zero_dq,
                warmup_s_, warmup_elapsed_s_, q, dq);
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                obs[i] = q[static_cast<size_t>(i)];
                obs[24 + i] = dq[static_cast<size_t>(i)];
            }
        } else {
            const float velocity_scale = warmup_s_ <= 0.0f ? 0.0f
                                                             : 6.0f * u * (1.0f - u) / warmup_s_;
            for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
                const float delta = goal[i] - warmup_q0_[i];
                obs[i] = warmup_q0_[i] + weight * delta;
                obs[24 + i] = velocity_scale * delta;
            }
        }
        ref_torso = warmup_quat0_.slerp(weight, motion_.torso_quat(start_frame_));
        warmup_elapsed_s_ += dt_;
    } else {
        const int frame = std::clamp(static_cast<int>(std::floor(phase_)), 0, N - 2);
        const float fraction = static_cast<float>(phase_ - frame);
        const auto& mjp0 = motion_.joint_pos(frame);
        const auto& mjp1 = motion_.joint_pos(frame + 1);
        const auto& mjv0 = motion_.joint_vel(frame);
        const auto& mjv1 = motion_.joint_vel(frame + 1);
        const float speed_scale = transition_v2_enabled_ && clip_ramp_s_ > 0.0f
                                      ? mimic_transition::QuinticEase(clip_ramp_elapsed_s_ / clip_ramp_s_)
                                      : 1.0f;
        const float phase_speed = speed_ * speed_scale;
        for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
            obs[i] = mjp0[i] + fraction * (mjp1[i] - mjp0[i]);
            obs[24 + i] = (mjv0[i] + fraction * (mjv1[i] - mjv0[i])) * phase_speed;
        }
        ref_torso = motion_.torso_quat(frame).slerp(fraction, motion_.torso_quat(frame + 1));
        phase_ += phase_speed;
        clip_ramp_elapsed_s_ += dt_;
        if (phase_ >= N - 1) is_finished_ = true;
    }

    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        last_ref_q_[i] = obs[i];
        last_ref_dq_[i] = obs[24 + i];
    }
    last_ref_quat_ = ref_torso;

    // 2. motion_anchor_ori_b (6)
    const Eigen::Quaternionf real_torso = torsoQuaternion(robot_state);
    const Eigen::Quaternionf rot_q = (init_quat_ * ref_torso).conjugate() * real_torso;
    Eigen::Matrix3f rot = rot_q.toRotationMatrix().transpose();
    
    obs[48] = rot(0, 0);
    obs[49] = rot(0, 1);
    obs[50] = rot(1, 0);
    obs[51] = rot(1, 1);
    obs[52] = rot(2, 0);
    obs[53] = rot(2, 1);

    // 3. Gyroscope (3)
    obs[54] = robot_state.imu_state().gyroscope()[0];
    obs[55] = robot_state.imu_state().gyroscope()[1];
    obs[56] = robot_state.imu_state().gyroscope()[2];

    // 4. q_rel (24) and dq (24)
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        int idl_idx = R1Config::PolicyToIdl(i);
        float q_real = robot_state.motor_state()[idl_idx].q();
        float dq_real = robot_state.motor_state()[idl_idx].dq();

        obs[57 + i] = q_real - default_q_[i];
        obs[81 + i] = dq_real;
    }

    // 5. Last action (24)
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        obs[105 + i] = last_action_[i];
    }
    if (phc_recovery_mode_input_) {
        // PHC's extra actor term is 1 only during its learned recovery task.
        // This simulator has no compatible recovery state estimator, so normal
        // reference tracking explicitly selects the 0 branch.
        obs[129] = 0.0f;
    }

    return obs;
}

std::array<float, R1Config::NUM_JOINTS> DanceController::ComputeTargetQ(const std::vector<float>& action) {
    std::array<float, R1Config::NUM_JOINTS> target_q;
    for (int i = 0; i < R1Config::NUM_JOINTS; ++i) {
        last_action_[i] = action[i];
        target_q[i] = default_q_[i] + action[i] * action_scale_[i];
    }
    return target_q;
}
