#ifndef DANCE_CONTROLLER_HPP
#define DANCE_CONTROLLER_HPP

#include "runtime/PolicyRunner.hpp"
#include "runtime/R1Config.hpp"
#include "motion/MotionData.hpp"
#include "motion/MimicTransition.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>

class DanceController : public PolicyRunner {
public:
    DanceController(const MotionData& motion, int start_frame, float speed = 1.0f,
                    float warmup_s = 1.5f);
    ~DanceController() override = default;

    void Init(const std::string& model_path, Ort::Env& env, const Ort::SessionOptions& session_options) override;

    std::vector<float> ComputeObservation(
        const LowState_& robot_state,
        const SportModeState_& sport_state,
        float target_vx, float target_vy, float target_yaw,
        float gait_time, const std::array<float, 2>& gait_phase
    ) override;

    std::array<float, R1Config::NUM_JOINTS> ComputeTargetQ(const std::vector<float>& action) override;

    void Reset(const LowState_& current_state) override;
    bool IsFinished() const override;

    // Keep the dance actor active while its reference returns to the standing
    // pose. The application hands control back to locomotion only afterwards.
    void BeginCooldown(const std::array<float, R1Config::NUM_JOINTS>& stand_q,
                       float cooldown_s, const LowState_& state);
    bool CoolingDown() const { return cooling_down_; }
    bool CooldownDone() const { return cooling_down_ && cooldown_elapsed_s_ >= cooldown_s_; }
    float CooldownElapsedS() const { return cooldown_elapsed_s_; }
    float WarmupProgress() const;
    int StartFrame() const { return start_frame_; }
    // During cooldown the quintic reference is the safety target. The dance
    // actor may not reproduce that stand pose from observation alone, so the
    // application can command this reference directly before locomotion takes over.
    const std::array<float, R1Config::NUM_JOINTS>& ReferencePosition() const {
        return last_ref_q_;
    }
    const std::array<float, R1Config::NUM_JOINTS>& CooldownReferencePosition() const {
        return ReferencePosition();
    }

    void ConfigureTransitionV2(bool enabled, bool auto_entry_selection,
                               int search_frames, float clip_ramp_s);
    bool TransitionV2Enabled() const { return transition_v2_enabled_; }
    float CooldownMaxPositionError(const LowState_& state) const;
    float CooldownMaxJointSpeed(const LowState_& state) const;
    bool CooldownPoseReady(const LowState_& state, float pos_tol, float dq_tol) const;

    int GetInputSize() const override { return input_size_; }

private:
    std::array<float, R1Config::NUM_JOINTS> last_action_;
    // Legacy tracking actor has 129 values. PHC recovery appends one current
    // mode bit; simulator dance always runs the normal-tracking branch (0).
    int input_size_ = 129;
    bool phc_recovery_mode_input_ = false;

    const MotionData& motion_;
    int    start_frame_  = 0;
    float  speed_        = 1.0f;
    double phase_        = 0.0;
    bool   is_finished_  = false;
    float  dt_           = 0.02f;

    Eigen::Quaternionf init_quat_ = Eigen::Quaternionf::Identity();

    // Reference-space warmup/cooldown mirrors HB's MimicController. It gives
    // the actor a continuous motion command instead of blending only its output.
    float warmup_s_ = 1.5f;
    float warmup_elapsed_s_ = 0.0f;
    std::array<float, R1Config::NUM_JOINTS> warmup_q0_{};
    std::array<float, R1Config::NUM_JOINTS> warmup_dq0_{};
    Eigen::Quaternionf warmup_quat0_ = Eigen::Quaternionf::Identity();

    bool transition_v2_enabled_ = false;
    bool auto_entry_selection_ = false;
    int entry_search_frames_ = 200;
    float clip_ramp_s_ = 0.35f;
    float clip_ramp_elapsed_s_ = 0.0f;

    bool cooling_down_ = false;
    float cooldown_s_ = 1.0f;
    float cooldown_elapsed_s_ = 0.0f;
    std::array<float, R1Config::NUM_JOINTS> cooldown_q0_{};
    std::array<float, R1Config::NUM_JOINTS> cooldown_dq0_{};
    std::array<float, R1Config::NUM_JOINTS> cooldown_goal_q_{};
    std::array<float, R1Config::NUM_JOINTS> cooldown_goal_dq_{};
    Eigen::Quaternionf cooldown_quat0_ = Eigen::Quaternionf::Identity();
    Eigen::Quaternionf cooldown_goal_quat_ = Eigen::Quaternionf::Identity();

    std::array<float, R1Config::NUM_JOINTS> last_ref_q_{};
    std::array<float, R1Config::NUM_JOINTS> last_ref_dq_{};
    Eigen::Quaternionf last_ref_quat_ = Eigen::Quaternionf::Identity();
};

#endif
