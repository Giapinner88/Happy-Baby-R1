#pragma once

// ============================================================================
// r1_armfree0917_multigait_h4_v2573: runnable multigait for the v2573/v2574
// training family (unitree_rl_mjlab_gait_pro, experiments/
// armfree0917_multigait_run25_v2573_* and _v2574_armtight_*).
//
//   obs     [1, 355] = 332 normalized (4 x 83 term-major H4, oldest -> newest)
//                      + 23 raw FSM conditioning (RAW_LAYOUT of contract.py)
//   actions [1, 25]  = 24 joint offsets + 1 timing command (index 24)
//
// What closes the four gaps listed by MultiGaitH4V2Controller:
//   [2] action 24 drives MultiGaitV257Runtime::AdvanceAction (timing.py Clock);
//       the frame phase is that clock, never the shared HB GaitScheduler.
//   [3] the 8-mode FSM, BRAKE/GATHER/SETTLE, FK stance gates and the IMU/FK stop
//       proxy are gait/MultiGaitV257Runtime.hpp + gait/LegFK.*, the same files
//       that run in simulate_cpp (sim_multigait_v257). Python parity:
//       multigait_v2573_runtime_parity_test (42000-tick trace, 8 modes).
//   [4] there is no HB observation key in these exports, so the identity gate
//       pins each artifact by task_id + source_sha256 + config_sha256 +
//       contract, plus every semantic key the runtime depends on.
//   [1] the exports carry deployable=False (a training-side flag). Selecting
//       one of these sources in a bundle is the explicit decision to run it on
//       hardware for trials; Init prints that decision every start.
//
// Observation uses the RAW gyro and RAW joint velocity, as in training (mjlab,
// no low-pass) and as in simulate_cpp. HB obs83 uses the filtered state, which
// is identical only while imu_gyro_lpf_hz/joint_vel_lpf_hz are 0.
// Gestures and H4 teleop are not supported by this contract.
// ============================================================================

#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../config/RobotSpec.hpp"
#include "../gait/LegFK.hpp"
#include "../gait/MultiGaitV257Runtime.hpp"
#include "BaseObservation83.hpp"
#include "BaseObservationHistory.hpp"
#include "PolicyController.hpp"

class MultiGaitV2573Controller final : public PolicyController {
public:
    static constexpr const char* kProfileContract = "r1_armfree0917_multigait_h4_v2573";
    static constexpr int kNormalizedPrefixDim = r1::history::kH4ActorDim;  // 332
    static constexpr int kConditioningDim = 23;
    static constexpr int kObsSize = kNormalizedPrefixDim + kConditioningDim;  // 355
    static constexpr int kTimingActionIndex = spec::kNumJoints;  // 24

    // Accepted training sources. All share the stopfix stop gates and the
    // v2573 FSM deltas; `anisotropic` selects the elliptic recovery urgency of
    // v2573_push (v2574 resumes from it).
    struct Source {
        const char* task;
        const char* source_sha256;
        const char* config_sha256;
        const char* contract;
        bool anisotropic;
    };
    static constexpr Source kSources[] = {
        {"Unitree-R1-ArmFree0917-MultiGait-Run25-v2573-TurnAbduct-SmoothStand19-2048",
         "a7ffce237fa5fc4cab4dc4738d0b047bfb099a3a3a208fdb145bed518a8b8532",
         "6f9f43eaf2d4a611f537b710f0908653788d0057443514d77ec23f484dd47f7c",
         "r1_armfree0917_multigait_h4_v2573_control_turnabduct_smoothstand19_v1", false},
        {"Unitree-R1-ArmFree0917-MultiGait-Run25-v2573-Push-Run20-4096",
         "dbd6da2bf2f4b23c0efb367939d8e52b945faba704a28f0efbd5b5f013cff24f",
         "4f97857c636ff7cd97dc9ef0fcbb57b68e04ee6cc945451a7678030a235c9cd9",
         "r1_armfree0917_multigait_h4_v2573_control_push_4096_v1", true},
        {"Unitree-R1-ArmFree0917-MultiGait-Run25-v2574-ArmTight-B-28k-2048",
         "1c9b29ce8601ed08b7f5b50ada70559d4a92f66cffd0c01a5929a8b8a8a993d3",
         "28b22c96caa477393ef639d465c1f316c3e33cdca55f687641a467628bf9c022",
         "r1_armfree0917_multigait_h4_v2574_control_armtight_b_28k_2048_v1", true},
        {"Unitree-R1-ArmFree0917-MultiGait-Run25-v2574-ArmTight-C-23k-2048",
         "9358dea4fe1295186ac5a6aa593ffbd40fe367069995dbec437aab61d4b783df",
         "5490bbe6959abc49273c647835ef34dbf8617de3e1304041833388aa849b9ff2",
         "r1_armfree0917_multigait_h4_v2574_control_armtight_c_23k_2048_v1", true},
    };

    std::string Name() const override { return "MultiGaitV2573"; }
    int ObsSize() const override { return kObsSize; }
    int ActionSize() const override { return spec::kNumJoints; }
    int OutputSize() const override { return spec::kNumJoints + 1; }

    void Reset(const RobotState& state) override {
        PolicyController::Reset(state);
        runtime_.Reset();
        history_.Reset();
        ticks_ = 0;
    }

    const MultiGaitV257Runtime& runtime() const { return runtime_; }
    // Observation fed to ONNX on the last Step() (tests and telemetry).
    const std::vector<float>& last_observation() const { return last_obs_; }
    const char* task_id() const { return source_ ? source_->task : ""; }
    // Quiet two-foot STAND with no latched or pending recovery.
    bool StandReady() const {
        return runtime_.mode == MultiGaitV257Runtime::STAND && runtime_.hold &&
               !runtime_.Recovery();
    }

protected:
    void ValidateExtraMetadata(const std::string& model_path) override {
        std::string task;
        policy_.ReadMetadataString("task_id", task);
        source_ = nullptr;
        for (const auto& s : kSources)
            if (task == s.task) source_ = &s;
        if (!source_)
            throw std::runtime_error(
                "ONNX metadata mismatch: task_id '" + (task.empty() ? std::string("<missing>") : task) +
                "' is not a v2573/v2574 multigait source accepted by " + std::string(kProfileContract) +
                " in " + model_path);
        RequireMetadata(model_path, "source_sha256", source_->source_sha256);
        RequireMetadata(model_path, "config_sha256", source_->config_sha256);
        RequireMetadata(model_path, "contract", source_->contract);
        RequireMetadata(model_path, "smoke", "False");
        RequireMetadata(model_path, "actor_dim", std::to_string(kObsSize));
        RequireMetadata(model_path, "action_dim", std::to_string(OutputSize()));
        RequireMetadata(model_path, "normalization_prefix", std::to_string(kNormalizedPrefixDim));
        RequireMetadata(model_path, "timing_action_index", std::to_string(kTimingActionIndex));
        RequireMetadata(model_path, "mode_names", "STAND,WALK,RUN,S2W,W2S,W2R,R2W,TURN");
        RequireMetadata(model_path, "phase_clock", "continuous_latched_action_step");
        RequireMetadata(model_path, "phase_zero_mask", "hold_in_STAND_or_W2S");
        RequireMetadata(model_path, "scheduled_support_semantics",
                        "cyclic_schedule_or_two_foot_hold_not_measured_contact");
        RequireMetadata(model_path, "timing_rule",
                        "walk_log_0.65_1.0_turn_log_0.55_1.0_run_log_0.5_0.7_turn_intent_in_S2W_slew_log_1.5_per_s");
        RequireMetadata(model_path, "recovery_revision",
                        "latched_stop_permission_imu_predict012_exit04_landing_level_expire4s_urg045_v257stopfix");
        RequireMetadata(model_path, "recovery_context_semantics",
                        "index17:latched_0.5+0.5*budget/1.5_else_legacy_budget/1.5");
        RequireMetadata(model_path, "lateral_command_governor",
                        "none_v23_constant_command_cap_0.6_before_acceleration_filter");
        RequireMetadata(model_path, "runtime_stop_gate",
                        "FK_Jdq_gyro_support_persistence_land030_motion030_006s_or022_020s_stand010_not_absolute_no_slip_sensor");
        RequireMetadata(model_path, "balance_revision",
                        "v257stopfix_jacobian_fk_hold030_move030_022_quiet010_motion_urgency035_070_recovery_landing");
        RequireMetadata(model_path, "quiet_conditioning",
                        "STAND_or_W2S_hold_no_recovery_small_scheduled_canonical_W2S_GATHER_progress1");
        RequireMetadata(model_path, "actor_quiet_revision", "W2S_GATHER_alias_including_true_STAND_v1");
        RequireMetadataFloat(model_path, "policy_dt", spec::kPolicyDt);
        RequireMetadata(model_path, "joint_names",
                        "left_hip_pitch_joint,left_hip_roll_joint,left_hip_yaw_joint,"
                        "left_knee_joint,left_ankle_pitch_joint,left_ankle_roll_joint,"
                        "right_hip_pitch_joint,right_hip_roll_joint,right_hip_yaw_joint,"
                        "right_knee_joint,right_ankle_pitch_joint,right_ankle_roll_joint,"
                        "waist_roll_joint,waist_yaw_joint,left_shoulder_pitch_joint,"
                        "left_shoulder_roll_joint,left_shoulder_yaw_joint,left_elbow_joint,"
                        "left_wrist_roll_joint,right_shoulder_pitch_joint,"
                        "right_shoulder_roll_joint,right_shoulder_yaw_joint,"
                        "right_elbow_joint,right_wrist_roll_joint");

        runtime_.gate = MultiGaitV257Runtime::StopGate::StopFix();
        runtime_.variant = MultiGaitV257Runtime::Variant{};
        runtime_.variant.v2573 = true;
        runtime_.variant.anisotropic = source_->anisotropic;
        fk_ = LegFK{};
        fk_.UseNarrowStance190();
        runtime_.Reset();
        history_.Reset();

        std::string deployable, iteration;
        policy_.ReadMetadataString("deployable", deployable);
        policy_.ReadMetadataString("iteration", iteration);
        std::cout << "[" << Name() << "] " << source_->task << " iteration="
                  << (iteration.empty() ? "<missing>" : iteration)
                  << " deployable=" << (deployable.empty() ? "<missing>" : deployable)
                  << " | 355 -> 24 joints + timing, FSM 8 mode, stance 0.190 m"
                  << (source_->anisotropic ? ", anisotropic recovery urgency" : "") << "\n"
                  << "[" << Name() << "] CANH BAO: export tu khai deployable=" << deployable
                  << "; chi bang chung mo phong (simulate_cpp). Chay thu tren gia treo, "
                     "co nguoi giu E-stop.\n";
    }

    void BuildObservation(const ControlContext& ctx, std::vector<float>& obs) override {
        const RobotState& s = ctx.state;
        std::vector<float> frame(r1::history::kBaseDim, 0.0f);
        r1::obs83::Build(ctx, default_q_, last_action_, frame);
        // Training and simulate_cpp use the unfiltered IMU gyro and joint velocity.
        for (int i = 0; i < 3; ++i) frame[r1::obs83::kAngVelOffset + i] = s.gyro_raw[i];
        for (int i = 0; i < spec::kNumJoints; ++i)
            frame[r1::obs83::kJointVelOffset + i] = s.dq_raw[i];

        // FSM inputs: operator command, IMU, absolute joint angles (policy order).
        MultiGaitV257Runtime::Inputs in;
        in.raw = {ctx.cmd_vx, ctx.cmd_vy, ctx.cmd_yaw};
        for (int i = 0; i < 3; ++i) {
            in.gyro[i] = frame[r1::obs83::kAngVelOffset + i];
            in.gravity[i] = frame[r1::obs83::kGravityOffset + i];
        }
        for (int i = 0; i < spec::kNumJoints; ++i) {
            in.q[i] = s.q[i];
            in.dq[i] = frame[r1::obs83::kJointVelOffset + i];
        }
        runtime_.Step(in, fk_);

        // Command and phase terms come from the FSM, not from HB's GaitScheduler.
        for (int i = 0; i < 3; ++i) frame[r1::obs83::kCommandOffset + i] = runtime_.scheduled[i];
        const float angle = 2.0f * static_cast<float>(M_PI) * runtime_.phase;
        frame[r1::obs83::kPhaseOffset + 0] = runtime_.hold ? 0.0f : std::sin(angle);
        frame[r1::obs83::kPhaseOffset + 1] = runtime_.hold ? 0.0f : std::cos(angle);

        history_.Append(frame);
        history_.PackTermMajor(obs);
        const auto conditioning = runtime_.Conditioning();
        obs.insert(obs.end(), conditioning.begin(), conditioning.end());
        if (static_cast<int>(obs.size()) != kObsSize)
            throw std::runtime_error("MultiGaitV2573 built observation size " +
                                     std::to_string(obs.size()));
        last_obs_ = obs;

        if (ticks_++ % 50 == 0) {
            static const char* kModes[] = {"STAND", "WALK", "RUN", "S2W", "W2S", "W2R", "R2W", "TURN"};
            std::cout << "[" << Name() << "] mode=" << kModes[runtime_.mode]
                      << " sub=" << runtime_.substate << " hold=" << runtime_.hold
                      << " T=" << runtime_.period << " cmd=" << runtime_.scheduled[0] << ","
                      << runtime_.scheduled[1] << "," << runtime_.scheduled[2]
                      << " speed_est=" << runtime_.balance.speed
                      << " recovery=" << runtime_.Recovery()
                      << " latched=" << runtime_.recovery_latched << "\n";
        }
    }

    void ComputeTargets(const std::vector<float>& action) override {
        PolicyController::ComputeTargets(action);  // joints 0..23 only
        runtime_.AdvanceAction(action[kTimingActionIndex]);
    }

private:
    const Source* source_ = nullptr;
    MultiGaitV257Runtime runtime_;
    LegFK fk_;
    r1::history::BaseObservationHistory history_{r1::history::kH4Steps};
    std::vector<float> last_obs_;
    unsigned ticks_ = 0;
};
