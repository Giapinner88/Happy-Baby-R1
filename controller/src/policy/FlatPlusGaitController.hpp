#pragma once

#include <array>
#include <cmath>

#include "FlatHistoryController.hpp"
#include "../gait/GaitModeScheduler.hpp"
#include "../gait/GaitScheduler.hpp"

// H4 history plus current STAND/WALK/W2S one-hot: 332 + 3 = 335D.
class FlatPlusGaitController final : public FlatHistoryController {
public:
    static constexpr int kObsSize = r1::history::kH4ActorDim + 3;

    FlatPlusGaitController()
        : FlatHistoryController(r1::history::kH4Steps,
                                "flat_plus_gait_h4_v1") {}
    ~FlatPlusGaitController() override = default;

    std::string Name() const override { return "FlatPlusGaitH4"; }
    bool SupportsHistoryGestureOverlay() const override { return true; }
    bool SupportsH4TeleopOverlay() const override { return true; }

    void ConfigureGait(const GaitModeConfig& config,
                       float gait_period_s = spec::kGaitPeriodS) {
        gait_config_ = config;
        gait_.Configure(config);
        gait_phase_.Configure(gait_period_s);
    }

    const GaitModeScheduler& gait() const { return gait_; }
    bool stand_recovery_enabled() const { return stand_recovery_enabled_; }

protected:
    int ActorExtraDim() const override { return 3; }

    void ResetVariant() override {
        gait_.Reset();
        gait_phase_.Reset();
    }

    void PrepareObservationContext(ControlContext& ctx) override {
        const float command_linear_norm =
            std::hypot(ctx.cmd_vx, ctx.cmd_vy);
        const float command_yaw_abs = std::abs(ctx.cmd_yaw);
        const std::array<float, 3> gyro = {
            ctx.state.gyro_raw.x(), ctx.state.gyro_raw.y(), ctx.state.gyro_raw.z()};
        // Feed raw sensors to this contract's own signed-vector stability filter;
        // actor observation still uses the existing HB filtered state in obs83.
        const GaitMode previous_mode = gait_.mode();
        if (stand_recovery_enabled_) {
            const std::array<float, 3> projected_gravity = {
                ctx.state.projected_gravity.x(), ctx.state.projected_gravity.y(),
                ctx.state.projected_gravity.z()};
            gait_.Update(command_linear_norm, command_yaw_abs, gyro,
                         ctx.state.dq_raw, projected_gravity, spec::kPolicyDt);
            const bool previous_stand = previous_mode == GaitMode::Stand;
            const bool current_stand = gait_.mode() == GaitMode::Stand;
            gait_phase_.UpdateStandRecovery(
                previous_stand, current_stand, spec::kPolicyDt);
            ctx.gait_phase = gait_phase_.PhaseObsStandRecovery(current_stand);
        } else {
            gait_.Update(command_linear_norm, command_yaw_abs, gyro,
                         ctx.state.dq_raw, spec::kPolicyDt);
        }
    }

    void AppendActorExtras(const ControlContext& /*ctx*/,
                           std::vector<float>& obs) override {
        const auto one_hot = gait_.OneHot();
        obs.insert(obs.end(), one_hot.begin(), one_hot.end());
    }

    void ValidateVariantMetadata() override {
        // Metadata, not the shared 335-D profile name, opts into recovery.
        stand_recovery_enabled_ = false;
        gait_config_.stand_upright_enabled = false;
        gait_config_.stand_push_exit_enabled = false;
        RequireMetadata(model_path_, "actor_obs_groups", "actor,gait");
        RequireMetadata(model_path_, "actor_obs_group_dims", "332,3");
        RequireMetadata(model_path_, "actor_normalized_prefix_dim", "332");
        RequireMetadata(model_path_, "gait_dim", "3");
        RequireMetadata(model_path_, "gait_encoding", "one_hot_exactly_one");
        RequireMetadata(model_path_, "gait_modes", "STAND,WALK,W2S");
        RequireMetadata(model_path_, "gait_normalization", "none_raw_onehot");
        RequireMetadata(model_path_, "normalizer_contract",
                        "prefix_only_then_raw_gait_v1");
        RequireMetadata(model_path_, "onnx_preprocess_contract",
                        "normalize_prefix_then_append_raw_gait_v1");
        // The observation/controller contract is shared by the original gait
        // artifacts and the 0917 exports. Keep the FSM check exact, but qualify
        // each stand-recovery task variant explicitly.
        std::string gait_fsm;
        if (!policy_.ReadMetadataString("gait_fsm_contract", gait_fsm) ||
            (gait_fsm != "flat_plus_gait_fsm_v1" &&
             gait_fsm != "flat_plus_gait_fsm_v1_stand_recovery_v1")) {
            throw std::runtime_error(
                "ONNX metadata mismatch: gait_fsm_contract expected one of "
                "'flat_plus_gait_fsm_v1', "
                "'flat_plus_gait_fsm_v1_stand_recovery_v1', model has '" +
                (gait_fsm.empty() ? std::string("<missing>") : gait_fsm) +
                "' in " + model_path_);
        }
        if (gait_fsm == "flat_plus_gait_fsm_v1_stand_recovery_v1") {
            std::string task_variant;
            if (!policy_.ReadMetadataString("gait_task_variant", task_variant) ||
                (task_variant != "0917" && task_variant != "0917V3" &&
                 task_variant != "0917V4" && task_variant != "0917V5A1" &&
                 task_variant != "0917V5A2A")) {
                throw std::runtime_error(
                    "ONNX metadata mismatch: gait_task_variant expected '0917' "
                    "or '0917V3'/'0917V4'/'0917V5A1'/'0917V5A2A' "
                    "for stand-recovery FSM, model has '" +
                    (task_variant.empty() ? std::string("<missing>") : task_variant) +
                    "' in " + model_path_);
            }
            if (task_variant == "0917V5A1" || task_variant == "0917V5A2A") {
                RequireMetadata(model_path_, "gait_v5_ablation", task_variant);
                RequireMetadata(model_path_, "gait_reward_overlay",
                                "0917_v5_versioned_ablation");
                RequireMetadata(model_path_, "gait_walk_push_matrix",
                                "direction_x_phase8_x_intensity3");
                RequireMetadata(model_path_, "gait_recovery_metrics",
                                "touchdown_displacement,time_to_touchdown,cp_margin");
            }
            RequireMetadata(model_path_, "gait_command_overlay",
                            "legacy_0909_v2_stand_recovery_v1");
            RequireMetadataFloat(model_path_, "gait_stand_upright_enabled", 1.0f);
            RequireMetadataFloat(model_path_, "gait_stand_entry_tilt_max_rad",
                                 gait_config_.stand_entry_tilt_max_rad);
            RequireMetadataFloat(model_path_, "gait_stand_push_exit_enabled", 1.0f);
            RequireMetadataFloat(model_path_, "gait_push_exit_tilt_immediate_rad",
                                 gait_config_.push_exit_tilt_immediate_rad);
            RequireMetadataFloat(model_path_, "gait_push_exit_tilt_sustained_rad",
                                 gait_config_.push_exit_tilt_sustained_rad);
            RequireMetadataFloat(model_path_, "gait_push_exit_gyro_immediate",
                                 gait_config_.push_exit_gyro_immediate);
            RequireMetadataFloat(model_path_, "gait_push_exit_gyro_sustained",
                                 gait_config_.push_exit_gyro_sustained);
            RequireMetadataFloat(model_path_, "gait_push_exit_sustain_s",
                                 gait_config_.push_exit_sustain_s);
            gait_config_.stand_upright_enabled = true;
            gait_config_.stand_push_exit_enabled = true;
            stand_recovery_enabled_ = true;
        }
        RequireMetadata(model_path_, "gait_stability_predicate",
                        "filtered_imu_ang_vel_norm_and_joint_vel_rms_v2");

        RequireMetadataFloat(model_path_, "gait_stop_request_lin",
                             gait_config_.stop_request_lin);
        RequireMetadataFloat(model_path_, "gait_stop_request_ang",
                             gait_config_.stop_request_ang);
        RequireMetadataFloat(model_path_, "gait_move_request_lin",
                             gait_config_.move_request_lin);
        RequireMetadataFloat(model_path_, "gait_move_request_ang",
                             gait_config_.move_request_ang);
        // SETTLE dwell is a runtime tuning, not an actor/observation contract.
        // Keep the FSM duration controlled by gait_settle_dwell_s in YAML;
        // exported gait_t_settle_s metadata may describe the training setup.
        RequireMetadataFloat(model_path_, "gait_w2s_timeout_s",
                             gait_config_.w2s_timeout_s);
        RequireMetadataFloat(model_path_, "gait_stability_ang_vel_max",
                             gait_config_.settle_gyro_norm);
        RequireMetadataFloat(model_path_, "gait_stability_joint_vel_rms_max",
                             gait_config_.settle_joint_velocity_rms);

        if (std::abs(gait_config_.stability_filter_tau_s - 0.20f) > 1.0e-6f) {
            throw std::runtime_error(
                "gait stability filter tau must remain 0.20 s for "
                "flat_plus_gait_h4_v1");
        }
        gait_.Configure(gait_config_);
    }

private:
    GaitModeConfig gait_config_{};
    GaitModeScheduler gait_{};
    GaitScheduler gait_phase_{};
    bool stand_recovery_enabled_ = false;
};
