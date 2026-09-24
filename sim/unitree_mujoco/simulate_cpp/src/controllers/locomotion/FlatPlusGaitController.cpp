#include "FlatPlusGaitController.hpp"

#include <cmath>
#include <stdexcept>

std::vector<float> FlatPlusGaitController::ComputeObservation(
    const LowState_& robot_state,
    const SportModeState_& sport_state,
    float target_vx, float target_vy, float target_yaw,
    float gait_time, const std::array<float, 2>& gait_phase) {
    auto observation = BuildHistoryObservation(
        robot_state, sport_state, target_vx, target_vy, target_yaw,
        gait_time, gait_phase);
    const auto& mode = CurrentGaitMode();
    observation.insert(observation.end(), mode.begin(), mode.end());
    return observation;
}

void FlatPlusGaitController::ValidateVariantMetadata() const {
    const auto require = [&](const char* key, const std::string& expected) {
        const auto actual = ModelMetadata(key);
        if (actual != expected) {
            throw std::runtime_error(
                std::string("gait policy metadata mismatch: ") + key
                + " expected '" + expected + "', model has '"
                + (actual.empty() ? "<missing>" : actual) + "'");
        }
    };
    // These names are the canonical training/export contract.  The previous
    // runtime expected gait_mode_* aliases that were never emitted by the
    // trained artifact, so a valid 335-D model was rejected before inference.
    require("actor_obs_groups", "actor,gait");
    require("actor_obs_group_dims", "332,3");
    require("actor_normalized_prefix_dim", "332");
    require("gait_dim", "3");
    require("gait_encoding", "one_hot_exactly_one");
    require("gait_modes", "STAND,WALK,W2S");
    require("gait_normalization", "none_raw_onehot");
    require("normalizer_contract", "prefix_only_then_raw_gait_v1");
    require("onnx_preprocess_contract",
            "normalize_prefix_then_append_raw_gait_v1");
    require("gait_stability_predicate",
            "filtered_imu_ang_vel_norm_and_joint_vel_rms_v2");

    const std::string fsm_contract = ModelMetadata("gait_fsm_contract");
    // The first Gait-G export used the variant-less v3 identifier even though
    // it carries the complete GATHER/SETTLE metadata. Accept that artifact
    // only when the task variant and overlay identify the F/G scheduler; an
    // unqualified generic v3 model must still fail closed.
    const bool generic_v3_fg = fsm_contract == "flat_plus_gait_fsm_v3"
        && (ModelMetadata("gait_task_variant") == "F"
            || ModelMetadata("gait_task_variant") == "G")
        && ModelMetadata("gait_fsm_overlay")
               == "gather_first_direction_balanced_v1";
    const std::string gait_task_variant = ModelMetadata("gait_task_variant");
    const bool gait_0917 =
        fsm_contract == "flat_plus_gait_fsm_v1_stand_recovery_v1"
        && (gait_task_variant == "0917"
            || gait_task_variant == "0917V3"
            || gait_task_variant == "0917V4"
            || gait_task_variant == "0917V5A1"
            || gait_task_variant == "0917V5A2A");
    if (fsm_contract != "flat_plus_gait_fsm_v1"
        && !gait_0917
        && fsm_contract != "flat_plus_gait_fsm_v3b"
        && fsm_contract != "flat_plus_gait_fsm_v3f"
        && !generic_v3_fg) {
        throw std::runtime_error(
            "unsupported gait_fsm_contract: "
            + (fsm_contract.empty() ? std::string("<missing>") : fsm_contract));
    }

    const auto require_float = [&](const char* key, float expected) {
        const auto actual = ModelMetadata(key);
        try {
            if (actual.empty() || std::abs(std::stof(actual) - expected) > 1.0e-6f) {
                throw std::runtime_error("mismatch");
            }
        } catch (...) {
            throw std::runtime_error(
                std::string("gait policy metadata mismatch: ") + key
                + " expected '" + std::to_string(expected) + "', model has '"
                + (actual.empty() ? "<missing>" : actual) + "'");
        }
    };
    const auto require_float_one_of = [&](const char* key,
                                          float first,
                                          float second) {
        const auto actual = ModelMetadata(key);
        try {
            const float value = std::stof(actual);
            if (actual.empty() || !std::isfinite(value)
                || (std::abs(value - first) > 1.0e-6f
                    && std::abs(value - second) > 1.0e-6f)) {
                throw std::runtime_error("mismatch");
            }
        } catch (...) {
            throw std::runtime_error(
                std::string("gait policy metadata mismatch: ") + key
                + " expected one of '" + std::to_string(first) + "', '"
                + std::to_string(second) + "', model has '"
                + (actual.empty() ? "<missing>" : actual) + "'");
        }
    };
    require_float("gait_stop_request_lin", 0.10f);
    require_float("gait_stop_request_ang", 0.10f);
    require_float("gait_move_request_lin", 0.15f);
    require_float("gait_move_request_ang", 0.15f);
    if (fsm_contract == "flat_plus_gait_fsm_v1" || gait_0917) {
        if (gait_0917) {
            // Keep legacy 0917 settle1.5 accepted while qualifying settle05.
            require_float_one_of("gait_t_settle_s", 1.50f, 0.50f);
        } else {
            require_float("gait_t_settle_s", 1.50f);
        }
        require_float("gait_w2s_timeout_s", 5.00f);
        require_float("gait_stability_ang_vel_max", 0.25f);
        require_float("gait_stability_joint_vel_rms_max", 0.60f);
        if (gait_0917) {
            require("gait_command_overlay",
                    "legacy_0909_v2_stand_recovery_v1");
            if (gait_task_variant == "0917V5A1"
                || gait_task_variant == "0917V5A2A") {
                require("gait_v5_ablation", gait_task_variant);
                require("gait_reward_overlay", "0917_v5_versioned_ablation");
                require("gait_walk_push_matrix",
                        "direction_x_phase8_x_intensity3");
                require("gait_recovery_metrics",
                        "touchdown_displacement,time_to_touchdown,cp_margin");
            }
            require_float("gait_stand_upright_enabled", 1.00f);
            require_float("gait_stand_entry_tilt_max_rad", 0.08727f);
            require_float("gait_stand_push_exit_enabled", 1.00f);
            require_float("gait_push_exit_tilt_immediate_rad", 0.13963f);
            require_float("gait_push_exit_tilt_sustained_rad", 0.11345f);
            require_float("gait_push_exit_gyro_immediate", 0.75f);
            require_float("gait_push_exit_gyro_sustained", 0.50f);
            require_float("gait_push_exit_sustain_s", 0.30f);
        }
        return;
    }

    // v3 uses the same 335-D actor layout but a different mode machine. Keep
    // this check structural here; PolicyApplication verifies every numeric
    // value against tuning.yaml before it allows the control loop to run.
    require("gait_w2s_substates", "GATHER,SETTLE");
    require("gait_stability_joints", "all_joints");
    require("gait_stance_gate", "fk_width_band_dx_yaw_v2");
    require("gait_phase_clock",
            "run_walk_gather_freeze_settle_stand_restart_on_stand_exit_v3");
    require("gait_phase_obs", "zero_in_settle_and_stand_v3");
    const char* numeric_keys[] = {
        "gait_t_settle_s", "gait_w2s_timeout_s", "gait_stance_home_width_m",
        "gait_stance_width_tol_in_m", "gait_stance_width_tol_out_m",
        "gait_stance_dx_tol_m", "gait_stance_yaw_tol_rad",
        "gait_stance_exit_factor", "gait_gather_max_strides",
        "gait_gather_widen_factor", "gait_gather_force_settle_strides",
        "gait_forced_settle_exit_margin", "gait_stance_threshold",
        "gait_push_exit_tilt_immediate_rad", "gait_push_exit_tilt_sustained_rad",
        "gait_push_exit_gyro_immediate", "gait_push_exit_gyro_sustained",
        "gait_push_exit_joint_rms_sustained", "gait_push_exit_sustain_s",
        "gait_stand_entry_tilt_max_rad", "gait_stand_entry_gyro_max",
    };
    for (const char* key : numeric_keys) {
        const auto actual = ModelMetadata(key);
        try {
            if (actual.empty() || !std::isfinite(std::stof(actual)))
                throw std::runtime_error("missing or non-finite");
        } catch (...) {
            throw std::runtime_error(
                std::string("gait policy metadata missing v3 key: ") + key);
        }
    }
}
