#include "onnx/OnnxModel.hpp"
#include "onnx/PolicyContract.hpp"
#include "runtime/R1Config.hpp"
#include "runtime/R1PolicyContract.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) {
        throw std::invalid_argument("usage: single_policy_smoke_test POLICY.onnx");
    }
    try {
        Ort::Env environment(ORT_LOGGING_LEVEL_WARNING, "single_policy_smoke");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        r1::policy::OnnxModel model(environment, argv[1], options);
        if (model.InputCount() != 1 || model.OutputCount() != 1
            || model.OutputSize(0) != R1Config::NUM_JOINTS) {
            throw std::runtime_error("expected one-input ONNX with a 24-D action");
        }
        const std::size_t observation_size = model.InputSize(0);
        const std::string contract = model.Metadata("policy_contract");
        const auto require_history_schema = [&](const char* expected_contract,
                                                const char* expected_steps,
                                                const char* expected_dim) {
            if (contract != expected_contract
                || model.Metadata("actor_input_schema") != expected_contract
                || model.Metadata("actor_history_order") != "term_major_oldest_to_newest"
                || model.Metadata("actor_history_padding") != "repeat_first"
                || model.Metadata("actor_history_steps") != expected_steps
                || model.Metadata("actor_input_dim") != expected_dim
                || model.Metadata("base_observation_schema") != "r1_common_pd_base83_v1"
                || model.Metadata("base_observation_dim") != "83"
                || model.Metadata("observation_terms") !=
                       "base_ang_vel,projected_gravity,command,phase,joint_pos,joint_vel,actions") {
                throw std::runtime_error("incomplete flat history schema");
            }
        };
        if (observation_size == 332) {
            // 332-D chỉ được đi qua khi metadata mô tả ĐẦY ĐỦ layout history.
            // Cùng số phần tử đó có thể là term-major hoặc time-major, và hai
            // cách cho action hoàn toàn khác nhau; nhận bừa theo dimension là
            // đúng cái lỗi mà kiểm tra kích thước không bao giờ bắt được.
            require_history_schema("flat_plus_h4_v1", "4", "332");
        } else if (observation_size == 335) {
            require_history_schema("flat_plus_gait_h4_v1", "4", "335");
            const std::string fsm_contract = model.Metadata("gait_fsm_contract");
            const std::string task_variant = model.Metadata("gait_task_variant");
            const bool generic_v3_fg = fsm_contract == "flat_plus_gait_fsm_v3"
                && (task_variant == "F" || task_variant == "G")
                && model.Metadata("gait_fsm_overlay")
                       == "gather_first_direction_balanced_v1";
            const bool gait_0917 =
                fsm_contract == "flat_plus_gait_fsm_v1_stand_recovery_v1"
                && (task_variant == "0917"
                    || task_variant == "0917V3"
                    || task_variant == "0917V4"
                    || task_variant == "0917V5A1"
                    || task_variant == "0917V5A2A");
            const auto require_float = [&](const char* key, float expected) {
                const std::string raw = model.Metadata(key);
                try {
                    if (raw.empty() || !std::isfinite(std::stof(raw))
                        || std::abs(std::stof(raw) - expected) > 1.0e-5f)
                        throw std::runtime_error("numeric mismatch");
                } catch (...) {
                    throw std::runtime_error(std::string("metadata mismatch: ") + key);
                }
            };
            const auto require_float_one_of = [&](const char* key,
                                                   float first,
                                                   float second) {
                const std::string raw = model.Metadata(key);
                try {
                    const float actual = std::stof(raw);
                    if (raw.empty() || !std::isfinite(actual)
                        || (std::abs(actual - first) > 1.0e-5f
                            && std::abs(actual - second) > 1.0e-5f))
                        throw std::runtime_error("numeric mismatch");
                } catch (...) {
                    throw std::runtime_error(std::string("metadata mismatch: ") + key);
                }
            };
            if (model.Metadata("actor_obs_groups") != "actor,gait"
                || model.Metadata("actor_obs_group_dims") != "332,3"
                || model.Metadata("actor_normalized_prefix_dim") != "332"
                || model.Metadata("gait_dim") != "3"
                || model.Metadata("gait_encoding") != "one_hot_exactly_one"
                || model.Metadata("gait_modes") != "STAND,WALK,W2S"
                || model.Metadata("gait_normalization") != "none_raw_onehot"
                || model.Metadata("normalizer_contract") != "prefix_only_then_raw_gait_v1"
                || model.Metadata("onnx_preprocess_contract")
                    != "normalize_prefix_then_append_raw_gait_v1"
                || (fsm_contract != "flat_plus_gait_fsm_v1"
                    && !gait_0917
                    && fsm_contract != "flat_plus_gait_fsm_v3b"
                    && fsm_contract != "flat_plus_gait_fsm_v3f"
                    && !generic_v3_fg)
                || model.Metadata("gait_stability_predicate")
                    != "filtered_imu_ang_vel_norm_and_joint_vel_rms_v2"
                || model.Metadata("gait_stop_request_lin") != "0.1"
                || model.Metadata("gait_stop_request_ang") != "0.1"
                || model.Metadata("gait_move_request_lin") != "0.15"
                || model.Metadata("gait_move_request_ang") != "0.15"
                ) {
                throw std::runtime_error("incomplete flat_plus_gait_h4_v1 schema");
            }
            if (fsm_contract == "flat_plus_gait_fsm_v1" || gait_0917) {
                if (gait_0917) {
                    // 0917 has the historical settle1.5 artifact and the
                    // explicitly qualified settle05 export.
                    require_float_one_of("gait_t_settle_s", 1.5f, 0.5f);
                } else {
                    require_float("gait_t_settle_s", 1.5f);
                }
                require_float("gait_w2s_timeout_s", 5.0f);
                require_float("gait_stability_ang_vel_max", 0.25f);
                require_float("gait_stability_joint_vel_rms_max", 0.6f);
                if (gait_0917) {
                    if (model.Metadata("gait_command_overlay")
                        != "legacy_0909_v2_stand_recovery_v1") {
                        throw std::runtime_error("metadata mismatch: gait_command_overlay");
                    }
                    if (task_variant == "0917V5A1" || task_variant == "0917V5A2A") {
                        if (model.Metadata("gait_v5_ablation") != task_variant
                            || model.Metadata("gait_reward_overlay")
                                != "0917_v5_versioned_ablation"
                            || model.Metadata("gait_walk_push_matrix")
                                != "direction_x_phase8_x_intensity3"
                            || model.Metadata("gait_recovery_metrics")
                                != "touchdown_displacement,time_to_touchdown,cp_margin") {
                            throw std::runtime_error("incomplete 0917 V5 ablation metadata");
                        }
                    }
                    require_float("gait_stand_upright_enabled", 1.0f);
                    require_float("gait_stand_entry_tilt_max_rad", 0.08727f);
                    require_float("gait_stand_push_exit_enabled", 1.0f);
                    require_float("gait_push_exit_tilt_immediate_rad", 0.13963f);
                    require_float("gait_push_exit_tilt_sustained_rad", 0.11345f);
                    require_float("gait_push_exit_gyro_immediate", 0.75f);
                    require_float("gait_push_exit_gyro_sustained", 0.50f);
                    require_float("gait_push_exit_sustain_s", 0.30f);
                }
            } else {
                const char* strings[][2] = {
                    {"gait_w2s_substates", "GATHER,SETTLE"},
                    {"gait_stability_joints", "all_joints"},
                    {"gait_stance_gate", "fk_width_band_dx_yaw_v2"},
                    {"gait_phase_clock", "run_walk_gather_freeze_settle_stand_restart_on_stand_exit_v3"},
                    {"gait_phase_obs", "zero_in_settle_and_stand_v3"},
                    {"gait_leg_offsets", "0.0,0.5"},
                    {"gait_command_accel", "2.5,2.0,3.0"},
                    {"gait_command_decel", "1.2,1.2,1.5"},
                    {"gait_phase_start", "0.0"},
                };
                for (const auto& item : strings) {
                    if (model.Metadata(item[0]) != item[1])
                        throw std::runtime_error(std::string("metadata mismatch: ") + item[0]);
                }
                const std::pair<const char*, float> numeric[] = {
                    {"gait_t_settle_s", 1.00f}, {"gait_w2s_timeout_s", 5.00f},
                    {"gait_stance_home_width_m", 0.212f},
                    {"gait_stance_width_tol_in_m", 0.04f},
                    {"gait_stance_width_tol_out_m", 0.08f},
                    {"gait_stance_dx_tol_m", 0.06f},
                    {"gait_stance_yaw_tol_rad", 0.30f},
                    {"gait_stance_exit_factor", 1.50f},
                    {"gait_gather_max_strides", 3.0f},
                    {"gait_gather_widen_factor", 2.0f},
                    {"gait_gather_force_settle_strides", 4.0f},
                    {"gait_forced_settle_exit_margin", 1.25f},
                    {"gait_stance_threshold", 0.56f},
                    {"gait_push_exit_tilt_immediate_rad", 0.13963f},
                    {"gait_push_exit_tilt_sustained_rad", 0.11345f},
                    {"gait_push_exit_gyro_immediate", 0.75f},
                    {"gait_push_exit_gyro_sustained", 0.50f},
                    {"gait_push_exit_joint_rms_sustained", 1.20f},
                    {"gait_push_exit_sustain_s", 0.30f},
                    {"gait_stand_entry_tilt_max_rad", 0.08727f},
                    {"gait_stand_entry_gyro_max", 0.50f},
                };
                for (const auto& item : numeric) {
                    const std::string raw = model.Metadata(item.first);
                    if (raw.empty() || !std::isfinite(std::stof(raw))
                        || std::abs(std::stof(raw) - item.second) > 1.0e-5f)
                        throw std::runtime_error(std::string("metadata mismatch: ") + item.first);
                }
            }
        } else if (observation_size == 415) {
            require_history_schema("flat_plus_h5_v1", "5", "415");
        } else if (observation_size != 83
                   && observation_size != 270) {
            throw std::runtime_error("unsupported single-policy observation dimension");
        }

        r1::policy::PolicyContract fallback{
            R1Config::DEFAULT_JOINT_POS,
            R1Config::ACTION_SCALE,
            R1Config::KP_ARRAY,
            R1Config::KD_ARRAY};
        (void)r1::policy::LoadPolicyContract(
            model, std::move(fallback), r1::contract::JointNames());

        const auto action = model.RunSingle(
            std::vector<float>(observation_size, 0.0f));
        if (action.size() != R1Config::NUM_JOINTS
            || !std::all_of(action.begin(), action.end(), [](float value) {
                return std::isfinite(value);
            })) {
            throw std::runtime_error("policy produced an invalid action");
        }
        std::cout << "single_policy_smoke_test: PASS obs=" << observation_size
                  << " model=" << argv[1] << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "single_policy_smoke_test: FAIL model=" << argv[1]
                  << " error=" << error.what() << '\n';
        return 1;
    }
}
