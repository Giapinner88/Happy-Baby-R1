#include "runtime/Tuning.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <iostream>

Tuning& Tuning::Get() {
    static Tuning instance;
    return instance;
}

std::string Tuning::LocomotionPolicyPath() const {
    if (!flat_policy_path.empty()) return flat_policy_path;
    if (locomotion_policy_dir.empty()) return locomotion_policy;
    if (locomotion_policy.empty()) return locomotion_policy_dir;
    if (locomotion_policy_dir.back() == '/') {
        return locomotion_policy_dir + locomotion_policy;
    }
    return locomotion_policy_dir + "/" + locomotion_policy;
}

const LocomotionProfile& Tuning::ResolveLocomotionProfile(
    int observation_dim, const std::string& policy_contract) const {
    // Nhánh 1: model KHÔNG khai policy_contract (mọi artifact legacy trong
    // repo). Chỉ hai dimension được phép, và không dimension nào khác được rơi
    // về Flat như resolver cũ vẫn làm.
    if (policy_contract.empty()) {
        if (observation_dim == 270) return rough_profile;
        if (observation_dim == 83) {
            std::string path = LocomotionPolicyPath();
            std::transform(path.begin(), path.end(), path.begin(),
                           [](unsigned char character) {
                               return static_cast<char>(std::tolower(character));
                           });
            if (path.find("slope/") != std::string::npos
                || path.find("/slope") != std::string::npos) {
                return slope_profile;
            }
            return flat_profile;
        }
        throw std::runtime_error(
            "locomotion policy input " + std::to_string(observation_dim)
            + "-D has no policy_contract metadata; only 83/270 may run "
              "without one");
    }

    // Nhánh 2: có contract -> phải khớp đúng cặp. Không suy layout từ dimension:
    // một vector 332-D time-major cũng đúng 332 phần tử.
    if (policy_contract == "flat_plus_h4_v1") {
        if (observation_dim != 332) {
            throw std::runtime_error(
                "flat_plus_h4_v1 requires a 332-D input, got "
                + std::to_string(observation_dim) + "-D");
        }
        return flat_profile;
    }
    if (policy_contract == "flat_plus_h5_v1") {
        if (observation_dim != 415) {
            throw std::runtime_error(
                "flat_plus_h5_v1 requires a 415-D input, got "
                + std::to_string(observation_dim) + "-D");
        }
        return flat_profile;
    }
    if (policy_contract == "flat_plus_gait_h4_v1") {
        if (observation_dim != 335) {
            throw std::runtime_error(
                "flat_plus_gait_h4_v1 requires a 335-D input, got "
                + std::to_string(observation_dim) + "-D");
        }
        return flat_profile;
    }
    throw std::runtime_error(
        "unknown policy_contract '" + policy_contract + "' with "
        + std::to_string(observation_dim) + "-D input; refusing to guess a profile");
}

void Tuning::Load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cout << "[Tuning] Không tìm thấy " << path
                  << ", dùng giá trị mặc định.\n";
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) line = line.substr(0, comment);

        const std::size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos) continue;
        line = line.substr(start);
        const std::size_t end = line.find_last_not_of(" \t\r\n");
        if (end != std::string::npos) line = line.substr(0, end + 1);

        const std::size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string key = line.substr(0, colon);
        std::string value = line.substr(colon + 1);
        key.erase(key.find_last_not_of(" \t\r\n") + 1);
        const std::size_t value_start = value.find_first_not_of(" \t\r\n");
        if (value_start != std::string::npos) value = value.substr(value_start);
        Apply(key, value);
    }
    std::cout << "[Tuning] Đã load config từ " << path << '\n';
}

void Tuning::Apply(const std::string& key, const std::string& value) {
    if (key == "locomotion_policy_dir") locomotion_policy_dir = value;
    else if (key == "locomotion_policy") locomotion_policy = value;
    else if (key == "rough_scene_path") rough_scene_path = value;
    else if (key == "flat_policy_path") flat_policy_path = value;
    else if (key == "gesture_dir") gesture_dir = value;
    else if (key == "gesture_history_enabled") {
        gesture_history_enabled = value == "true" || value == "1";
    }
    else if (key.size() == 14 && key.compare(0, 13, "gesture_slot_") == 0
             && key[13] >= '1' && key[13] <= '8') {
        gesture_slot[key[13] - '1'] = value;
    } else if (key == "dance_dir") dance_dir = value;
    else if (key == "dance_1_npz") dance_1_npz = value;
    else if (key == "dance_1_onnx") dance_1_onnx = value;
    else if (key == "dance_1_speed") dance_1_speed = std::stof(value);
    else if (key == "dance_2_npz") dance_2_npz = value;
    else if (key == "dance_2_onnx") dance_2_onnx = value;
    else if (key == "dance_2_speed") dance_2_speed = std::stof(value);
    else if (key == "dance_3_npz") dance_3_npz = value;
    else if (key == "dance_3_onnx") dance_3_onnx = value;
    else if (key == "dance_3_speed") dance_3_speed = std::stof(value);
    else if (key == "dance_4_npz") dance_4_npz = value;
    else if (key == "dance_4_onnx") dance_4_onnx = value;
    else if (key == "dance_4_speed") dance_4_speed = std::stof(value);
    else if (key == "mimic_warmup_s") mimic_warmup_s = std::stof(value);
    else if (key == "mimic_cooldown_s") mimic_cooldown_s = std::stof(value);
    else if (key == "mimic_handover_tilt") mimic_handover_tilt = std::stof(value);
    else if (key == "mimic_handover_gyro") mimic_handover_gyro = std::stof(value);
    else if (key == "mimic_handover_min_s") mimic_handover_min_s = std::stof(value);
    else if (key == "mimic_handover_max_s") mimic_handover_max_s = std::stof(value);
    else if (key == "mimic_entry_settle_s") mimic_entry_settle_s = std::stof(value);
    else if (key == "mimic_entry_gyro_max") mimic_entry_gyro_max = std::stof(value);
    else if (key == "mimic_entry_dq_max") mimic_entry_dq_max = std::stof(value);
    else if (key == "mimic_transition_v2") mimic_transition_v2 = value == "true" || value == "1";
    else if (key == "mimic_transition_search_frames") mimic_transition_search_frames = std::stoi(value);
    else if (key == "mimic_transition_clip_ramp_s") mimic_transition_clip_ramp_s = std::stof(value);
    else if (key == "mimic_transition_exit_pos_tol") mimic_transition_exit_pos_tol = std::stof(value);
    else if (key == "mimic_transition_exit_dq_tol") mimic_transition_exit_dq_tol = std::stof(value);
    else if (key == "mimic_transition_fallback_pos_tol") mimic_transition_fallback_pos_tol = std::stof(value);
    else if (key == "mimic_transition_fallback_dq_tol") mimic_transition_fallback_dq_tol = std::stof(value);
    else if (key == "dance_blend_time_s") dance_blend_time_s = std::stof(value);
    else if (key == "dance_return_rate_limit") dance_return_rate_limit = std::stof(value);
    else if (key == "dance_return_pos_tol") dance_return_pos_tol = std::stof(value);
    else if (key == "dance_return_timeout_s") dance_return_timeout_s = std::stof(value);
    else if (key == "audio_dir") audio_dir = value;
    else if (key == "audio_sit") audio_sit = value;
    else if (key == "audio_dance_1") audio_dance_1 = value;
    else if (key == "audio_dance_2") audio_dance_2 = value;
    else if (key == "audio_dance_3") audio_dance_3 = value;
    else if (key == "audio_dance_4") audio_dance_4 = value;
    else {
        try {
            const float number = std::stof(value);
            if (key == "flat_gait_period_s") flat_profile.gait_period_s = number;
            else if (key == "flat_speed_vx") flat_profile.slow[0] = number;
            else if (key == "flat_speed_vy") flat_profile.slow[1] = number;
            else if (key == "flat_speed_yaw") flat_profile.slow[2] = number;
            else if (key == "flat_fast_speed_vx") flat_profile.fast[0] = number;
            else if (key == "flat_fast_speed_vy") flat_profile.fast[1] = number;
            else if (key == "flat_fast_speed_yaw") flat_profile.fast[2] = number;
            else if (key == "rough_gait_period_s") rough_profile.gait_period_s = number;
            else if (key == "rough_speed_vx") rough_profile.slow[0] = number;
            else if (key == "rough_speed_vy") rough_profile.slow[1] = number;
            else if (key == "rough_speed_yaw") rough_profile.slow[2] = number;
            else if (key == "rough_fast_speed_vx") rough_profile.fast[0] = number;
            else if (key == "rough_fast_speed_vy") rough_profile.fast[1] = number;
            else if (key == "rough_fast_speed_yaw") rough_profile.fast[2] = number;
            else if (key == "slope_gait_period_s") slope_profile.gait_period_s = number;
            else if (key == "slope_speed_vx") slope_profile.slow[0] = number;
            else if (key == "slope_speed_vy") slope_profile.slow[1] = number;
            else if (key == "slope_speed_yaw") slope_profile.slow[2] = number;
            else if (key == "slope_fast_speed_vx") slope_profile.fast[0] = number;
            else if (key == "slope_fast_speed_vy") slope_profile.fast[1] = number;
            else if (key == "slope_fast_speed_yaw") slope_profile.fast[2] = number;
            else if (key == "speed_vx") flat_profile.slow[0] = number;
            else if (key == "speed_vy") flat_profile.slow[1] = number;
            else if (key == "speed_yaw") flat_profile.slow[2] = number;
            else if (key == "locomotion_gait_period_s") flat_profile.gait_period_s = number;
            else if (key == "gait_mode_stop_request_lin") gait_mode.stop_request_lin = number;
            else if (key == "gait_mode_stop_request_ang") gait_mode.stop_request_ang = number;
            else if (key == "gait_mode_move_request_lin") gait_mode.move_request_lin = number;
            else if (key == "gait_mode_move_request_ang") gait_mode.move_request_ang = number;
            else if (key == "gait_mode_settle_gyro_norm") gait_mode.settle_gyro_norm = number;
            else if (key == "gait_mode_settle_joint_velocity_rms") gait_mode.settle_joint_velocity_rms = number;
            else if (key == "gait_mode_settle_dwell_s") gait_mode.settle_dwell_s = number;
            else if (key == "gait_mode_w2s_timeout_s") gait_mode.w2s_timeout_s = number;
            else if (key == "gait_mode_stability_filter_tau_s") gait_mode.stability_filter_tau_s = number;
            else if (key == "gait_command_accel_vx") gait_mode_v3.command_accel[0] = number;
            else if (key == "gait_command_accel_vy") gait_mode_v3.command_accel[1] = number;
            else if (key == "gait_command_accel_yaw") gait_mode_v3.command_accel[2] = number;
            else if (key == "gait_command_decel_vx") gait_mode_v3.command_decel[0] = number;
            else if (key == "gait_command_decel_vy") gait_mode_v3.command_decel[1] = number;
            else if (key == "gait_command_decel_yaw") gait_mode_v3.command_decel[2] = number;
            else if (key == "gait_t_settle_s") gait_mode_v3.t_settle_s = number;
            else if (key == "gait_stance_width_tol_in_m") gait_mode_v3.stance_width_tol_in_m = number;
            else if (key == "gait_stance_width_tol_out_m") gait_mode_v3.stance_width_tol_out_m = number;
            else if (key == "gait_stance_dx_tol_m") gait_mode_v3.stance_dx_tol_m = number;
            else if (key == "gait_stance_yaw_tol_rad") gait_mode_v3.stance_yaw_tol_rad = number;
            else if (key == "gait_stance_exit_factor") gait_mode_v3.stance_exit_factor = number;
            else if (key == "gait_gather_max_strides") gait_mode_v3.gather_max_strides = static_cast<int>(number);
            else if (key == "gait_gather_widen_factor") gait_mode_v3.gather_widen_factor = number;
            else if (key == "gait_gather_force_settle_strides") gait_mode_v3.gather_force_settle_strides = static_cast<int>(number);
            else if (key == "gait_forced_settle_exit_margin") gait_mode_v3.forced_settle_exit_margin = number;
            else if (key == "gait_stance_threshold") gait_mode_v3.stance_threshold = number;
            else if (key == "gait_push_exit_tilt_immediate_rad") gait_mode_v3.push_exit_tilt_immediate_rad = number;
            else if (key == "gait_push_exit_tilt_sustained_rad") gait_mode_v3.push_exit_tilt_sustained_rad = number;
            else if (key == "gait_push_exit_gyro_immediate") gait_mode_v3.push_exit_gyro_immediate = number;
            else if (key == "gait_push_exit_gyro_sustained") gait_mode_v3.push_exit_gyro_sustained = number;
            else if (key == "gait_push_exit_joint_rms_sustained") gait_mode_v3.push_exit_joint_rms_sustained = number;
            else if (key == "gait_push_exit_sustain_s") gait_mode_v3.push_exit_sustain_s = number;
            else if (key == "gait_stand_entry_tilt_max_rad") gait_mode_v3.stand_entry_tilt_max_rad = number;
            else if (key == "gait_stand_entry_gyro_max") gait_mode_v3.stand_entry_gyro_max = number;
            else if (key == "gesture_balance_kg") gesture_balance_kg = number;
            else if (key == "gesture_balance_kv") gesture_balance_kv = number;
            else if (key == "gesture_history_stand_dwell_s") gesture_history_stand_dwell_s = number;
            else if (key == "gesture_history_move_threshold") gesture_history_move_threshold = number;
            else if (key == "fast_mode_multiplier") {
                flat_profile.fast = {
                    flat_profile.slow[0] * number,
                    flat_profile.slow[1] * number,
                    flat_profile.slow[2] * number};
            } else if (key == "sit_hip_deg") sit_hip_deg = number;
            else if (key == "sit_knee_deg") sit_knee_deg = number;
            else if (key == "sit_lean_deg") sit_lean_deg = number;
            else if (key == "sit_seated_lean_deg") sit_seated_lean_deg = number;
            else if (key == "sit_arm_forward") sit_arm_forward = number;
            else if (key == "sit_arm_elbow") sit_arm_elbow = number;
            else if (key == "sit_spread") sit_spread = number;
            else if (key == "sit_ankle_gravity_gain") sit_ankle_gravity_gain = number;
            else if (key == "sit_descent_time_s") sit_descent_time_s = number;
            else if (key == "sit_settle_time_s") sit_settle_time_s = number;
            else if (key == "sit_hold_s") sit_hold_s = number;
            else if (key == "sit_kp_leg") sit_kp_leg = number;
            else if (key == "sit_kd") sit_kd = number;
            else if (key == "sit_rate_limit") sit_rate_limit = number;
            else if (key == "sit_gather_time_s") sit_gather_time_s = number;
            else if (key == "sit_release_after") {
                sit_release_after = value == "true" || value == "1";
            } else {
                std::cout << "[Tuning] Cảnh báo: không nhận diện được key '"
                          << key << "'\n";
            }
        } catch (...) {
            if (key == "sit_release_after") {
                sit_release_after = value == "true" || value == "1";
            } else {
                std::cout << "[Tuning] Lỗi parse '" << key << "': " << value << '\n';
            }
        }
    }
}
