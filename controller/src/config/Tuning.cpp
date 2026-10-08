#include "Tuning.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <stdexcept>

namespace {

std::string Trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// Token lạ là LỖI, không phải false. Trước đây "ture"/"flase"/"Fasle" đều lặng
// lẽ thành false, tức một ký tự gõ nhầm là tắt fall detector hoặc arm gate mà
// preflight vẫn báo xanh. Ném ra để đường parse bên dưới ghi nhận và chặn chạy.
bool ToBool(const std::string& v) {
    std::string s = v;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "true" || s == "1" || s == "yes" || s == "on")   return true;
    if (s == "false" || s == "0" || s == "no" || s == "off")  return false;
    throw std::runtime_error("gia tri boolean khong hop le '" + v +
                             "' (chi nhan true/false/1/0/yes/no/on/off)");
}

} // namespace

void Tuning::Apply(const std::string& key, const std::string& value, bool& known) {
    known = true;
    // std::stof/stoi dừng ở ký tự lạ đầu tiên và KHÔNG báo lỗi: "50deg" thành
    // 50, "3.0.0" thành 3.0, "1,5" thành 1. Bắt buộc tiêu thụ hết chuỗi.
    auto f = [&]() {
        size_t pos = 0;
        const float out = std::stof(value, &pos);
        if (pos != value.size())
            throw std::runtime_error("thua ky tu sau so: '" + value + "'");
        return out;
    };
    auto i = [&]() {
        size_t pos = 0;
        const int out = std::stoi(value, &pos);
        if (pos != value.size())
            throw std::runtime_error("thua ky tu sau so: '" + value + "'");
        return out;
    };

    if      (key == "network_interface")  network_interface = value;
    else if (key == "dev_no_keyboard")    dev_no_keyboard = ToBool(value);
    else if (key == "imu_gyro_lpf_hz")    imu_gyro_lpf_hz = f();
    else if (key == "joint_vel_lpf_hz")   joint_vel_lpf_hz = f();
    else if (key == "imu_pitch_trim_deg") imu_pitch_trim_deg = f();
    else if (key == "imu_pitch_trim_affects_locomotion") imu_pitch_trim_affects_locomotion = ToBool(value);
    else if (key == "imu_pitch_trim_affects_dance") imu_pitch_trim_affects_dance = ToBool(value);
    else if (key == "locomotion_trim_deg") { locomotion_trim_deg = f(); locomotion_trim_configured = true; }
    else if (key == "stand_kp_leg")       stand_kp_leg = f();
    else if (key == "stand_kp_waist")     stand_kp_waist = f();
    else if (key == "stand_kp_arm")       stand_kp_arm = f();
    else if (key == "stand_kd")           stand_kd = f();
    else if (key == "return_kp_ankle")    return_kp_ankle = f();
    else if (key == "policy_kp_scale")    policy_kp_scale = f();
    else if (key == "policy_kd_scale")    policy_kd_scale = f();
    else if (key == "gait_period_s")      gait_period_s = f();
    else if (key == "slow_vx")            slow_vx = f();
    else if (key == "slow_vy")            slow_vy = f();
    else if (key == "slow_yaw")           slow_yaw = f();
    else if (key == "slow_vx_back")       slow_vx_back = f();
    else if (key == "fast_vx")            fast_vx = f();
    else if (key == "fast_vy")            fast_vy = f();
    else if (key == "fast_yaw")           fast_yaw = f();
    else if (key == "fast_vx_back")       fast_vx_back = f();
    else if (key == "cmd_accel_vx")       cmd_accel_vx = f();
    else if (key == "cmd_accel_vy")       cmd_accel_vy = f();
    else if (key == "cmd_accel_yaw")      cmd_accel_yaw = f();
    else if (key == "cmd_decel_vx")       cmd_decel_vx = f();
    else if (key == "cmd_decel_vy")       cmd_decel_vy = f();
    else if (key == "cmd_decel_yaw")      cmd_decel_yaw = f();
    else if (key == "heading_hold_enabled")      heading_hold_enabled = ToBool(value);
    else if (key == "heading_hold_kp")           heading_hold_kp = f();
    else if (key == "heading_hold_max_yaw")      heading_hold_max_yaw = f();
    else if (key == "heading_hold_move_min")     heading_hold_move_min = f();
    else if (key == "heading_hold_relatch_gyro") heading_hold_relatch_gyro = f();
    else if (key == "stand_up_time_s")    stand_up_time_s = f();
    else if (key == "blend_time_s")       blend_time_s = f();
    else if (key == "stand_rate_limit")   stand_rate_limit = f();
    else if (key == "lock_rate_limit")    lock_rate_limit = f();
    else if (key == "return_rate_limit")  return_rate_limit = f();
    else if (key == "stand_lock_spread")  stand_lock_spread = f();
    else if (key == "return_pos_tol")     return_pos_tol = f();
    else if (key == "return_timeout_s")   return_timeout_s = f();
    else if (key == "settle_time_s")      settle_time_s = f();
    else if (key == "settle_gyro_max")    settle_gyro_max = f();
    else if (key == "policy_rate_limit")  policy_rate_limit = f();
    else if (key == "fall_enabled")       fall_enabled = ToBool(value);
    else if (key == "fall_tilt_deg")      fall_tilt_deg = f();
    else if (key == "fall_flip_tilt_deg") fall_flip_tilt_deg = f();
    else if (key == "fall_flip_gyro")     fall_flip_gyro = f();
    else if (key == "fall_debounce_ms")   fall_debounce_ms = f();
    else if (key == "joint_speed_guard_enabled") joint_speed_guard_enabled = ToBool(value);
    else if (key == "joint_speed_limit")         joint_speed_limit = f();
    else if (key == "joint_speed_debounce_ms")   joint_speed_debounce_ms = f();
    else if (key == "sit_knee_deg")        sit_knee_deg = f();
    else if (key == "sit_descent_time_s")  sit_descent_time_s = f();
    else if (key == "sit_hold_s")          sit_hold_s = f();
    else if (key == "sit_release_after")   sit_release_after = ToBool(value);
    else if (key == "sit_kp_leg")          sit_kp_leg = f();
    else if (key == "sit_kd")              sit_kd = f();
    else if (key == "sit_rate_limit")      sit_rate_limit = f();
    else if (key == "stand_lock_warn_s")   stand_lock_warn_s = f();
    else if (key == "dance_abort_lock_block_s") dance_abort_lock_block_s = f();
    else if (key == "stand_lock_sit_block_s")   stand_lock_sit_block_s = f();
    else if (key == "sit_gather_time_s")   sit_gather_time_s = f();
    else if (key == "getup_motion_file")     getup_motion_file = value;
    else if (key == "liedown_motion_file")   liedown_motion_file = value;
    else if (key == "getup_kp_leg")          getup_kp_leg = f();
    else if (key == "getup_kp_waist")        getup_kp_waist = f();
    else if (key == "getup_kp_arm")          getup_kp_arm = f();
    else if (key == "getup_kd")              getup_kd = f();
    else if (key == "getup_rate_limit")      getup_rate_limit = f();
    else if (key == "liedown_rate_limit")    liedown_rate_limit = f();
    else if (key == "getup_blend_time_s")    getup_blend_time_s = f();
    else if (key == "liedown_blend_time_s")  liedown_blend_time_s = f();
    else if (key == "getup_speed")           getup_speed = f();
    else if (key == "liedown_speed")         liedown_speed = f();
    else if (key == "getup_liedown_block_s") getup_liedown_block_s = f();
    else if (key == "getup_ankle_gravity_gain") getup_ankle_gravity_gain = f();
    else if (key == "lying_tilt_deg")        lying_tilt_deg = f();
    else if (key == "voice_get_up")          voice_get_up = value;
    else if (key == "voice_lie_down")        voice_lie_down = value;
    else if (key == "sit_hip_deg")         sit_hip_deg = f();
    else if (key == "sit_lean_deg")        sit_lean_deg = f();
    else if (key == "sit_seated_lean_deg") sit_seated_lean_deg = f();
    else if (key == "sit_arm_forward")     sit_arm_forward = f();
    else if (key == "sit_arm_elbow")       sit_arm_elbow = f();
    else if (key == "sit_seated_arm_pitch") sit_seated_arm_pitch = f();
    else if (key == "sit_seated_arm_elbow") sit_seated_arm_elbow = f();
    else if (key == "sit_spread")          sit_spread = f();
    else if (key == "sit_rest_hip_deg")    sit_rest_hip_deg = f();
    else if (key == "sit_rest_knee_deg")   sit_rest_knee_deg = f();
    else if (key == "sit_rest_spread")     sit_rest_spread = f();
    else if (key == "sit_rest_hip_yaw")    sit_rest_hip_yaw = f();
    else if (key == "sit_ankle_gravity_gain") sit_ankle_gravity_gain = f();
    else if (key == "sit_settle_time_s")   sit_settle_time_s = f();
    else if (key == "safe_stop_enabled")   safe_stop_enabled = ToBool(value);
    else if (key == "safe_stop_debounce_ms") safe_stop_debounce_ms = f();
    else if (key == "arm_gate_enabled")        arm_gate_enabled = ToBool(value);
    else if (key == "arm_require_button")      arm_require_button = ToBool(value);
    else if (key == "arm_hold_s")              arm_hold_s = f();
    else if (key == "arm_silence_ms")          arm_silence_ms = f();
    else if (key == "arm_require_seen_builtin") arm_require_seen_builtin = ToBool(value);
    else if (key == "arm_min_foreign_seen")    arm_min_foreign_seen = i();
    else if (key == "arm_conflict_min")        arm_conflict_min = i();
    else if (key == "arm_conflict_window_ms")  arm_conflict_window_ms = f();
    else if (key == "arm_conflict_release")    arm_conflict_release = ToBool(value);
    else if (key == "arm_no_builtin_timeout_s") arm_no_builtin_timeout_s = f();
    else if (key == "battery_monitor_enabled") battery_monitor_enabled = ToBool(value);
    else if (key == "battery_topic")           battery_topic = value;
    else if (key == "battery_warn_pct")        battery_warn_pct = i();
    else if (key == "battery_critical_pct")    battery_critical_pct = i();
    else if (key == "battery_critical_action") battery_critical_action = value;
    else if (key == "battery_announce_period_s") battery_announce_period_s = f();
    else if (key == "battery_stale_s")         battery_stale_s = f();
    else if (key == "voice_enabled")       voice_enabled = ToBool(value);
    else if (key == "voice_volume")        voice_volume = f();
    else if (key == "voice_speaker_id")    voice_speaker_id = i();
    else if (key == "voice_startup")       voice_startup = value;
    else if (key == "startup_voice_delay_s") startup_voice_delay_s = f();
    else if (key == "voice_stand_lock")    voice_stand_lock = value;
    else if (key == "voice_locomotion")    voice_locomotion = value;
    else if (key == "voice_sit_down")      voice_sit_down = value;
    else if (key == "voice_safe_stop")     voice_safe_stop = value;
    else if (key == "voice_fast_speed")    voice_fast_speed = value;
    else if (key == "voice_slow_speed")    voice_slow_speed = value;
    else if (key == "voice_conflict")      voice_conflict = value;
    else if (key == "voice_battery_low")      voice_battery_low = value;
    else if (key == "voice_battery_critical") voice_battery_critical = value;
    else if (key == "voice_zero_torque")      voice_zero_torque = value;
    else if (key == "voice_teleop_on")        voice_teleop_on = value;
    else if (key == "voice_teleop_off")       voice_teleop_off = value;
    else if (key.size() > 12 && key.substr(0, 12) == "voice_mimic_") {
        int idx = std::stoi(key.substr(12)) - 2; // voice_mimic_2 -> idx 0
        if (idx >= 0 && idx < kMaxDances) voice_mimic[idx] = value;
    }
    else if (key == "hold_to_trigger_s")  hold_to_trigger_s = f();
    else if (key == "gesture_enabled")        gesture_enabled = ToBool(value);
    else if (key == "gesture_history_enabled") gesture_history_enabled = ToBool(value);
    else if (key == "gesture_history_stand_dwell_s") gesture_history_stand_dwell_s = f();
    else if (key == "gesture_history_move_threshold") gesture_history_move_threshold = f();
    else if (key == "gesture_folder")         gesture_folder = value;
    else if (key == "gesture_double_click_s") gesture_double_click_s = f();
    else if (key == "gesture_blend_in_s")     gesture_blend_in_s = f();
    else if (key == "gesture_retract_s")      gesture_retract_s = f();
    else if (key == "gesture_safety_retract_s") gesture_safety_retract_s = f();
    else if (key == "gesture_safety_max_vel_rad_s") gesture_safety_max_vel_rad_s = f();
    else if (key == "gesture_return_min_duration_s") gesture_return_min_duration_s = f();
    else if (key == "gesture_return_max_velocity_rad_s") gesture_return_max_velocity_rad_s = f();
    else if (key == "gesture_return_max_acceleration_rad_s2") gesture_return_max_acceleration_rad_s2 = f();
    else if (key == "gesture_return_pause_gyro_norm") gesture_return_pause_gyro_norm = f();
    else if (key == "gesture_handover_position_tol_rad") gesture_handover_position_tol_rad = f();
    else if (key == "gesture_handover_measured_position_tol_rad") gesture_handover_measured_position_tol_rad = f();
    else if (key == "gesture_handover_arm_dq_rms") gesture_handover_arm_dq_rms = f();
    else if (key == "gesture_handover_tilt_rad") gesture_handover_tilt_rad = f();
    else if (key == "gesture_handover_gyro_norm") gesture_handover_gyro_norm = f();
    else if (key == "gesture_handover_dwell_s") gesture_handover_dwell_s = f();
    else if (key == "gesture_balance_kg")     gesture_balance_kg = f();
    else if (key == "gesture_balance_kv")     gesture_balance_kv = f();
    else if (key == "gesture_guard_tilt")     gesture_guard_tilt = f();
    else if (key == "gesture_guard_gyro")     gesture_guard_gyro = f();
    else if (key == "gesture_demo_wave")      gesture_demo_wave = ToBool(value);
    else if (key == "gesture_head_yaw_max") gesture_head_yaw_max = f();
    else if (key == "gesture_head_pitch_max") gesture_head_pitch_max = f();
    else if (key == "gesture_head_rate_limit_rad_s") gesture_head_rate_limit_rad_s = f();
    else if (key == "gesture_voice_name")     gesture_voice_name = value;
    else if (key == "gesture_voice_marker")   gesture_voice_marker = value;
    else if (key == "gesture_voice_stale_s")  gesture_voice_stale_s = f();
    else if (key == "gesture_voice_auto_start") gesture_voice_auto_start = ToBool(value);
    else if (key == "gesture_voice_speed") gesture_voice_speed = f();
    else if (key == "gesture_voice_move_threshold") gesture_voice_move_threshold = f();
    else if (key == "gesture_voice_resume_threshold") gesture_voice_resume_threshold = f();
    else if (key == "gesture_voice_resume_delay_s") gesture_voice_resume_delay_s = f();
    else if (key == "gesture_voice_command_socket") gesture_voice_command_socket = value;
    else if (key == "gesture_voice_wave_slot") gesture_voice_wave_slot = i();
    else if (key == "gesture_voice_heart_slot") gesture_voice_heart_slot = i();
    else if (key == "gesture_voice_handshake_slot") gesture_voice_handshake_slot = i();
    else if (key == "gesture_voice_cool_pose_slot") gesture_voice_cool_pose_slot = i();
    else if (key == "gesture_voice_determined_slot") gesture_voice_determined_slot = i();
    else if (key == "gesture_voice_give_gift_slot") gesture_voice_give_gift_slot = i();
    else if (key.size() == 20 && key.substr(0, 19) == "gesture_slot_speed_") {
        int idx = key[19] - '1'; // gesture_slot_speed_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot_speed[idx] = f();
        else known = false;
    }
    else if (key.size() == 20 && key.substr(0, 19) == "gesture_slot_blend_") {
        int idx = key[19] - '1'; // gesture_slot_blend_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot_blend[idx] = f();
        else known = false;
    }
    else if (key.size() == 23 && key.substr(0, 22) == "gesture_slot_trim_deg_") {
        int idx = key[22] - '1'; // gesture_slot_trim_deg_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot_trim_deg[idx] = f();
    }
    else if (key.size() == 22 && key.substr(0, 21) == "gesture_slot_vx_bias_") {
        int idx = key[21] - '1'; // gesture_slot_vx_bias_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot_vx_bias[idx] = f();
    }
    else if (key.size() == 22 && key.substr(0, 21) == "gesture_slot_retract_") {
        int idx = key[21] - '1'; // gesture_slot_retract_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot_retract[idx] = f();
        else known = false;
    }
    else if (key.size() == 14 && key.substr(0, 13) == "gesture_slot_") {
        int idx = key[13] - '1'; // gesture_slot_1 -> 0
        if (idx >= 0 && idx < 8) gesture_slot[idx] = value;
    }
    else if (key == "teleop_enabled")         teleop_enabled = ToBool(value);
    else if (key == "teleop_h4_enabled")      teleop_h4_enabled = ToBool(value);
    else if (key == "teleop_h4_model_sha256") teleop_h4_model_sha256 = value;
    else if (key == "teleop_udp_port")        teleop_udp_port = i();
    else if (key == "teleop_timeout_ms")      teleop_timeout_ms = f();
    else if (key == "teleop_blend_in_s")      teleop_blend_in_s = f();
    else if (key == "teleop_retract_s")       teleop_retract_s = f();
    else if (key == "teleop_smooth_hz")       teleop_smooth_hz = f();
    else if (key == "teleop_head_yaw_max")    teleop_head_yaw_max = f();
    else if (key == "teleop_head_pitch_max")  teleop_head_pitch_max = f();
    else if (key == "teleop_arm_kp")           teleop_arm_kp = f();
    else if (key == "teleop_arm_kd")           teleop_arm_kd = f();
    else if (key == "teleop_max_rate_rad_s")   teleop_max_rate_rad_s = f();
    else if (key == "teleop_head_rate_limit_rad_s") teleop_head_rate_limit_rad_s = f();
    else if (key == "teleop_lock_base")       teleop_lock_base = ToBool(value);
    else if (key == "teleop_lock_others_enabled") teleop_lock_others_enabled = ToBool(value);
    else if (key == "teleop_lock_kp")          teleop_lock_kp = f();
    else if (key == "teleop_lock_kd")          teleop_lock_kd = f();
    else if (key == "teleop_lock_max_rate_rad_s") teleop_lock_max_rate_rad_s = f();
    else if (key == "state_timeout_ms")   state_timeout_ms = f();
    else if (key == "remote_state_timeout_ms") remote_state_timeout_ms = f();
    else if (key == "remote_timeout_ms")  remote_timeout_ms = f();
    else if (key == "remote_recover_ms")  remote_recover_ms = f();
    else if (key == "remote_require_neutral") remote_require_neutral = ToBool(value);
    else if (key == "x11_release_ms")     x11_release_ms = f();
    else if (key == "dance_start_frame")  dance_start_frame = i();
    else if (key == "dance_start_search_frames") dance_start_search_frames = i();
    else if (key == "mimic_announce_delay_s") mimic_announce_delay_s = f();
    else if (key == "mimic_announce_min_s")   mimic_announce_min_s = f();
    else if (key == "mimic_announce_timeout_s") mimic_announce_timeout_s = f();
    else if (key == "mimic_entry_settle_s") mimic_entry_settle_s = f();
    else if (key == "mimic_entry_gyro_max") mimic_entry_gyro_max = f();
    else if (key == "mimic_entry_dq_max") mimic_entry_dq_max = f();
    else if (key == "mimic_warmup_s")     mimic_warmup_s = f();
    else if (key == "mimic_cooldown_s")   mimic_cooldown_s = f();
    else if (key == "mimic_handover_tilt")  mimic_handover_tilt = f();
    else if (key == "mimic_handover_gyro")  mimic_handover_gyro = f();
    else if (key == "mimic_handover_min_s") mimic_handover_min_s = f();
    else if (key == "mimic_handover_max_s") mimic_handover_max_s = f();
    else if (key == "mimic_transition_v2") mimic_transition_v2 = ToBool(value);
    else if (key == "mimic_transition_search_frames") mimic_transition_search_frames = i();
    else if (key == "mimic_transition_clip_ramp_s") mimic_transition_clip_ramp_s = f();
    else if (key == "mimic_transition_exit_pos_tol") mimic_transition_exit_pos_tol = f();
    else if (key == "mimic_transition_exit_dq_tol") mimic_transition_exit_dq_tol = f();
    else if (key == "mimic_transition_fallback_pos_tol") mimic_transition_fallback_pos_tol = f();
    else if (key == "mimic_transition_fallback_dq_tol") mimic_transition_fallback_dq_tol = f();
    else if (key == "mimic_telemetry_enabled") mimic_telemetry_enabled = ToBool(value);
    else if (key == "mimic_telemetry_hz")      mimic_telemetry_hz = i();
    else if (key == "mimic_telemetry_post_s")  mimic_telemetry_post_s = f();
    else if (key == "mimic_telemetry_dir")     mimic_telemetry_dir = value;
    else if (key == "flat_policy_contract") flat_policy_contract = value;
    else if (key == "flat_model")         flat_model = value;
    else if (key == "gait_stop_request_lin") gait_stop_request_lin = f();
    else if (key == "gait_stop_request_ang") gait_stop_request_ang = f();
    else if (key == "gait_move_request_lin") gait_move_request_lin = f();
    else if (key == "gait_move_request_ang") gait_move_request_ang = f();
    else if (key == "gait_stability_gyro_norm") gait_stability_gyro_norm = f();
    else if (key == "gait_stability_joint_velocity_rms") gait_stability_joint_velocity_rms = f();
    else if (key == "gait_settle_dwell_s") gait_settle_dwell_s = f();
    else if (key == "gait_w2s_timeout_s") gait_w2s_timeout_s = f();
    else if (key == "gait_stability_filter_tau_s") gait_stability_filter_tau_s = f();
    else if (key.size() > 12 && key.substr(0, 12) == "dance_speed_") {
        int idx = std::stoi(key.substr(12)) - 2;
        if (idx >= 0 && idx < kMaxDances) dance_speed[idx] = f();
    }
    else if (key.size() > 13 && key.substr(0, 13) == "dance_volume_") {
        int idx = std::stoi(key.substr(13)) - 2;
        if (idx >= 0 && idx < kMaxDances) dance_volume[idx] = f();
    }
    else if (key.size() > 15 && key.substr(0, 15) == "dance_trim_deg_") {
        int idx = std::stoi(key.substr(15)) - 2;
        if (idx >= 0 && idx < kMaxDances) {
            dance_trim_deg[idx] = f();
            dance_trim_configured[idx] = true;
        }
    }
    // Parse dance_N folder name
    else if (key.size() > 6 && key.substr(0, 6) == "dance_" &&
             key.find_first_not_of("0123456789", 6) == std::string::npos) {
        int idx = std::stoi(key.substr(6)) - 2;
        if (idx >= 0 && idx < kMaxDances) dance_folder[idx] = value;
    }
    else if (key == "head_yaw_kp")        head_yaw_kp = f();
    else if (key == "head_yaw_kd")        head_yaw_kd = f();
    else if (key == "head_pitch_kp")      head_pitch_kp = f();
    else if (key == "head_pitch_kd")      head_pitch_kd = f();
    else known = false;
}

bool Tuning::LoadFromFile(const std::string& path) {
    std::set<std::string> loading;
    return LoadFromFileImpl(path, loading) && Validate();
}

bool Tuning::Validate() const {
    auto finite_in = [](float value, float lo, float hi) {
        return std::isfinite(value) && value >= lo && value <= hi;
    };
    bool ok = true;
    auto require = [&](bool condition, const char* message) {
        if (!condition) {
            std::cerr << "[Tuning] Invalid configuration: " << message << "\n";
            ok = false;
        }
    };

    require(!flat_model.empty(), "flat_model must not be empty");
    require(flat_policy_contract == "legacy_83" ||
                flat_policy_contract == "flat_plus_h4_v1" ||
                flat_policy_contract == "flat_plus_h5_v1" ||
                flat_policy_contract == "flat_plus_gait_h4_v1",
            "flat_policy_contract must be legacy_83, flat_plus_h4_v1, "
            "flat_plus_h5_v1 or flat_plus_gait_h4_v1");
    require(gesture_voice_command_socket.rfind("/run/hb/", 0) == 0 &&
                gesture_voice_command_socket.size() < 108 &&
                gesture_voice_command_socket.find("..") == std::string::npos,
            "gesture_voice_command_socket must be a short path under /run/hb");
    const int voice_gesture_slots[] = {
        gesture_voice_wave_slot, gesture_voice_heart_slot,
        gesture_voice_handshake_slot, gesture_voice_cool_pose_slot,
        gesture_voice_determined_slot, gesture_voice_give_gift_slot};
    for (const int slot : voice_gesture_slots) {
        require(slot >= 0 && slot <= 8,
                "voice gesture slot values must be 0 (disabled) or in [1, 8]");
        require(slot != 5 || gesture_voice_name.empty(),
                "voice gesture actions cannot use slot 5 while slot 5 is reserved for voice-auto");
    }
    if (teleop_h4_enabled) {
        const bool hash_ok = teleop_h4_model_sha256.size() == 64 &&
            std::all_of(teleop_h4_model_sha256.begin(), teleop_h4_model_sha256.end(),
                        [](char c) { return (c >= '0' && c <= '9') ||
                                            (c >= 'a' && c <= 'f'); });
        require(teleop_enabled && flat_policy_contract == "flat_plus_gait_h4_v1" && hash_ok,
                "teleop H4 requires teleop_enabled, exact gait-H4 contract and 64-char lowercase SHA-256");
    }
    // Sàn 100: dưới 84 thì 2 cổ chân không thắng nổi m*g*h=168 Nm/rad của robot ->
    // kReturningToDefault (không có policy đỡ) sẽ đổ. Xem chú thích ở Tuning.hpp.
    require(finite_in(return_kp_ankle, 100.0f, 400.0f),
            "return_kp_ankle must be finite and in [100, 400] "
            "(duoi 84 = robot do trong kReturningToDefault vi khong co policy do)");
    // PhaseObs chia cho gait_period_s -> 0 hoặc âm là chia cho 0/NaN đi thẳng vào obs.
    // Trần/sàn rộng nhưng vẫn chặn được lỗi nhập đơn vị (ms) hoặc để trống.
    require(finite_in(gait_period_s, 0.3f, 1.5f),
            "gait_period_s must be finite and in [0.3, 1.5] "
            "(phai bang 'period' cua phase obs luc train policy dang nap)");
    require(finite_in(gait_stop_request_lin, 0.0f, 10.0f),
            "gait_stop_request_lin must be finite and in [0, 10]");
    require(finite_in(gait_stop_request_ang, 0.0f, 10.0f),
            "gait_stop_request_ang must be finite and in [0, 10]");
    require(finite_in(gait_move_request_lin, 0.0f, 10.0f) &&
                gait_move_request_lin > gait_stop_request_lin,
            "gait_move_request_lin must be > gait_stop_request_lin and in [0, 10]");
    require(finite_in(gait_move_request_ang, 0.0f, 10.0f) &&
                gait_move_request_ang > gait_stop_request_ang,
            "gait_move_request_ang must be > gait_stop_request_ang and in [0, 10]");
    require(finite_in(gait_stability_gyro_norm, 0.0f, 20.0f),
            "gait_stability_gyro_norm must be finite and in [0, 20]");
    require(finite_in(gait_stability_joint_velocity_rms, 0.0f, 50.0f),
            "gait_stability_joint_velocity_rms must be finite and in [0, 50]");
    require(finite_in(gait_settle_dwell_s, 0.01f, 60.0f),
            "gait_settle_dwell_s must be finite and in [0.01, 60]");
    require(finite_in(gait_w2s_timeout_s, 0.1f, 120.0f),
            "gait_w2s_timeout_s must be finite and in [0.1, 120]");
    require(finite_in(gait_stability_filter_tau_s, 0.001f, 10.0f),
            "gait_stability_filter_tau_s must be finite and in [0.001, 10]");
    require(teleop_udp_port >= 1024 && teleop_udp_port <= 65535,
            "teleop_udp_port must be in [1024, 65535]");
    require(finite_in(teleop_timeout_ms, 20.0f, 2000.0f),
            "teleop_timeout_ms must be finite and in [20, 2000]");
    require(finite_in(teleop_blend_in_s, 0.05f, 5.0f),
            "teleop_blend_in_s must be finite and in [0.05, 5.0]");
    require(finite_in(teleop_retract_s, 0.05f, 5.0f),
            "teleop_retract_s must be finite and in [0.05, 5.0]");
    require(finite_in(teleop_smooth_hz, 0.1f, 50.0f),
            "teleop_smooth_hz must be finite and in [0.1, 50.0]");
    require(finite_in(teleop_head_yaw_max, 0.05f, 2.0f),
            "teleop_head_yaw_max must be finite and in [0.05, 2.0]");
    require(finite_in(teleop_head_pitch_max, 0.05f, 1.5f),
            "teleop_head_pitch_max must be finite and in [0.05, 1.5]");
    require(finite_in(teleop_arm_kp, 0.0f, 120.0f),
            "teleop_arm_kp must be finite and in [0, 120]");
    require(finite_in(teleop_arm_kd, 0.0f, 20.0f),
            "teleop_arm_kd must be finite and in [0, 20]");
    require(finite_in(teleop_max_rate_rad_s, 0.05f, 2.0f),
            "teleop_max_rate_rad_s must be finite and in [0.05, 2.0]");
    require(finite_in(teleop_head_rate_limit_rad_s, 0.05f, 4.0f),
            "teleop_head_rate_limit_rad_s must be finite and in [0.05, 4.0]");
    require(finite_in(teleop_lock_kp, 1.0f, 100.0f),
            "teleop_lock_kp must be finite and in [1.0, 100.0]");
    require(finite_in(teleop_lock_kd, 0.1f, 10.0f),
            "teleop_lock_kd must be finite and in [0.1, 10.0]");
    require(finite_in(teleop_lock_max_rate_rad_s, 0.02f, 0.50f),
            "teleop_lock_max_rate_rad_s must be finite and in [0.02, 0.50]");
    require(finite_in(gesture_head_yaw_max, 0.05f, 1.5f),
            "gesture_head_yaw_max must be finite and in [0.05, 1.5]");
    require(finite_in(gesture_history_stand_dwell_s, 0.1f, 10.0f),
            "gesture_history_stand_dwell_s must be finite and in [0.1, 10.0]");
    require(finite_in(gesture_history_move_threshold, 0.0f, 0.20f),
            "gesture_history_move_threshold must be finite and in [0.0, 0.20]");
    require(finite_in(gesture_return_min_duration_s, 0.2f, 15.0f),
            "gesture_return_min_duration_s must be finite and in [0.2, 15.0]");
    require(finite_in(gesture_return_max_velocity_rad_s, 0.05f, 4.0f),
            "gesture_return_max_velocity_rad_s must be finite and in [0.05, 4.0]");
    require(finite_in(gesture_return_max_acceleration_rad_s2, 0.05f, 20.0f),
            "gesture_return_max_acceleration_rad_s2 must be finite and in [0.05, 20.0]");
    require(finite_in(gesture_return_pause_gyro_norm, 0.05f, 5.0f),
            "gesture_return_pause_gyro_norm must be finite and in [0.05, 5.0]");
    require(finite_in(gesture_handover_position_tol_rad, 0.001f, 0.20f),
            "gesture_handover_position_tol_rad must be finite and in [0.001, 0.20]");
    require(finite_in(gesture_handover_measured_position_tol_rad, 0.001f, 0.30f),
            "gesture_handover_measured_position_tol_rad must be finite and in [0.001, 0.30]");
    require(finite_in(gesture_handover_arm_dq_rms, 0.01f, 2.0f),
            "gesture_handover_arm_dq_rms must be finite and in [0.01, 2.0]");
    require(finite_in(gesture_handover_tilt_rad, 0.01f, 0.50f),
            "gesture_handover_tilt_rad must be finite and in [0.01, 0.50]");
    require(finite_in(gesture_handover_gyro_norm, 0.01f, 5.0f),
            "gesture_handover_gyro_norm must be finite and in [0.01, 5.0]");
    require(finite_in(gesture_handover_dwell_s, 0.0f, 10.0f),
            "gesture_handover_dwell_s must be finite and in [0.0, 10.0]");
    require(finite_in(gesture_head_pitch_max, 0.05f, 1.0f),
            "gesture_head_pitch_max must be finite and in [0.05, 1.0]");
    require(finite_in(gesture_head_rate_limit_rad_s, 0.05f, 4.0f),
            "gesture_head_rate_limit_rad_s must be finite and in [0.05, 4.0]");

    // Gains và rate-limit của mọi state không chạy policy. Trước đây KHÔNG khoá
    // nào trong nhóm này được kiểm: một chữ số thừa (200 -> 2000) hoặc giá trị
    // âm/NaN đi thẳng vào packet motor. LowCmdSender có trần cứng (kp 500, kd 50,
    // rate 200 rad/s) nhưng đó là chặn RÁC — trần đó vẫn cho qua những giá trị
    // đủ sai để giật hỏng khớp, nên phải chặn ngay từ config.
    auto require_kp = [&](float value, const char* name) {
        require(finite_in(value, 10.0f, 400.0f), name);
    };
    auto require_kd = [&](float value, const char* name) {
        require(finite_in(value, 0.5f, 20.0f), name);
    };
    auto require_rate = [&](float value, const char* name) {
        require(finite_in(value, 0.05f, 50.0f), name);
    };
    require_kp(stand_kp_leg,   "stand_kp_leg must be finite and in [10, 400]");
    require_kp(stand_kp_waist, "stand_kp_waist must be finite and in [10, 400]");
    require_kp(stand_kp_arm,   "stand_kp_arm must be finite and in [10, 400]");
    require_kd(stand_kd,       "stand_kd must be finite and in [0.5, 20]");
    require_kp(sit_kp_leg,     "sit_kp_leg must be finite and in [10, 400]");
    require_kd(sit_kd,         "sit_kd must be finite and in [0.5, 20]");
    require_kp(getup_kp_leg,   "getup_kp_leg must be finite and in [10, 400]");
    require_kp(getup_kp_waist, "getup_kp_waist must be finite and in [10, 400]");
    require_kp(getup_kp_arm,   "getup_kp_arm must be finite and in [10, 400]");
    require_kd(getup_kd,       "getup_kd must be finite and in [0.5, 20]");
    require_rate(stand_rate_limit,   "stand_rate_limit must be finite and in [0.05, 50]");
    require_rate(lock_rate_limit,    "lock_rate_limit must be finite and in [0.05, 50]");
    require_rate(return_rate_limit,  "return_rate_limit must be finite and in [0.05, 50]");
    require_rate(sit_rate_limit,     "sit_rate_limit must be finite and in [0.05, 50]");
    require_rate(getup_rate_limit,   "getup_rate_limit must be finite and in [0.05, 50]");
    require_rate(liedown_rate_limit, "liedown_rate_limit must be finite and in [0.05, 50]");
    require_rate(policy_rate_limit,  "policy_rate_limit must be finite and in [0.05, 50]");

    // Scale nhân THẲNG vào gains đã export trong ONNX — lệch khỏi 1.0 là lệch
    // khỏi điều kiện lúc train. Dải hẹp cố ý: đây là núm tinh chỉnh, không phải
    // chỗ đổi gains.
    require(finite_in(policy_kp_scale, 0.5f, 2.0f),
            "policy_kp_scale must be finite and in [0.5, 2.0]");
    require(finite_in(policy_kd_scale, 0.5f, 2.0f),
            "policy_kd_scale must be finite and in [0.5, 2.0]");

    // Gain đầu: cùng lý do, cùng trần với các gain khác nhưng dải thấp hơn hẳn
    // (đầu nhẹ, kp lớn là giật cổ).
    require(finite_in(head_yaw_kp, 0.0f, 60.0f),
            "head_yaw_kp must be finite and in [0, 60]");
    require(finite_in(head_pitch_kp, 0.0f, 60.0f),
            "head_pitch_kp must be finite and in [0, 60]");
    require(finite_in(head_yaw_kd, 0.0f, 20.0f),
            "head_yaw_kd must be finite and in [0, 20]");
    require(finite_in(head_pitch_kd, 0.0f, 20.0f),
            "head_pitch_kd must be finite and in [0, 20]");

    // ─── Ngưỡng của các lớp an toàn ───────────────────────────────────────
    // Nhóm này trước đây KHÔNG được kiểm gì. Bật/tắt (fall_enabled,
    // joint_speed_guard_enabled, arm_gate_enabled...) nay đã an toàn nhờ parser
    // từ chối token boolean lạ; phần còn lại là các NGƯỠNG — đặt sai một chữ số
    // ở đây thì lớp an toàn vẫn "bật" nhưng không bao giờ kích hoạt.
    require(finite_in(fall_tilt_deg, 10.0f, 89.0f),
            "fall_tilt_deg must be finite and in [10, 89] "
            "(>=90 do la robot phai nam ngang han moi bao nga)");
    require(finite_in(fall_flip_tilt_deg, 5.0f, 89.0f) &&
                fall_flip_tilt_deg <= fall_tilt_deg,
            "fall_flip_tilt_deg must be finite, in [5, 89] and <= fall_tilt_deg "
            "(nguong lat nhanh phai NHAY hon nguong nghieng thuan)");
    require(finite_in(fall_flip_gyro, 0.5f, 30.0f),
            "fall_flip_gyro must be finite and in [0.5, 30] rad/s");
    require(finite_in(fall_debounce_ms, 2.0f, 1000.0f),
            "fall_debounce_ms must be finite and in [2, 1000]");
    require(finite_in(joint_speed_limit, 5.0f, 100.0f),
            "joint_speed_limit must be finite and in [5, 100] rad/s");
    require(finite_in(joint_speed_debounce_ms, 2.0f, 1000.0f),
            "joint_speed_debounce_ms must be finite and in [2, 1000]");

    // Watchdog DDS: nay con quyet dinh ca viec goi tay cam co duoc doc khong.
    require(finite_in(state_timeout_ms, 20.0f, 5000.0f),
            "state_timeout_ms must be finite and in [20, 5000]");
    require(finite_in(remote_state_timeout_ms, 20.0f, 5000.0f) &&
                remote_state_timeout_ms >= state_timeout_ms,
            "remote_state_timeout_ms must be in [20,5000] and >= state_timeout_ms");
    require(finite_in(remote_timeout_ms, 100.0f, 30000.0f),
            "remote_timeout_ms must be finite and in [100, 30000]");
    require(finite_in(remote_recover_ms, 0.0f, 5000.0f),
            "remote_recover_ms must be finite and in [0, 5000]");
    require(finite_in(safe_stop_debounce_ms, 0.0f, 10000.0f),
            "safe_stop_debounce_ms must be finite and in [0, 10000]");

    require(mimic_transition_search_frames >= 1 && mimic_transition_search_frames <= 5000,
            "mimic_transition_search_frames must be in [1, 5000]");
    require(finite_in(mimic_transition_clip_ramp_s, 0.0f, 5.0f),
            "mimic_transition_clip_ramp_s must be finite and in [0, 5]");
    require(finite_in(mimic_transition_exit_pos_tol, 0.01f, 0.50f),
            "mimic_transition_exit_pos_tol must be finite and in [0.01, 0.50]");
    require(finite_in(mimic_transition_exit_dq_tol, 0.05f, 10.0f),
            "mimic_transition_exit_dq_tol must be finite and in [0.05, 10]");
    require(finite_in(mimic_transition_fallback_pos_tol, 0.01f, 0.75f) &&
                mimic_transition_fallback_pos_tol >= mimic_transition_exit_pos_tol,
            "mimic_transition_fallback_pos_tol must be >= exit_pos_tol and in [0.01, 0.75]");
    require(finite_in(mimic_transition_fallback_dq_tol, 0.05f, 15.0f) &&
                mimic_transition_fallback_dq_tol >= mimic_transition_exit_dq_tol,
            "mimic_transition_fallback_dq_tol must be >= exit_dq_tol and in [0.05, 15]");
    require(finite_in(mimic_entry_settle_s, 0.0f, 5.0f),
            "mimic_entry_settle_s must be finite and in [0, 5]");
    require(finite_in(mimic_entry_gyro_max, 0.0f, 10.0f),
            "mimic_entry_gyro_max must be finite and in [0, 10]");
    require(finite_in(mimic_entry_dq_max, 0.05f, 15.0f),
            "mimic_entry_dq_max must be finite and in [0.05, 15]");

    // Cổng bàn giao với built-in.
    require(finite_in(arm_hold_s, 0.5f, 30.0f),
            "arm_hold_s must be finite and in [0.5, 30] "
            "(duoi 0.5s la cham tay vao cung arm)");
    require(finite_in(arm_silence_ms, 100.0f, 60000.0f),
            "arm_silence_ms must be finite and in [100, 60000]");
    require(arm_min_foreign_seen >= 1 && arm_min_foreign_seen <= 10000,
            "arm_min_foreign_seen must be in [1, 10000]");
    require(finite_in(arm_no_builtin_timeout_s, 0.0f, 600.0f),
            "arm_no_builtin_timeout_s must be finite and in [0, 600] (0 = tat bypass)");

    // Pin: ngưỡng đảo ngược thì cảnh báo không bao giờ tới trước lúc cạn.
    require(battery_warn_pct >= 0 && battery_warn_pct <= 100,
            "battery_warn_pct must be in [0, 100]");
    require(battery_critical_pct >= 0 && battery_critical_pct <= 100 &&
                battery_critical_pct < battery_warn_pct,
            "battery_critical_pct must be in [0, 100] and < battery_warn_pct");
    require(battery_critical_action == "sit" || battery_critical_action == "damp" ||
                battery_critical_action == "voice",
            "battery_critical_action must be sit, damp or voice");
    return ok;
}

bool Tuning::LoadFromFileImpl(const std::string& path, std::set<std::string>& loading) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path resolved = fs::absolute(fs::path(path), ec).lexically_normal();
    const std::string resolved_text = resolved.string();
    if (!loading.insert(resolved_text).second) {
        std::cerr << "[Tuning] Include cycle detected at " << resolved_text << "\n";
        return false;
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cout << "[Tuning] Config file not found: " << path << " -> using defaults.\n";
        loading.erase(resolved_text);
        return false;
    }

    std::string line;
    int line_no = 0, loaded = 0;
    bool success = true;
    while (std::getline(file, line)) {
        ++line_no;
        // Bỏ comment
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = Trim(line);
        if (line.empty()) continue;

        size_t colon = line.find(':');
        if (colon == std::string::npos) {
            std::cerr << "[Tuning] Line " << line_no << " missing ':' in " << path
                      << " -> " << line << "\n";
            success = false;   // dòng không parse được = config không dùng được
            continue;
        }
        std::string key = Trim(line.substr(0, colon));
        std::string value = Trim(line.substr(colon + 1));
        // Bỏ ngoặc kép
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
            value = value.substr(1, value.size() - 2);

        // Entry-point config có thể tách thành nhiều file phẳng theo nhóm:
        //   include: locomotion.yaml
        // Đường dẫn tương đối với file đang đọc; thứ tự include quyết định override.
        if (key == "include") {
            if (value.empty()) {
                std::cerr << "[Tuning] Empty include on line " << line_no << " in " << path << "\n";
                success = false;
                continue;
            }
            fs::path include_path(value);
            if (include_path.is_relative()) include_path = fs::path(path).parent_path() / include_path;
            std::cout << "[Tuning] Include " << include_path.string() << "\n";
            if (!LoadFromFileImpl(include_path.lexically_normal().string(), loading)) success = false;
            continue;
        }

        // FAIL-CLOSED. Cả ba nhánh dưới đây trước kia chỉ in ra rồi chạy tiếp,
        // nên một dòng hỏng nghĩa là khoá đó lặng lẽ giữ giá trị mặc định (hoặc
        // bị bỏ hẳn) trong khi người vận hành tin là đã đặt. Với file safety.yaml
        // thì đó là chạy robot bằng một cấu hình không ai đọc được.
        bool known = false;
        try {
            Apply(key, value, known);
        } catch (const std::exception& e) {
            std::cerr << "[Tuning] Parse error on line " << line_no << " of " << path
                      << " ('" << key << ": " << value << "'): " << e.what() << "\n";
            success = false;
            continue;
        }
        if (!known) {
            std::cerr << "[Tuning] Unknown key '" << key << "' on line " << line_no
                      << " of " << path << "\n";
            success = false;
        } else {
            ++loaded;
        }
    }
    std::cout << "[Tuning] Loaded " << loaded << " parameters from " << path << "\n";
    if (policy_kp_scale != 1.0f || policy_kd_scale != 1.0f) {
        std::cout << "[Tuning] Warning: policy gains scaled (KP=" << policy_kp_scale << ", KD=" << policy_kd_scale << ")\n";
    }
    loading.erase(resolved_text);
    return success;
}
