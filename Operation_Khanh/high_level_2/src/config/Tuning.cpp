#include "Tuning.hpp"

#include <algorithm>
#include <cctype>

namespace {

std::string Trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool ToBool(const std::string& v) {
    std::string s = v;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return (s == "true" || s == "1" || s == "yes" || s == "on");
}

} // namespace

void Tuning::Apply(const std::string& key, const std::string& value, bool& known) {
    known = true;
    auto f = [&]() { return std::stof(value); };
    auto i = [&]() { return std::stoi(value); };

    if      (key == "network_interface")  network_interface = value;
    else if (key == "dev_no_keyboard")    dev_no_keyboard = ToBool(value);
    else if (key == "imu_gyro_lpf_hz")    imu_gyro_lpf_hz = f();
    else if (key == "joint_vel_lpf_hz")   joint_vel_lpf_hz = f();
    else if (key == "imu_pitch_trim_deg") imu_pitch_trim_deg = f();
    else if (key == "imu_pitch_trim_affects_locomotion") imu_pitch_trim_affects_locomotion = ToBool(value);
    else if (key == "imu_pitch_trim_affects_dance") imu_pitch_trim_affects_dance = ToBool(value);
    else if (key == "stand_kp_leg")       stand_kp_leg = f();
    else if (key == "stand_kp_waist")     stand_kp_waist = f();
    else if (key == "stand_kp_arm")       stand_kp_arm = f();
    else if (key == "stand_kd")           stand_kd = f();
    else if (key == "policy_kp_scale")    policy_kp_scale = f();
    else if (key == "policy_kd_scale")    policy_kd_scale = f();
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
    else if (key.size() > 12 && key.substr(0, 12) == "voice_mimic_") {
        int idx = std::stoi(key.substr(12)) - 2; // voice_mimic_2 -> idx 0
        if (idx >= 0 && idx < kMaxDances) voice_mimic[idx] = value;
    }
    else if (key == "hold_to_trigger_s")  hold_to_trigger_s = f();
    else if (key == "state_timeout_ms")   state_timeout_ms = f();
    else if (key == "remote_timeout_ms")  remote_timeout_ms = f();
    else if (key == "remote_recover_ms")  remote_recover_ms = f();
    else if (key == "remote_require_neutral") remote_require_neutral = ToBool(value);
    else if (key == "x11_release_ms")     x11_release_ms = f();
    else if (key == "dance_start_frame")  dance_start_frame = i();
    else if (key == "dance_start_search_frames") dance_start_search_frames = i();
    else if (key == "mimic_announce_delay_s") mimic_announce_delay_s = f();
    else if (key == "mimic_announce_min_s")   mimic_announce_min_s = f();
    else if (key == "mimic_announce_timeout_s") mimic_announce_timeout_s = f();
    else if (key == "mimic_warmup_s")     mimic_warmup_s = f();
    else if (key == "mimic_cooldown_s")   mimic_cooldown_s = f();
    else if (key == "mimic_handover_tilt")  mimic_handover_tilt = f();
    else if (key == "mimic_handover_gyro")  mimic_handover_gyro = f();
    else if (key == "mimic_handover_min_s") mimic_handover_min_s = f();
    else if (key == "mimic_handover_max_s") mimic_handover_max_s = f();
    else if (key == "mimic_telemetry_enabled") mimic_telemetry_enabled = ToBool(value);
    else if (key == "mimic_telemetry_hz")      mimic_telemetry_hz = i();
    else if (key == "mimic_telemetry_post_s")  mimic_telemetry_post_s = f();
    else if (key == "mimic_telemetry_dir")     mimic_telemetry_dir = value;
    else if (key == "flat_model")         flat_model = value;
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
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cout << "[Tuning] Config file not found: " << path << " -> using defaults.\n";
        return false;
    }

    std::string line;
    int line_no = 0, loaded = 0;
    while (std::getline(file, line)) {
        ++line_no;
        // Bỏ comment
        size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = Trim(line);
        if (line.empty()) continue;

        size_t colon = line.find(':');
        if (colon == std::string::npos) {
            std::cerr << "[Tuning] Line " << line_no << " missing ':' -> skip: " << line << "\n";
            continue;
        }
        std::string key = Trim(line.substr(0, colon));
        std::string value = Trim(line.substr(colon + 1));
        // Bỏ ngoặc kép
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
            value = value.substr(1, value.size() - 2);

        bool known = false;
        try {
            Apply(key, value, known);
        } catch (const std::exception& e) {
            std::cerr << "[Tuning] Parse error on line " << line_no << " ('" << key
                      << ": " << value << "'): " << e.what() << "\n";
            continue;
        }
        if (!known) {
            std::cerr << "[Tuning] Unknown key: '" << key << "'\n";
        } else {
            ++loaded;
        }
    }
    std::cout << "[Tuning] Loaded " << loaded << " parameters from " << path << "\n";
    if (policy_kp_scale != 1.0f || policy_kd_scale != 1.0f) {
        std::cout << "[Tuning] Warning: policy gains scaled (KP=" << policy_kp_scale << ", KD=" << policy_kd_scale << ")\n";
    }
    return true;
}
