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
    else if (key == "sit_knee_deg")        sit_knee_deg = f();
    else if (key == "sit_descent_time_s")  sit_descent_time_s = f();
    else if (key == "sit_hold_s")          sit_hold_s = f();
    else if (key == "sit_release_after")   sit_release_after = ToBool(value);
    else if (key == "sit_kp_leg")          sit_kp_leg = f();
    else if (key == "sit_kd")              sit_kd = f();
    else if (key == "sit_rate_limit")      sit_rate_limit = f();
    else if (key == "sit_gather_time_s")   sit_gather_time_s = f();
    else if (key == "sit_stance")          sit_stance = f();
    else if (key == "sit_lateral_flat")    sit_lateral_flat = ToBool(value);
    else if (key == "safe_stop_enabled")   safe_stop_enabled = ToBool(value);
    else if (key == "safe_stop_debounce_ms") safe_stop_debounce_ms = f();
    else if (key == "voice_enabled")       voice_enabled = ToBool(value);
    else if (key == "voice_volume")        voice_volume = f();
    else if (key == "voice_speaker_id")    voice_speaker_id = i();
    else if (key == "voice_startup")       voice_startup = value;
    else if (key == "voice_stand_lock")    voice_stand_lock = value;
    else if (key == "voice_locomotion")    voice_locomotion = value;
    else if (key == "voice_sit_down")      voice_sit_down = value;
    else if (key == "voice_safe_stop")     voice_safe_stop = value;
    else if (key.size() > 12 && key.substr(0, 12) == "voice_mimic_") {
        int idx = std::stoi(key.substr(12)) - 2;
        if (idx >= 0 && idx < kMaxDances) voice_mimic[idx] = value;
    }
    else if (key == "state_timeout_ms")   state_timeout_ms = f();
    else if (key == "remote_timeout_ms")  remote_timeout_ms = f();
    else if (key == "x11_release_ms")     x11_release_ms = f();
    else if (key == "dance_start_frame")  dance_start_frame = i();
    else if (key == "dance_start_search_frames") dance_start_search_frames = i();
    else if (key == "mimic_announce_delay_s") mimic_announce_delay_s = f();
    else if (key == "flat_model")         flat_model = value;
    else if (key.size() > 12 && key.substr(0, 12) == "dance_speed_") {
        int idx = std::stoi(key.substr(12)) - 2;
        if (idx >= 0 && idx < kMaxDances) dance_speed[idx] = f();
    }
    else if (key.size() > 13 && key.substr(0, 13) == "dance_volume_") {
        int idx = std::stoi(key.substr(13)) - 2;
        if (idx >= 0 && idx < kMaxDances) dance_volume[idx] = f();
    }
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
        std::cout << "[Tuning] Không tìm thấy " << path
                  << " -> dùng toàn bộ giá trị mặc định trong code.\n";
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
            std::cerr << "[Tuning] Dòng " << line_no << " không có ':' -> bỏ qua: " << line << "\n";
            continue;
        }
        std::string key = Trim(line.substr(0, colon));
        std::string value = Trim(line.substr(colon + 1));
        // Bỏ ngoặc kép nếu có
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
            value = value.substr(1, value.size() - 2);

        bool known = false;
        try {
            Apply(key, value, known);
        } catch (const std::exception& e) {
            std::cerr << "[Tuning] Lỗi parse dòng " << line_no << " ('" << key
                      << ": " << value << "'): " << e.what() << "\n";
            continue;
        }
        if (!known) {
            std::cerr << "[Tuning] ⚠ Key không nhận diện: '" << key << "'\n";
        } else {
            ++loaded;
        }
    }
    std::cout << "[Tuning] Đã nạp " << loaded << " tham số từ " << path << "\n";
    if (policy_kp_scale != 1.0f || policy_kd_scale != 1.0f) {
        std::cout << "[Tuning] ⚠⚠ policy_kp_scale=" << policy_kp_scale
                  << " policy_kd_scale=" << policy_kd_scale
                  << " — gains LỆCH so với train!\n";
    }
    return true;
}
