#include <cassert>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "config/Tuning.hpp"

namespace {
void Require(bool condition) {
    if (!condition) std::abort();
}
}  // namespace

// Standard assert disappears under NDEBUG; CTest also runs in Release.
#undef assert
#define assert(condition) Require(static_cast<bool>(condition))

int main() {
    Tuning tuning;
    assert(tuning.LoadFromFile(HB_TUNING_CONFIG_PATH));
    assert(tuning.network_interface == "eth10");
    assert(tuning.flat_policy_contract == "flat_plus_gait_h4_v1");
    // Ghim route policy đang active để đổi model phải cập nhật manifest và
    // preflight cùng lúc; policy legacy vẫn giữ riêng để rollback.
    assert(tuning.flat_model == "policy_flat_plus_gait_0917_armfree.onnx");
    assert(tuning.teleop_h4_enabled);
    assert(tuning.teleop_h4_model_sha256 ==
           "7daeea2a351abda42e2e361836637b85bb8122c7d1c5d09339b909ecad815ccb");
    // Audio cues and per-motor head gains are part of the active runtime
    // config; dropping audio.yaml values silently falls back to Tuning defaults.
    assert(std::fabs(tuning.voice_volume - 0.95f) < 1e-6f);
    assert(tuning.voice_startup == "src/audio/start2.mp3");
    assert(tuning.voice_stand_lock == "src/audio/khoacung2.mp3");
    assert(tuning.voice_locomotion == "src/audio/batchedodichuyen2.mp3");
    assert(tuning.voice_mimic[0] == "src/audio/lamtynhac.mp3");
    assert(tuning.voice_mimic[1] == "src/audio/voice_doremon.mp3");
    assert(std::fabs(tuning.head_yaw_kp - 10.0f) < 1e-6f);
    assert(std::fabs(tuning.head_yaw_kd - 1.0f) < 1e-6f);
    assert(std::fabs(tuning.head_pitch_kp - 20.0f) < 1e-6f);
    assert(std::fabs(tuning.head_pitch_kd - 10.0f) < 1e-6f);
    assert(std::fabs(tuning.teleop_arm_kp - 40.0f) < 1e-6f);
    assert(std::fabs(tuning.teleop_arm_kd - 2.0f) < 1e-6f);
    assert(std::fabs(tuning.teleop_max_rate_rad_s - 2.0f) < 1e-6f);
    assert(!tuning.teleop_lock_others_enabled);
    assert(std::fabs(tuning.teleop_lock_kp - 20.0f) < 1e-6f);
    assert(std::fabs(tuning.teleop_lock_kd - 3.0f) < 1e-6f);
    assert(std::fabs(tuning.teleop_lock_max_rate_rad_s - 0.20f) < 1e-6f);
    assert(std::fabs(tuning.gesture_head_yaw_max - 0.65f) < 1e-6f);
    assert(std::fabs(tuning.gesture_head_pitch_max - 0.64f) < 1e-6f);
    assert(std::fabs(tuning.gesture_return_min_duration_s - 1.25f) < 1e-6f);
    assert(std::fabs(tuning.gesture_return_max_velocity_rad_s - 1.20f) < 1e-6f);
    assert(std::fabs(tuning.gesture_return_max_acceleration_rad_s2 - 8.0f) < 1e-6f);
    assert(std::fabs(tuning.gesture_handover_measured_position_tol_rad - 0.06f) < 1e-6f);
    assert(std::fabs(tuning.gesture_handover_dwell_s - 0.5f) < 1e-6f);
    assert(tuning.gesture_voice_command_socket == "/run/hb/gesture_owner.sock");
    assert(tuning.GestureVoiceSlot("wave") == 1);
    assert(tuning.GestureVoiceSlot("heart") == 6);
    assert(tuning.GestureVoiceSlot("handshake") == 2);
    assert(tuning.GestureVoiceSlot("cool_pose") == 7);
    assert(tuning.GestureVoiceSlot("determined") == 8);
    assert(tuning.GestureVoiceSlot("give_gift") == 4);
    assert(tuning.GestureVoiceSlot("unknown") == 0);
    assert(tuning.state_timeout_ms == 1000.0f);
    assert(tuning.remote_state_timeout_ms == 1000.0f);

    const auto invalid_path = std::filesystem::temp_directory_path() /
                              "hb_high_level_invalid_tuning.yaml";
    {
        std::ofstream invalid(invalid_path);
        invalid << "teleop_max_rate_rad_s: nan\n";
    }
    Tuning invalid;
    assert(!invalid.LoadFromFile(invalid_path.string()));

    {
        std::ofstream invalid_head(invalid_path);
        invalid_head << "teleop_head_yaw_max: nan\n";
    }
    Tuning invalid_head;
    assert(!invalid_head.LoadFromFile(invalid_path.string()));

    {
        std::ofstream invalid_hold(invalid_path);
        invalid_hold << "teleop_lock_kp: nan\n";
    }
    Tuning invalid_hold;
    assert(!invalid_hold.LoadFromFile(invalid_path.string()));

    // ─── Parser fail-closed ───────────────────────────────────────────────
    // Mọi trường hợp dưới đây TRƯỚC ĐÂY chỉ in cảnh báo rồi chạy tiếp, nên
    // config sai vẫn qua preflight.
    auto rejects = [&](const std::string& body) {
        {
            std::ofstream f(invalid_path);
            f << body;
        }
        Tuning t;
        return !t.LoadFromFile(invalid_path.string());
    };

    // Boolean gõ nhầm: từng lặng lẽ thành false = tắt bộ phát hiện ngã.
    assert(rejects("fall_enabled: ture\n"));
    assert(rejects("fall_enabled: Fasle\n"));
    assert(rejects("arm_gate_enabled: enable\n"));
    assert(rejects("joint_speed_guard_enabled: \n"));
    // Khoá lạ (gõ sai tên khoá) và dòng thiếu dấu ':'.
    assert(rejects("fall_tilt_de: 50.0\n"));
    assert(rejects("fall_tilt_deg 50.0\n"));
    // Giá trị số không parse được.
    assert(rejects("fall_tilt_deg: 50deg\n"));

    // Boolean hợp lệ vẫn phải nhận đủ các dạng.
    {
        std::ofstream f(invalid_path);
        f << "fall_enabled: FALSE\n"
             "arm_gate_enabled: off\n"
             "joint_speed_guard_enabled: yes\n";
    }
    Tuning bools;
    assert(bools.LoadFromFile(invalid_path.string()));
    assert(!bools.fall_enabled);
    assert(!bools.arm_gate_enabled);
    assert(bools.joint_speed_guard_enabled);

    // Ngưỡng an toàn phi lý: lớp an toàn vẫn "bật" nhưng không bao giờ kích hoạt.
    assert(rejects("fall_tilt_deg: 95.0\n"));
    assert(rejects("remote_state_timeout_ms: nan\n"));
    assert(rejects("state_timeout_ms: 1000\nremote_state_timeout_ms: 50\n"));
    assert(rejects("joint_speed_limit: 1000.0\n"));
    assert(rejects("gesture_voice_wave_slot: 9\n"));
    assert(rejects("state_timeout_ms: 0\n"));
    assert(rejects("arm_hold_s: 0.05\n"));
    // Ngưỡng lật nhanh phải NHẠY hơn ngưỡng nghiêng thuần, không thì vô dụng.
    assert(rejects("fall_tilt_deg: 30.0\nfall_flip_tilt_deg: 50.0\n"));
    // Pin: cảnh báo phải tới trước lúc cạn.
    assert(rejects("battery_warn_pct: 5\nbattery_critical_pct: 20\n"));
    assert(rejects("battery_critical_action: lie_down\n"));
    assert(rejects("teleop_h4_enabled: true\n"));
    assert(rejects("teleop_enabled: true\nteleop_h4_enabled: true\n"
                   "flat_policy_contract: legacy_83\n"
                   "teleop_h4_model_sha256: " + std::string(64, 'a') + "\n"));

    std::filesystem::remove(invalid_path);

    return 0;
}
