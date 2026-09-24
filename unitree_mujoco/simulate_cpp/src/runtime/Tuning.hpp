#pragma once

#include <array>
#include <string>

struct LocomotionProfile {
    std::string name = "Flat";
    float gait_period_s = 0.6f;
    std::array<float, 3> slow = {1.0f, 0.5f, 1.0f};
    std::array<float, 3> fast = {2.0f, 1.0f, 2.0f};
};

struct GaitModeTuning {
    float stop_request_lin = 0.10f;
    float stop_request_ang = 0.10f;
    float move_request_lin = 0.15f;
    float move_request_ang = 0.15f;
    float settle_gyro_norm = 0.25f;
    float settle_joint_velocity_rms = 0.60f;
    float settle_dwell_s = 1.50f;
    float w2s_timeout_s = 5.00f;
    float stability_filter_tau_s = 0.20f;
};

// Opt-in v3 FSM parameters. The defaults are the exported B/F/G training
// candidate values; the active policy metadata remains the source of truth
// and is checked against these values before inference starts.
struct GaitModeV3Tuning {
    std::array<float, 3> command_accel = {2.5f, 2.0f, 3.0f};
    std::array<float, 3> command_decel = {1.2f, 1.2f, 1.5f};
    float t_settle_s = 1.00f;
    float stance_width_tol_in_m = 0.04f;
    float stance_width_tol_out_m = 0.08f;
    float stance_dx_tol_m = 0.06f;
    float stance_yaw_tol_rad = 0.30f;
    float stance_exit_factor = 1.50f;
    int gather_max_strides = 3;
    float gather_widen_factor = 2.0f;
    int gather_force_settle_strides = 4;
    float forced_settle_exit_margin = 1.25f;
    float stance_threshold = 0.56f;
    float push_exit_tilt_immediate_rad = 0.13963f;
    float push_exit_tilt_sustained_rad = 0.11345f;
    float push_exit_gyro_immediate = 0.75f;
    float push_exit_gyro_sustained = 0.50f;
    float push_exit_joint_rms_sustained = 1.20f;
    float push_exit_sustain_s = 0.30f;
    float stand_entry_tilt_max_rad = 0.08727f;
    float stand_entry_gyro_max = 0.50f;
};

// Cấu hình runtime đọc từ config/tuning.yaml (singleton).
// Có thể chỉnh không cần build lại bằng cách sửa file YAML.
struct Tuning {
    // ── Profile locomotion ──
    // Tốc độ nhanh là giới hạn tuyệt đối theo từng trục, không còn nhân đồng loạt.
    LocomotionProfile flat_profile{
        "Flat", 0.6f, {1.0f, 0.5f, 1.0f}, {2.0f, 1.0f, 2.0f}};
    LocomotionProfile rough_profile{
        "Rough", 0.6f, {1.0f, 0.5f, 1.0f}, {2.0f, 0.8f, 1.0f}};
    LocomotionProfile slope_profile{
        "Slope", 0.6f, {1.0f, 0.5f, 1.0f}, {1.5f, 0.5f, 1.0f}};
    GaitModeTuning gait_mode{};
    GaitModeV3Tuning gait_mode_v3{};

    // ── Đường dẫn Policy & Motion ──
    // Chỉ cần đổi locomotion_policy trong tuning.yaml để chuyển model.
    // Tương thích config cũ: nếu flat_policy_path được khai báo thì nó ưu tiên hơn dir+file.
    std::string locomotion_policy_dir = "../policy/locomotion/";
    std::string locomotion_policy     = "flat/policy_v9.onnx";
    std::string flat_policy_path;
    // "auto" đọc đúng -s/--scene từ tiến trình unitree_mujoco R1 đang chạy.
    // Vẫn cho phép đường dẫn explicit để debug hoặc dùng simulator ngoài.
    std::string rough_scene_path = "auto";

    std::string dance_dir     = "../../../HB/high_level_2/policies/dance/";
    std::string dance_1_npz   = "lacmong1/r1_lacmong1.npz";
    std::string dance_1_onnx  = "lacmong1/r1_lacmong1.onnx";
    float dance_1_speed        = 1.0f;
    std::string dance_2_npz   = "lacmong2/r1_lacmong2_trim.npz";
    std::string dance_2_onnx  = "lacmong2/r1_lacmong2.onnx";
    float dance_2_speed        = 1.0f;
    std::string dance_3_npz   = "pokemon/pokemon_r1.npz";
    std::string dance_3_onnx  = "pokemon/policy.onnx";
    float dance_3_speed        = 1.0f;
    std::string dance_4_npz   = "doremon/doremon_r1.npz";
    std::string dance_4_onnx  = "doremon/policy.onnx";
    float dance_4_speed        = 1.0f;
    // Mirrors HB/high_level_2 dance.yaml. These affect only the handover
    // between locomotion and dance; the ONNX contracts remain unchanged.
    float mimic_warmup_s         = 1.5f;
    float mimic_cooldown_s       = 1.8f;
    float mimic_handover_tilt    = 0.15f;
    float mimic_handover_gyro    = 0.5f;
    float mimic_handover_min_s   = 0.3f;
    float mimic_handover_max_s   = 3.0f;
    float mimic_entry_settle_s   = 0.4f;
    float mimic_entry_gyro_max   = 0.5f;
    float mimic_entry_dq_max     = 0.6f;
    bool  mimic_transition_v2     = false;
    int   mimic_transition_search_frames = 200;
    float mimic_transition_clip_ramp_s = 0.35f;
    float mimic_transition_exit_pos_tol = 0.12f;
    float mimic_transition_exit_dq_tol = 0.60f;
    float mimic_transition_fallback_pos_tol = 0.25f;
    float mimic_transition_fallback_dq_tol = 1.50f;
    float dance_blend_time_s     = 0.5f;
    float dance_return_rate_limit = 1.5f;
    float dance_return_pos_tol    = 0.12f;
    float dance_return_timeout_s  = 6.0f;

    // ── Gesture tay (overlay khi Flat) ──
    // Gán slot GIỐNG HỆT config/gestures.yaml của deploy để thứ tự bấm trong sim
    // trùng thứ tự nút trên tay cầm robot: 1=Up 2=Down 3=Left 4=Right 5=A(voice)
    // 6=B 7=X 8=Y. Phím G trong sim duyệt đúng thứ tự này.
    std::string gesture_dir = "../../../HB/high_level_2/policies/gestures/";
    std::string gesture_slot[8] = {"", "", "", "", "", "", "", ""};
    // H4/gait gesture gate. Teleop remains limited to the legacy 83-D policy.
    bool gesture_history_enabled = false;
    float gesture_history_stand_dwell_s = 0.5f;
    float gesture_history_move_threshold = 0.05f;

    // Feedforward bù thăng bằng (khớp high_level_2). 0 = tắt. Đo/tune bằng --autotest.
    float gesture_balance_kg = 0.0f;   // -> obs[3] (projected gravity x = pitch)
    float gesture_balance_kv = 0.0f;   // -> obs[6] (cmd vx)

    // ── Tham số ngồi (SitController) — đồng bộ với high_level_2 ──
    float sit_hip_deg            = 150.0f;  // Độ gập hông. Dải thử: 130-160
    float sit_knee_deg           = 100.0f;  // Độ gập gối. Dải thử: 90-115
    float sit_lean_deg           = 58.0f;   // Đổ thân về trước khi hạ. Dải thử: 40-70
    float sit_seated_lean_deg    = 20.0f;   // Thân còn đổ sau khi mông chạm ghế
    float sit_arm_forward        = -1.5f;   // Vai vươn ra trước (rad, âm = ra trước)
    float sit_arm_elbow          = 0.3f;    // Khuỷu duỗi khi vươn tay (rad)
    float sit_spread             = 0.08f;   // Mở rộng chân đế (rad). Dải thử: 0.05-0.12
    float sit_ankle_gravity_gain = 0.4f;    // Bàn chân bám mặt đất theo IMU. 0=tắt
    float sit_descent_time_s     = 4.0f;    // Thời gian pha hạ (giây)
    float sit_settle_time_s      = 1.5f;    // Giao trọng lượng cho ghế + thu tay
    float sit_gather_time_s      = 1.5f;    // Thời gian pha thu chân (giây)
    float sit_hold_s             = 0.5f;
    float sit_kp_leg             = 200.0f;
    float sit_kd                 = 3.0f;
    float sit_rate_limit         = 3.0f;
    bool  sit_release_after      = false;

    // ── Âm thanh (LocalAudioPlayer) ──
    std::string audio_dir      = "../../../HB/high_level_2/src/audio/";
    std::string audio_sit      = "dangangngoixuong.mp3";
    std::string audio_dance_1  = "";
    std::string audio_dance_2  = "";
    std::string audio_dance_3  = "voice_pokemon.mp3";
    std::string audio_dance_4  = "voice_doremon.mp3";

    static Tuning& Get();
    std::string LocomotionPolicyPath() const;
    // Chọn profile theo cặp (policy_contract, observation_dim), FAIL-CLOSED.
    //
    // `policy_contract` là chuỗi đọc từ ONNX metadata, RỖNG nếu model không có
    // khoá đó. Hai nhánh hợp lệ:
    //   * không có contract -> chỉ chấp nhận dim thuộc {83,270} và đi đúng
    //     route legacy. 46/48 model trong repo thuộc nhóm này, kể cả baseline
    //     common-PD, nên bỏ nhánh này là làm vỡ toàn bộ đường cũ.
    //   * có contract       -> phải khớp đúng cặp đã đăng ký.
    // Mọi trường hợp khác ném ngoại lệ thay vì âm thầm rơi về Flat như trước.
    const LocomotionProfile& ResolveLocomotionProfile(
        int observation_dim, const std::string& policy_contract = "") const;
    void Load(const std::string& path);

private:
    void Apply(const std::string& key, const std::string& value);
};
