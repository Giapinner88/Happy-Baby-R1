#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

// Struct chứa các tham số có thể tune (load từ tuning.yaml)
struct Tuning {
    // Mạng / giao tiếp
    std::string network_interface = "eth10";

    // Cấu hình điều khiển headless
    bool dev_no_keyboard = false;

    // Bộ lọc IMU & khớp
    float imu_gyro_lpf_hz = 20.0f;    // Dải thử: 15-25 Hz. 0.0 để tắt.
    float joint_vel_lpf_hz = 0.0f;    // Dải thử: 10-20 Hz. 0.0 để tắt.
    float imu_pitch_trim_deg = 0.0f;  // Bù góc pitch cho gravity. Dải thử: ±1..4°.
    // Bật/tắt trim riêng cho từng phía — độc lập nhau, có thể chỉ bật 1 trong 2:
    //   affects_locomotion: true (mặc định) = trim áp vào projected_gravity (locomotion + fall-detector).
    //   affects_dance:      false (mặc định) = trim KHÔNG áp vào state_.quat (dance/Mimic TorsoQuat).
    bool imu_pitch_trim_affects_locomotion = true;
    bool imu_pitch_trim_affects_dance      = false;

    // Gains khi không chạy policy
    float stand_kp_leg   = 200.0f;
    float stand_kp_waist = 200.0f;
    float stand_kp_arm   = 40.0f;
    float stand_kd       = 3.0f;

    // Tỉ lệ scale gains policy (mặc định 1.0)
    float policy_kp_scale = 1.0f;
    float policy_kd_scale = 1.0f;

    // Giới hạn tốc độ đi bộ (vx, vy, yaw)
    float slow_vx = 0.5f, slow_vy = 0.3f, slow_yaw = 0.5f, slow_vx_back = 0.4f;
    float fast_vx = 1.0f, fast_vy = 0.5f, fast_yaw = 1.0f, fast_vx_back = 0.5f;

    // Giới hạn gia tốc lệnh di chuyển (tăng tốc / đổi hướng)
    float cmd_accel_vx  = 2.5f;
    float cmd_accel_vy  = 2.0f;
    float cmd_accel_yaw = 3.0f;

    // Giới hạn giảm tốc lệnh (khi lệnh về 0 — thả stick); < accel = lướt hãm mềm
    float cmd_decel_vx  = 1.2f;
    float cmd_decel_vy  = 1.2f;
    float cmd_decel_yaw = 1.5f;

    // Heading-hold: giữ hướng khi đi thẳng (đóng vòng lặp yaw, khử drift open-loop).
    // enabled=false -> hành vi y hệt cũ. kp = heading_control_stiffness lúc train.
    // max_yaw = trần lệnh sửa (rad/s, thấp cho an toàn nếu IMU lỗi).
    // move_min = dưới tốc độ này coi như đứng -> tắt. relatch_gyro = thân còn xoay nhanh
    // hơn mức này thì CHƯA chốt hướng (đợi hết quán tính sau khi bẻ lái).
    bool  heading_hold_enabled      = false;
    float heading_hold_kp           = 0.8f;
    float heading_hold_max_yaw      = 0.4f;
    float heading_hold_move_min     = 0.15f;
    float heading_hold_relatch_gyro = 0.3f;

    // Chuyển trạng thái & blend tư thế
    float stand_up_time_s   = 2.5f;
    float blend_time_s      = 0.5f;
    float stand_rate_limit  = 5.0f;
    float lock_rate_limit   = 0.2f;
    float return_rate_limit = 1.5f;
    float stand_lock_spread = 0.08f;
    float return_pos_tol    = 0.12f;

    // Thời gian chờ robot dừng hẳn trước khi chuyển trạng thái
    float settle_time_s   = 1.0f;
    float settle_gyro_max = 0.5f;

    // Phát voice "đã khóa đứng" TRƯỚC, chờ ngần này rồi mới ép cứng (người đỡ kịp phản ứng).
    // Khi đến từ đi bộ, robot chờ DƯỚI POLICY nên vẫn tự giữ thăng bằng.
    float stand_lock_warn_s = 1.5f;

    // Sau khi huỷ điệu nhảy, khóa lệnh "khóa cứng" trong ngần này giây — tránh bấm 0 hai
    // lần quá nhanh làm robot gồng cứng giữa sàn khi chưa ai kịp tới đỡ.
    float dance_abort_lock_block_s = 2.0f;

    // Sau khi VỪA vào STAND LOCK, chặn lệnh "ngồi" (L2+Trái) trong ngần này giây — chống bấm
    // dội / kẹt phím / gói tay cầm nhiễu khiến robot vừa đứng xong lại ngồi ngay lập tức.
    float stand_lock_sit_block_s = 1.0f;

    // Giới hạn tốc độ thay đổi góc khớp policy (khuyến nghị >= 15)
    float policy_rate_limit = 40.0f;

    // Bộ phát hiện ngã
    bool  fall_enabled        = true;
    float fall_tilt_deg       = 50.0f;
    float fall_flip_tilt_deg  = 30.0f;
    float fall_flip_gyro      = 4.0f;   // hạ 6->4: bắt lurch bàn giao (33°+~4.6rad/s)
    float fall_debounce_ms    = 30.0f;

    // Lớp an toàn tốc độ góc khớp: max|dq| trên 24 khớp vượt limit liên tục quá debounce
    // -> coi như "vung loạn" -> Damping (giống ngã). Ngưỡng tạm cao để khỏi báo oan giữa bài.
    bool  joint_speed_guard_enabled = true;
    float joint_speed_limit         = 25.0f;   // rad/s
    float joint_speed_debounce_ms   = 30.0f;

    // Cấu hình ngồi ghế (phím 9 / giữ L2+Trái, và tự động khi pin cạn — KHÔNG AI ĐỠ, phải tự cân bằng)
    // Tư thế đích: gập hông + gối. Thân ĐỔ VỀ TRƯỚC và tay VƯƠN RA để giữ trọng tâm
    // trên bàn chân trong suốt lúc hạ (nếu giữ thân thẳng đứng, trọng tâm lùi ra sau gót -> ngã ngửa).
    //
    // HAI ĐỘ SÂU TÁCH BẠCH (bài học từ lần ngã khi trộn lẫn 2 khái niệm này):
    //  - "desc" (sit_hip_deg/sit_knee_deg/sit_spread): tư thế lúc HẠ, KHI CHƯA CHẮC GHẾ ĐÃ ĐỠ
    //    -> PHẢI tự cân bằng được (đã kiểm MuJoCo: 150/100 + lean58 -> biên +9.2cm, xem
    //    docs/PLAN_sit_balanced.md). ĐỪNG đổi các số này nếu chưa re-verify bằng mô phỏng.
    //  - "rest" (sit_rest_*): tư thế NGỒI CUỐI sau khi ghế ĐÃ đỡ trọng lượng (đo trên robot
    //    thật, limp, đã ngồi ổn) — nông hơn, dạng chân + xoay mũi chân, tay trên đùi. Chỉ áp
    //    dụng ở pha 2 (giao lực) trở đi, không dùng làm đích cho pha hạ.
    float sit_hip_deg         = 150.0f;  // [DESC] Độ gập hông lúc hạ -> ghế ~43cm. Dải thử: 130-160.
    float sit_knee_deg        = 100.0f;  // [DESC] Độ gập gối lúc hạ. Dải thử: 90-115.
    float sit_spread          = 0.08f;   // [DESC] Dạng chân NHẸ lúc hạ (rad, đã kiểm an toàn).
    float sit_rest_hip_deg    = 87.0f;   // [REST] Độ gập hông CUỐI, đo thực trên ghế (ghế đã đỡ).
    float sit_rest_knee_deg   = 82.0f;   // [REST] Độ gập gối CUỐI, đo thực.
    float sit_rest_spread     = 0.45f;   // [REST] Dạng chân CUỐI, đo thực (~26°, ghế đã đỡ nên an toàn).
    float sit_rest_hip_yaw    = 0.45f;   // [REST] Xoay mũi chân ra CUỐI, đo thực (~26°).
    float sit_lean_deg        = 58.0f;   // Đổ thân về trước khi hạ. Dải thử: 40 - 70.
    float sit_seated_lean_deg = 20.0f;   // Đổ thân còn lại sau khi mông chạm ghế.
    float sit_arm_forward     = -1.5f;   // Vai vươn ra trước khi hạ (rad, âm = ra trước).
    float sit_arm_elbow       = 0.3f;    // Khuỷu duỗi bớt khi vươn tay (rad).
    // Tư thế tay CUỐI khi ngồi hẳn = TAY ĐẶT TRÊN ĐÙI (đo thực): vai buông ~0, khuỷu cong nhẹ.
    float sit_seated_arm_pitch = 0.0f;   // Vai lúc ngồi hẳn (rad). 0 = buông thẳng xuống.
    float sit_seated_arm_elbow = 0.42f;  // Khuỷu lúc ngồi hẳn (rad) để bàn tay chạm đùi.
    float sit_ankle_gravity_gain = 0.4f; // Bám mặt đất bằng IMU. 0 = tắt, 1 = ép phẳng tuyệt đối.
    float sit_descent_time_s  = 4.0f;
    float sit_settle_time_s   = 1.5f;    // Thời gian giao trọng lượng cho ghế + thu tay.
    float sit_hold_s          = 0.5f;
    bool  sit_release_after   = false;
    float sit_kp_leg          = 200.0f;
    float sit_kd              = 3.0f;
    float sit_rate_limit      = 3.0f;
    float sit_gather_time_s   = 1.5f;

    // Đứng lên / Nằm xuống (L2+X) — phát lại quỹ đạo ghi từ built-in (xem
    // tools/record_motion.cpp). PD thuần, KHÔNG có policy cân bằng, nên lần chạy
    // thật đầu tiên phải có người đứng đỡ. Đường dẫn tương đối proj_dir.
    std::string getup_motion_file   = "motions/getup.npz";
    std::string liedown_motion_file = "motions/liedown.npz";
    float getup_kp_leg    = 220.0f;
    float getup_kp_waist  = 200.0f;
    float getup_kp_arm    = 60.0f;
    float getup_kd        = 4.0f;
    float getup_rate_limit    = 6.0f;
    float liedown_rate_limit  = 4.0f;
    float getup_blend_time_s   = 0.6f;   // nội suy êm từ tư thế hiện tại sang frame đầu clip
    float liedown_blend_time_s = 0.6f;
    float getup_speed     = 1.0f;        // tốc độ phát lại (1.0 = đúng nhịp lúc ghi)
    float liedown_speed   = 1.0f;
    float getup_liedown_block_s = 1.0f;  // chặn bấm dội sau khi vừa xong 1 chiều
    // Bù cổ chân theo IMU khi phát lại (giữ lòng bàn chân phẳng với sàn dù thân lệch so
    // với lúc ghi). 0 = tắt, 0.4-0.6 nên dùng. Cần file .npz có 'torso_quat' (ghi bằng
    // bản record_motion mới); file cũ không có -> tự tắt.
    float getup_ankle_gravity_gain = 0.5f;
    // Ngưỡng nghiêng thân (độ, so với phương đứng) để IMU coi robot là ĐANG NẰM.
    // Dùng cho L2+X chung: đang nằm -> chuẩn bị/đứng dậy, đang đứng -> nằm xuống.
    // Nằm thật ~82°, đứng ~0-2°; 60° là vùng đệm an toàn.
    float lying_tilt_deg = 60.0f;
    std::string voice_get_up   = "";     // rỗng = không phát
    std::string voice_lie_down = "";

    // Chế độ an toàn khi mất điều khiển
    bool  safe_stop_enabled   = true;
    float safe_stop_debounce_ms = 300.0f;

    // ── P0-1: cổng chống xung đột built-in (kDisarmed) ──
    // Bật cổng: run_r1 KHÔNG publish gì tới khi built-in (bo .161) đã nhả quyền.
    // false = hành vi cũ (CHỈ dùng khi chắc chắn đã vào dev mode thủ công trước).
    bool  arm_gate_enabled        = false;   // mặc định TẮT tới khi verify trên robot
    bool  arm_require_button      = true;    // lớp 1: phải giữ R1+R2 mới arm (chống boot-race)
    float arm_hold_s              = 5.0f;    // giữ R1+R2 liên tục bấy nhiêu = ý định người
    float arm_silence_ms          = 400.0f;  // lớp 2: built-in im lowcmd bấy nhiêu = đã nhả
    bool  arm_require_seen_builtin = true;   // lớp 2-bis: chỉ tin "im" sau khi ĐÃ nghe built-in
    int   arm_min_foreign_seen    = 5;       // số gói lowcmd built-in tối thiểu phải thấy
    // Ngắt xung đột: chỉ nhả quyền khi thấy >= arm_conflict_min gói lạ trong cửa sổ ms này.
    // Chống 1 gói lạc gây nhả oan; built-in thật phun ~621Hz nên vượt ngưỡng trong vài ms.
    int   arm_conflict_min        = 3;
    float arm_conflict_window_ms  = 150.0f;
    // false = phát hiện conflict thì CHỈ log, KHÔNG nhả quyền (dùng khi chắc chắn built-in đã
    // tắt hẳn — để chạy/test dù có self-echo). true = nhả quyền như cũ (an toàn mặc định).
    bool  arm_conflict_release    = true;
    // Nếu sau bao nhiêu giây không thấy gói built-in nào → bỏ qua điều kiện
    // arm_require_seen_builtin (robot đã ở Dev Mode từ trước khi service start).
    // 0 = tắt bypass. Khuyến nghị: 15.0s (boot bình thường built-in phát < 1s).
    float arm_no_builtin_timeout_s = 15.0f;

    // ── P0-3: giám sát pin (topic RIÊNG rt/lf/bmsstate) ──
    bool  battery_monitor_enabled = true;
    std::string battery_topic     = "rt/lf/bmsstate";
    int   battery_warn_pct        = 20;
    int   battery_critical_pct    = 8;
    std::string battery_critical_action = "sit";  // sit | damp | voice
    float battery_announce_period_s = 60.0f;
    float battery_stale_s         = 30.0f;

    // Âm thanh tiếng Việt
    bool  voice_enabled    = true;
    float voice_volume     = 0.9f;
    int   voice_speaker_id = 0;
    std::string voice_startup    = "Máy tính phát triển đã sẵn sàng";
    // Hoãn voice_startup ngần này giây sau khi arm, tránh đè voice "Development Mode" của
    // built-in (phát khi giữ L2+R2 vào dev mode). 0 = phát ngay.
    float startup_voice_delay_s  = 0.0f;
    std::string voice_stand_lock = "Đã khóa đứng";
    std::string voice_locomotion = "Bật chế độ đi bộ";
    std::string voice_sit_down   = "Đang ngồi xuống";
    std::string voice_safe_stop  = "Kích hoạt chế độ an toàn";
    std::string voice_mimic[7];
    // Các voice này để rỗng "" = không phát (user tự chèn file/câu sau).
    std::string voice_fast_speed = "";   // khi chuyển sang tốc độ nhanh
    std::string voice_slow_speed = "";   // khi chuyển sang tốc độ chậm
    std::string voice_conflict   = "";   // P0-1: phát hiện xung đột, đã nhả quyền
    // (arm xong không cần voice riêng — vào kIdle sẽ phát voice_startup)
    std::string voice_battery_low      = "";  // P0-3: pin thấp (ngưỡng cảnh báo)
    std::string voice_battery_critical = "";  // P0-3: pin cạn (ngưỡng nguy cấp)
    std::string voice_zero_torque      = "";  // L2+Y: xả lực hoàn toàn (kZeroTorque)
    std::string voice_teleop_on        = "";  // L2+Phải: bật teleop (rỗng = không phát)
    std::string voice_teleop_off       = "";  // L2+Phải: tắt teleop

    // Giữ combo tay cầm bao lâu (giây) thì mới kích hoạt các động tác nguy hiểm
    // (L2+X nằm/đứng dậy, L2+Trái ngồi ghế) — chống bấm nhầm.
    float hold_to_trigger_s = 3.0f;

    // --- Overlay động tác tay (gesture) khi đang Locomotion ---
    // Trigger = DOUBLE-CLICK nút R3 KHÔNG kèm modifier: 1=Up 2=Down 3=Left 4=Right 5=A 6=B 7=X 8=Y.
    bool  gesture_enabled        = true;
    std::string gesture_folder   = "policies/gestures";   // chứa slotN.npz (N=1..8) và/hoặc slotN.loop
    float gesture_double_click_s = 0.4f;   // cửa sổ nhận double-click
    float gesture_blend_in_s     = 0.4f;   // thời gian trộn tay lên
    float gesture_retract_s      = 1.0f;   // thời gian thu tay về (thẩm mỹ, khi chủ động tắt)
    float gesture_safety_retract_s = 0.4f; // thu NHANH khi guard nghiêng/rời loco/teleop (an toàn)
    float gesture_balance_kg     = 0.0f;   // feedforward bù -> obs gravity.x (ĐO trong sim; 0=tắt)
    float gesture_balance_kv     = 0.0f;   // feedforward bù -> obs cmd_vx  (ĐO trong sim; 0=tắt)
    float gesture_guard_tilt     = 0.35f;  // rad: nghiêng vượt ngưỡng -> tự thu tay về
    float gesture_guard_gyro     = 3.0f;   // rad/s: gyro vượt ngưỡng -> tự thu tay về
    bool  gesture_demo_wave      = false;  // nạp gesture mẫu (MakeDemoWave) vào slot 1 nếu folder trống
    // Track head_pos (yaw,pitch) tùy chọn trong gesture NPZ. Đầu nằm ngoài policy;
    // các trần này được áp trước LowCmd, còn rate-limit chặn frame dữ liệu lỗi.
    float gesture_head_yaw_max        = 0.8f;  // rad
    float gesture_head_pitch_max      = 0.5f;  // rad
    float gesture_head_rate_limit_rad_s = 1.5f;
    std::string gesture_voice_name = "";   // gesture loop tự chạy khi voice đang phát; rỗng = tắt
    std::string gesture_voice_marker = "/run/hb/voice_speaking";
    float gesture_voice_stale_s  = 1.0f;   // marker cũ hơn ngưỡng này -> coi voice đã dừng/crash
    bool  gesture_voice_auto_start = false; // false: phải double-click A mới bật
    float gesture_voice_speed = 1.0f;      // hệ số tốc độ riêng cho gesture khi nói
    float gesture_voice_move_threshold = 0.05f; // có lệnh locomotion lớn hơn ngưỡng -> tắt tay voice
    float gesture_voice_resume_threshold = 0.02f;
    float gesture_voice_resume_delay_s = 0.3f;
    // Gán động tác theo TÊN cho slot 1..8 (giống kiểu dance_folder): gesture_slot_3: vaytay
    // -> tìm <folder>/vaytay.loop.npz (lặp) rồi <folder>/vaytay.npz (giữ frame cuối).
    // Slot bỏ trống -> fallback tìm theo số: <folder>/3.loop.npz / 3.npz.
    std::string gesture_slot[8];
    // Hệ số tốc độ phát riêng từng slot. 1.0 = fps gốc; giới hạn thực thi 0.5..2.0.
    // Dùng để giảm tốc pha chuyển động của một gesture mà không động vào file npz gốc.
    float gesture_slot_speed[8] = {1.0f, 1.0f, 1.0f, 1.0f,
                                   1.0f, 1.0f, 1.0f, 1.0f};
    // Thời gian trộn tay LÊN / thu tay VỀ riêng từng slot (giây). 0 = dùng mặc định
    // toàn cục gesture_blend_in_s / gesture_retract_s. Chỉnh retract để bớt lung lay
    // theo từng phím (vd battay thu chậm, vaytay thu nhanh) mà không sửa npz.
    float gesture_slot_blend[8]   = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    float gesture_slot_retract[8] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    // --- Teleop thân trên (tay + đầu) qua UDP — chỉ nhánh legacy_83 hiện dùng ---
    bool  teleop_enabled          = false;  // false = không mở socket, teleop tắt hoàn toàn
    int   teleop_udp_port         = 5560;
    float teleop_timeout_ms       = 300.0f; // mất gói quá lâu -> thu tay về policy
    float teleop_blend_in_s       = 0.4f;
    float teleop_retract_s        = 0.5f;
    float teleop_smooth_hz        = 8.0f;    // làm mượt tới hạn target tay/đầu giữa các gói
    float teleop_head_yaw_max     = 1.0f;    // rad: clamp góc đầu (an toàn)
    float teleop_head_pitch_max   = 0.6f;    // rad
    float teleop_arm_kp           = 40.0f;   // chỉ dùng cho tay trong ZERO TORQUE
    float teleop_arm_kd           = 2.0f;
    float teleop_max_rate_rad_s   = 0.30f;   // slew limit tại sole lowcmd owner
    // Khoá cứng mọi khớp KHÔNG thuộc teleop (chân 0-11, eo 12-13) trong ZERO
    // TORQUE teleop. Bản gốc để chúng limp: kp=kd=0. Với robot treo trên giá,
    // chân đung đưa tự do là nhiễu cơ khí lẫn vào đúng thứ đang cần đo, nên bản
    // cô lập này giữ chúng tại chính encoder chốt lúc bóp cò.
    // TẮT theo mặc định: bật lên là bắt đầu cấp dòng cho chân, và số kp/kd dưới
    // đây CHƯA được hardware gate duyệt.
    bool  teleop_lock_others_enabled = false;
    float teleop_lock_kp          = 20.0f;   // giữ vị trí, không phải đỡ tải
    float teleop_lock_kd          = 3.0f;    // = kKdTrain của hông/eo
    float teleop_lock_max_rate_rad_s = 0.20f;
    // L2+Phải phải được giữ liên tục đủ lâu mới đổi trạng thái teleop. Tách riêng
    // khỏi hold_to_trigger_s để sau này có thể chỉnh teleop mà không làm đổi các
    // thao tác ngồi/nằm/đứng.
    float teleop_toggle_hold_s    = 3.0f;

    // Watchdogs
    float state_timeout_ms       = 1000.0f;
    // SDK Unitree dùng 3000 ms để phân biệt mất remote với gián đoạn ngắn.
    float remote_timeout_ms      = 3000.0f;
    // Sau LOST, chỉ nhận lại lệnh khi remote hợp lệ và trung tính đủ lâu.
    float remote_recover_ms      = 200.0f;
    bool  remote_require_neutral = true;
    float x11_release_ms         = 2500.0f;

    // Điệu nhảy (mimic)
    static constexpr int kMaxDances = 7;
    std::string dance_folder[kMaxDances];
    float dance_speed[kMaxDances]  = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};  // Dải bám tốt: 0.6 - 0.85.
    float dance_volume[kMaxDances] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    // Trim pitch riêng cho từng bài nhảy. Nếu không khai báo dance_trim_deg_N,
    // bài N giữ nguyên imu_pitch_trim_deg để tương thích các tuning.yaml cũ.
    float dance_trim_deg[kMaxDances] = {};
    bool  dance_trim_configured[kMaxDances] = {};

    float DanceTrimDeg(int dance_key) const {
        const int idx = dance_key - 2;
        return (idx >= 0 && idx < kMaxDances && dance_trim_configured[idx])
                   ? dance_trim_deg[idx]
                   : imu_pitch_trim_deg;
    }

    int   dance_start_frame = -1;
    int   dance_start_search_frames = 200;
    float mimic_announce_delay_s = 2.0f;    // fallback khi announce KHÔNG có voice
    float mimic_announce_min_s = 0.3f;      // sàn: chờ tối thiểu để voice kịp bắt đầu
    float mimic_announce_timeout_s = 20.0f; // trần an toàn: chờ voice tối đa

    // Soft-start: thời gian policy mimic tự đưa robot từ tư thế đứng vào tư thế mở màn
    // của clip (clip đứng yên trong lúc này, nhạc bật khi xong). Dải thử: 0.8 - 2.0 s.
    float mimic_warmup_s = 1.2f;

    // Soft-stop: khi huỷ / hết điệu, mimic policy tự đưa robot VỀ tư thế đứng trong ngần
    // này giây rồi mới giao cho locomotion. Không có nó, locomotion nhận nguyên tư thế nhảy
    // (sâu, bất đối xứng) -> đạp loạn để gượng. Đây là MỐC TRẦN. Dải thử: 0.8 - 2.0
    float mimic_cooldown_s = 1.0f;

    // Bàn giao mimic->locomotion theo điều kiện: đứng đủ yên (nghiêng ngang < tilt VÀ
    // |gyro| thân < gyro) sau min_s -> giao ngay; nếu KHÔNG yên thì mimic GIỮ default
    // tới khi yên, trần cứng = mimic_handover_max_s (tránh dump robot còn trớn/nghiêng
    // sang locomotion -> ngã, vd doremon kết thúc giữa động tác).
    // REVERT hành vi cũ (ép giao đúng lúc cooldown xong): đặt mimic_handover_max_s = mimic_cooldown_s.
    float mimic_handover_tilt  = 0.12f;  // sin(nghiêng ngang), ~7°
    float mimic_handover_gyro  = 0.5f;   // rad/s
    float mimic_handover_min_s = 0.3f;   // cooldown tối thiểu trước khi cho giao sớm
    float mimic_handover_max_s = 3.0f;   // TRẦN cứng: chờ yên tối đa bao lâu rồi vẫn giao (>= cooldown_s)

    // Telemetry sim2real: tự mở một CSV cho mỗi điệu, ghi trên writer thread riêng.
    bool  mimic_telemetry_enabled = true;
    int   mimic_telemetry_hz = 100;       // 1..500; 100 Hz đủ phân tích rung >5 Hz.
    float mimic_telemetry_post_s = 5.0f;  // giữ log sau handover để đo transient.
    std::string mimic_telemetry_dir = "logs/telemetry";
    
    // Contract quyết định cách dựng observation. legacy_83 giữ nguyên đường deploy
    // cũ; r1_unified_v10 dùng 105-D và đưa gesture vào arm reference cho policy.
    // r1_unified_v11_teleop được dành tên trước, nhưng chưa được phép chọn khi
    // teleop live chưa được chuẩn hoá/train nghiệm thu.
    std::string flat_policy_contract = "legacy_83";
    std::string flat_model  = "policy_r1_1.onnx";

    // Unified V10: gesture đứng yên, reference đi qua limiter trước khi vào ONNX.
    // Các trần dưới đây KHÔNG được vượt envelope train của V11 (4 rad/s, 14 rad/s2)
    // để cùng có thể tái sử dụng khi V11 được bật trong tương lai.
    float unified_stationary_cmd_threshold = 0.02f;
    float unified_stationary_hold_s = 0.30f;
    float unified_arm_ref_max_vel_rad_s = 3.0f;
    float unified_arm_ref_max_acc_rad_s2 = 10.0f;

    // Khớp đầu
    float head_yaw_kp = 20.0f, head_yaw_kd = 10.0f;
    float head_pitch_kp = 10.0f, head_pitch_kd = 1.0f;

    // Nạp cấu hình từ file yaml
    bool LoadFromFile(const std::string& path);

private:
    void Apply(const std::string& key, const std::string& value, bool& known);
    bool LoadFromFileImpl(const std::string& path, std::set<std::string>& loading);
    bool Validate() const;
};
