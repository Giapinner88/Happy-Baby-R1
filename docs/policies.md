# Policy và tích hợp kết quả từ các nhánh

Nhánh này chỉ nhận **kết quả** (ONNX/NPZ, cấu hình, motion) từ các nhánh năng lực. Mã train, mô phỏng và thí nghiệm ở lại nhánh gốc.

| Nguồn | Đưa vào đâu | Trạng thái |
| --- | --- | --- |
| `loco` (đi bộ) | `controller/policies/locomotion/<tên>/` (bundle) + `controller/config/locomotion.yaml` | `u2a_seed42` (U2a, HB pose) là `teleop_policy` từ 2026-10-16, thay `u1_seed42` (U1, còn trong repo để rollback); chưa có đánh giá nào cho run này (E1/E2/E3/push lỗi hoặc chưa xong), chưa chạy robot (xem README trong bundle) |
| dance / mimic (GMR) | `controller/policies/dance/<tên>/` + `controller/config/dance.yaml` | Đang dùng; xem §4 |
| `teleop` | `teleop/` | Dùng đường chạy trên robot (H4). Mã của nhánh teleop chưa hỗ trợ H4 nên chưa đưa vào |
| `vla` (dataset/LeRobot) | — | Chưa tích hợp |

## 1. Chọn policy đang chạy

- Đi bộ: mỗi policy là một **bundle** `controller/policies/locomotion/<tên>/` gồm `policy.onnx` + `policy.yaml` (`contract`, `sha256`). Chọn trong `controller/config/locomotion.yaml`:
  - `walk_policy: <tên>` — policy chạy khi chọn R2+A;
  - `teleop_policy: <tên>` — policy chạy khi chọn R2+X; bỏ trống = dùng chung `walk_policy`. Hai phím chỉ chọn policy; teleop hành xử như nhau ở cả hai (`teleop_lock_base` trong `gestures.yaml`). Policy có contract hỗ trợ H4 (`flat_plus_gait_h4_v1`, `r1_wb_legs_v1_blind`) là teleop được; contract khác thì START bị từ chối ở phím đó, không chặn khởi động.
  - Hai bundle khác nhau phải trùng `default_joint_pos` của tay; chân + eo được lệch ≤ 0.10 rad (đổi chế độ blend `blend_time_s`), lệch hơn thì run_r1 từ chối khởi động. `./hb policy list` cho biết bundle nào đang được chọn.
- Route cũ `flat_model` + `flat_policy_contract` vẫn chạy được để rollback/so sánh (đọc từ `policies/flat/` hoặc `policies/locomotion/_archive/flat_plus/`); không được khai cùng lúc với `walk_policy`.
- Nhảy: `controller/config/dance.yaml` → `dance_1` … `dance_7` (slot ứng với nút trong [controls.md](controls.md)).
- Toàn vẹn: `run_r1` kiểm `sha256` trong `policy.yaml` của cả hai bundle lúc khởi động; test `policy_artifact_manifest_test` kiểm mọi bundle và `_archive/flat_plus/SHA256SUMS`. `integration/config/model_manifest.conf` ghi SHA của walk policy; `./hb deploy` cập nhật file này.

Tên bundle hỗ trợ `nhóm/tên`, ví dụ `walk_policy: armfree/nomirror_d20`; tên phẳng cũ vẫn dùng như trước. Bốn bản ArmFree checkpoint 14999 được gom trong [`controller/policies/locomotion/armfree/`](../controller/policies/locomotion/armfree/README.md): `nomirror_d20`, `nomirror_d25`, `speed20_oldaction_d20`, `speed20_nominal_d20`. `./hb policy list` liệt kê đường dẫn đầy đủ để chép vào config. Việc thêm nhóm này giữ nguyên lựa chọn active và các giá trị tuning.

Bốn bản này yêu cầu FSM `flat_plus_gait_fsm_legs12_stand_recovery_v1`: RMS trên 12 khớp chân, settle 0.5 s; phase `episode_clock_command_gated_v1`: thời gian từ reset, frame đầu t=0, thời gian vẫn chạy ở STAND, sin/cos bằng 0 nếu norm lệnh ba chiều <0.1. Runtime bật từng nhánh bằng metadata; các bundle cũ giữ cách xử lý cũ. Damping/action/default pose lấy từ metadata đúng run, không lấy nhầm của policy khác. Đã có bằng chứng mô phỏng và kiểm tra contract; chưa nghiệm thu trên robot.

## 2. Hợp đồng locomotion

| Contract | Input | Quan sát |
|---|---:|---|
| `flat_plus_h4_v1` | 332D | 4 frame 83D chuẩn, term-major, cũ → mới |
| `flat_plus_h5_v1` | 415D | 5 frame 83D (runtime hỗ trợ; chưa có artifact trong repo) |
| `flat_plus_gait_h4_v1` | 335D | H4 + one-hot `STAND,WALK,W2S` hiện tại |
| `legacy_83` | 83D | `policies/flat/`, giữ để rollback |
| `r1_wb_legs_v1_blind` | 71D → **12** | 59 phần tử đầu của frame 83D + 12 raw action; policy chỉ điều khiển chân, eo giữ default, tay thuộc teleop/default (không mask obs tay). Chỉ qua route bundle |

Model nào cũng phải qua kiểm tra shape ONNX, metadata contract, metadata gait FSM và metadata PD trước khi preflight đạt. `r1_wb_legs_v1_blind` còn kiểm `observation_layout`, `joint_names`, `action_joint_names` và `gait_period_s` == `locomotion.yaml`; test `wb_legs_golden_test` so obs/action/target với golden Python (`controller/scripts/make_wb_legs_golden.py`). Model `flat_plus_*` không hỗ trợ overlay tay gesture kiểu legacy.

## 3. Đưa một policy đi bộ mới vào

1. Xem metadata ONNX (`controller/scripts/dump_onnx_metadata.py`), rồi tạo bundle:
   `./hb policy add <file.onnx> <tên> [--contract C]` (mặc định `flat_plus_gait_h4_v1`). Dòng `teleop_h4` trong `policy.yaml` cũ không còn tác dụng và được bỏ qua. Script chép file thành `policy.onnx` và ghi `sha256` vào `policy.yaml`.
2. Ghi provenance (nhánh, run, checkpoint, đánh giá) vào `README.md` trong bundle.
3. Đổi `walk_policy` hoặc `teleop_policy` trong `locomotion.yaml`; kiểm thủ công:
   - `gait_period_s` bằng `period` của phase obs lúc train model đó (dùng chung cho cả hai policy);
   - trần tốc độ `slow_*` / `fast_*` nằm trong dải lệnh đã train.
4. Cập nhật tên bundle được ghim trong `controller/tests/tuning_include_test.cpp` và `controller/tests/config_reader_test.sh` (cố ý ghim để bắt đổi model ngoài ý muốn).
5. `./hb test` → `./hb deploy` từ máy trạm (code C++ và policy đi cùng một lệnh). Ghi nguồn: nhánh, run, checkpoint, SHA vào log buổi thử.

**Đổi nóng giữa hai policy** (R2+A ↔ R2+X khi đang LOCOMOTION) chỉ xảy ra khi tay đã thu và gait ở STAND ổn định liên tục `policy_swap_dwell_s`; quá `policy_swap_timeout_s` thì huỷ. Policy mới được reset history và blend `blend_time_s` từ target vừa gửi. Đường chuyển policy→policy này **chưa có bằng chứng sim/robot**; lần đầu với hai model khác nhau phải chạy trên giá treo (xem [safety.md](safety.md)).

Mỗi file `locomotion_*.example.yaml` trong `controller/config/` là một bộ giá trị đã kiểm cho từng model trong `_archive/flat_plus/` (route cũ); test `flat_plus_profiles_preflight_test` chạy preflight cho từng bộ.

## 4. Đưa một điệu nhảy mới vào

1. Tạo `controller/policies/dance/<tên>/` gồm `policy.onnx`, file `.npz` chuyển động (`joint_pos`/`joint_vel` `(frames, 24)`, `body_quat_w` `(frames, N, 4)` wxyz, 50 Hz) và nhạc nếu có. Policy thiếu 4 trường metadata (`default_joint_pos`, `action_scale`, `joint_stiffness`, `joint_damping`) thì chép từ policy cùng hợp đồng bằng `controller/scripts/backfill_onnx_metadata.py`.
2. Gán vào slot trong `dance.yaml` (`dance_N`, `dance_speed_N`, `dance_volume_N`, `dance_trim_deg_N`). `dance_start_frame: -1` để tự chọn frame bắt đầu mượt nhất.
3. `./hb build && ./hb preflight` trên máy trạm: phải thấy `Loaded Mimic N`. Cảnh báo `default_joint_pos LECH` nghĩa là điệu được train trên thế hệ policy đi bộ khác.
4. Từ máy trạm: `./hb deploy`.

MP3/audio là tùy chọn: thiếu nhạc vẫn nạp và chạy dance bằng ONNX + NPZ, log ghi `no audio; dance runs without music`. Khi có nhạc, runtime phát sau warmup như thường. Thiếu ONNX/NPZ hoặc model/motion không hợp lệ vẫn làm dance bị từ chối.

Dance được đánh số 1–7; đổi từ schema 2–8 bằng cách giảm mọi hậu tố slot (dance, speed, volume, trim, voice_mimic) đi 1 và giữ nguyên thứ tự folder. Config active và replay fixture đã được chuyển đổi. Config cũ bên ngoài repo cần migrate trước khi dùng binary mới.

Config hiện tại dùng `state_motion_guard: true`: tất cả dance tự kiểm tra nghiêng
so với **torso reference vừa phát cho actor**, gồm warmup/clip/cooldown. Lệch
`dance_tracking_error_deg: 45` độ liên tục `dance_tracking_debounce_s: 0.10` giây
thì chốt ngắt toàn bộ motor. Góc so sánh dựa trên hướng trọng lực trong cùng torso,
có tính waist và yaw alignment; quay yaw toàn cục không tự gây ngắt.
Locomotion/RETURNING/stand-up/stand-lock dùng nghiêng tuyệt đối và fast-flip theo
`fall_*`. Tư thế ngồi/nằm/getup không áp yêu cầu luôn đứng thẳng; vẫn kiểm tra
dữ liệu, tốc độ khớp và tiến triển LowState. `motion_safety_state_timeout_s: 0.10`
chặn cả mất gói lẫn gói lặp tick cũ. Tốc độ khớp dùng dữ liệu thô, gồm cả đầu.

Không cần `dance_tilt_guard_N` nữa khi mode này bật; parser vẫn nhận khóa cũ cho
cấu hình legacy (`state_motion_guard: false`). Preflight hiện
`mode=state_reference` và `DANCE slot=N tilt_guard=reference`. Guard đưa robot về
IDLE/damping như L2+B, vẫn armed; không tự tiếp tục khi dựng thẳng hoặc có lại
dữ liệu. Nhả điều khiển rồi ra lệnh mới theo trình tự cũ, không cần DISARM hay
restart. Xem [phục hồi sau guard](safety.md#8-guard-theo-trạng-thái-và-reference).

`dance_home_exit_N: on` cho bài có đoạn cuối HOME (tư thế đứng), độc lập với
guard. Mặc định `off`: đường cooldown hiện tại được giữ nguyên.
Khi bật, runtime chỉ bỏ cooldown sau khi clip chạy hết và đã ổn định liên tục
0,15 s trong 25 frame cuối: tham chiếu cách default locomotion tối đa 0,10 rad,
khớp thật cách default tối đa 0,30 rad, nghiêng <=0,15 rad, gyro <=0,5 rad/s,
max |dq| <=0,6 rad/s. Sau đó reset history locomotion, blend từ target vừa gửi
trong `mimic_home_exit_blend_s` (mặc định 0,25 s, cho phép 0,05–1 s).
Không đạt gate hoặc hủy giữa bài vẫn dùng cooldown; không cắt ngắn clip.
Khi thay ONNX/NPZ cần kiểm lại cả bài và đoạn trả locomotion trước khi bật cờ.
Kết quả MuJoCo và test ứng dụng chưa phải nghiệm thu trên robot thật.
