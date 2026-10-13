# Policy và tích hợp kết quả từ các nhánh

Nhánh này chỉ nhận **kết quả** (ONNX/NPZ, cấu hình, motion) từ các nhánh năng lực. Mã train, mô phỏng và thí nghiệm ở lại nhánh gốc.

| Nguồn | Đưa vào đâu | Trạng thái |
| --- | --- | --- |
| `loco` (đi bộ) | `controller/policies/locomotion/<tên>/` (bundle) + `controller/config/locomotion.yaml` | `u1_seed42` (U1) là `teleop_policy`; chỉ có bằng chứng mô phỏng Isaac, chưa chạy robot (xem README trong bundle) |
| dance / mimic (GMR) | `controller/policies/dance/<tên>/` + `controller/config/dance.yaml` | Đang dùng; xem §4 |
| `teleop` | `teleop/` | Dùng đường chạy trên robot (H4). Mã của nhánh teleop chưa hỗ trợ H4 nên chưa đưa vào |
| `vla` (dataset/LeRobot) | — | Chưa tích hợp |

## 1. Chọn policy đang chạy

- Đi bộ: mỗi policy là một **bundle** `controller/policies/locomotion/<tên>/` gồm `policy.onnx` + `policy.yaml` (`contract`, `sha256`, `teleop_h4`). Chọn trong `controller/config/locomotion.yaml`:
  - `walk_policy: <tên>` — policy chạy khi chọn R2+A;
  - `teleop_policy: <tên>` — policy chạy khi chọn R2+X; bỏ trống = dùng chung `walk_policy`. Khi H4 được bật, cả hai bundle phải có `teleop_h4: true` và contract hỗ trợ H4. R2+A yêu cầu lệnh vận tốc về 0 trước START; R2+X cho phép vừa đi vừa teleop.
  - Hai bundle khác nhau phải trùng `default_joint_pos` của tay; chân + eo được lệch ≤ 0.10 rad (đổi chế độ blend `blend_time_s`), lệch hơn thì run_r1 từ chối khởi động. `./hb policy list` cho biết bundle nào đang được chọn.
- Route cũ `flat_model` + `flat_policy_contract` vẫn chạy được để rollback/so sánh (đọc từ `policies/flat/` hoặc `policies/locomotion/_archive/flat_plus/`); không được khai cùng lúc với `walk_policy`.
- Nhảy: `controller/config/dance.yaml` → `dance_2` … `dance_8` (slot ứng với nút trong [controls.md](controls.md)).
- Toàn vẹn: `run_r1` kiểm `sha256` trong `policy.yaml` của cả hai bundle lúc khởi động; test `policy_artifact_manifest_test` kiểm mọi bundle và `_archive/flat_plus/SHA256SUMS`. `integration/config/model_manifest.conf` ghi SHA của walk policy; `./hb deploy policy` cập nhật file này.

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
   `./hb policy add <file.onnx> <tên> [--contract C] [--teleop]` (mặc định `flat_plus_gait_h4_v1`, `teleop_h4: false`; `--teleop` chỉ khi artifact đã được duyệt cho overlay tay H4). Script chép file thành `policy.onnx` và ghi `sha256` vào `policy.yaml`.
2. Ghi provenance (nhánh, run, checkpoint, đánh giá) vào `README.md` trong bundle.
3. Đổi `walk_policy` hoặc `teleop_policy` trong `locomotion.yaml`; kiểm thủ công:
   - `gait_period_s` bằng `period` của phase obs lúc train model đó (dùng chung cho cả hai policy);
   - trần tốc độ `slow_*` / `fast_*` nằm trong dải lệnh đã train.
4. Cập nhật tên bundle được ghim trong `controller/tests/tuning_include_test.cpp` và `controller/tests/config_reader_test.sh` (cố ý ghim để bắt đổi model ngoài ý muốn).
5. `./hb test` → `./hb deploy policy` từ máy trạm (preflight bằng `run_r1` local, đồng bộ cả thư mục bundle walk + teleop, `locomotion.yaml`, `gestures.yaml`, restart khi DISARMED); đổi C++ thì `./hb field`. Ghi nguồn: nhánh, run, checkpoint, SHA vào log buổi thử.

**Đổi nóng giữa hai policy** (R2+A ↔ R2+X khi đang LOCOMOTION) chỉ xảy ra khi tay đã thu và gait ở STAND ổn định liên tục `policy_swap_dwell_s`; quá `policy_swap_timeout_s` thì huỷ. Policy mới được reset history và blend `blend_time_s` từ target vừa gửi. Đường chuyển policy→policy này **chưa có bằng chứng sim/robot**; lần đầu với hai model khác nhau phải chạy trên giá treo (xem [safety.md](safety.md)).

Mỗi file `locomotion_*.example.yaml` trong `controller/config/` là một bộ giá trị đã kiểm cho từng model trong `_archive/flat_plus/` (route cũ); test `flat_plus_profiles_preflight_test` chạy preflight cho từng bộ.

## 4. Đưa một điệu nhảy mới vào

1. Tạo `controller/policies/dance/<tên>/` gồm `policy.onnx`, file `.npz` chuyển động (`joint_pos`/`joint_vel` `(frames, 24)`, `body_quat_w` `(frames, N, 4)` wxyz, 50 Hz) và nhạc nếu có. Policy thiếu 4 trường metadata (`default_joint_pos`, `action_scale`, `joint_stiffness`, `joint_damping`) thì chép từ policy cùng hợp đồng bằng `controller/scripts/backfill_onnx_metadata.py`.
2. Gán vào slot trong `dance.yaml` (`dance_N`, `dance_speed_N`, `dance_volume_N`, `dance_trim_deg_N`). `dance_start_frame: -1` để tự chọn frame bắt đầu mượt nhất.
3. `./hb build && ./hb preflight` trên máy trạm: phải thấy `Loaded Mimic N`. Cảnh báo `default_joint_pos LECH` nghĩa là điệu được train trên thế hệ policy đi bộ khác.
4. Từ máy trạm: `./hb deploy dance`.

**Hiện trạng (2026-10-09, gộp từ HB):** slot 3–7 dùng các thư mục ngày 2026-09-04 → 2026-10-08 (`xexe_trimmed_129obs_goc…`, `xexe_r1motion_goc_fric11to17…`, `xexe_r1motion_meta…`, `nhay3_20260904_trimmed_after_bilateral_tilt_f084`, `r1nhayngan_3_meta…`), đều có trong repo; `dance_8` để trống. `xexe_meta_129obs_fresh…` và `xexe_trimmed_129obs_meta…` có trong repo nhưng chưa gán slot. Các điệu này chưa có log chạy trên robot trong repo này. `make dance-list` liệt kê thư mục đang dùng; `make dance-prune` xem trước (thêm `ARGS=--apply` để chuyển thư mục không tham chiếu sang `_archive/dance/`, `make dance-restore ARGS=<tên>` để đưa lại).
