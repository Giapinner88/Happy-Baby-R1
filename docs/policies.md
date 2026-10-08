# Policy và tích hợp kết quả từ các nhánh

Nhánh này chỉ nhận **kết quả** (ONNX/NPZ, cấu hình, motion) từ các nhánh năng lực. Mã train, mô phỏng và thí nghiệm ở lại nhánh gốc.

| Nguồn | Đưa vào đâu | Trạng thái |
| --- | --- | --- |
| `loco` (đi bộ) | `controller/policies/locomotion/flat_plus/` + `controller/config/locomotion.yaml` | Chưa có policy nào đạt để đưa vào |
| dance / mimic (GMR) | `controller/policies/dance/<tên>/` + `controller/config/dance.yaml` | Đang dùng; xem §4 |
| `teleop` | `teleop/` | Dùng đường chạy trên robot (H4). Mã của nhánh teleop chưa hỗ trợ H4 nên chưa đưa vào |
| `vla` (dataset/LeRobot) | — | Chưa tích hợp |

## 1. Chọn policy đang chạy

- Đi bộ: `controller/config/locomotion.yaml` → `flat_policy_contract` + `flat_model` (phải khớp nhau).
- Nhảy: `controller/config/dance.yaml` → `dance_2` … `dance_8` (slot ứng với nút trong [controls.md](controls.md)).
- Toàn vẹn: `controller/policies/locomotion/flat_plus/SHA256SUMS` phải chứa **mọi** `.onnx` trong thư mục (test `policy_artifact_manifest_test`). `integration/config/model_manifest.conf` ghi SHA của model đang chọn; `make deploy` cập nhật file này.

## 2. Hợp đồng locomotion

| Contract | Input | Quan sát |
|---|---:|---|
| `flat_plus_h4_v1` | 332D | 4 frame 83D chuẩn, term-major, cũ → mới |
| `flat_plus_h5_v1` | 415D | 5 frame 83D (runtime hỗ trợ; chưa có artifact trong repo) |
| `flat_plus_gait_h4_v1` | 335D | H4 + one-hot `STAND,WALK,W2S` hiện tại |
| `legacy_83` | 83D | `policies/flat/`, giữ để rollback |

Model nào cũng phải qua kiểm tra shape ONNX, metadata contract, metadata gait FSM và metadata PD trước khi preflight đạt. Model `flat_plus_*` không hỗ trợ overlay tay gesture kiểu legacy.

## 3. Đưa một policy đi bộ mới vào

1. Copy `.onnx` (đã có metadata contract khi export) vào `controller/policies/locomotion/flat_plus/`; xem metadata bằng `controller/scripts/dump_onnx_metadata.py`.
2. Thêm dòng `sha256sum <file>` vào `SHA256SUMS`.
3. Đổi `flat_policy_contract` / `flat_model` trong `locomotion.yaml`; kiểm thủ công:
   - `gait_period_s` bằng `period` của phase obs lúc train model đó;
   - trần tốc độ `slow_*` / `fast_*` nằm trong dải lệnh đã train.
4. Cập nhật tên model được ghim trong `controller/tests/tuning_include_test.cpp` và `controller/tests/config_reader_test.sh` (cố ý ghim để bắt đổi model ngoài ý muốn).
5. `make test` → `make preflight` (trên robot) hoặc `make deploy-policy` (từ máy dev). Ghi nguồn: nhánh, run, checkpoint, SHA vào log buổi thử.

Mỗi file `locomotion_*.example.yaml` trong `controller/config/` là một bộ giá trị đã kiểm cho từng model; test `flat_plus_profiles_preflight_test` chạy preflight cho từng bộ.

## 4. Đưa một điệu nhảy mới vào

1. Tạo `controller/policies/dance/<tên>/` gồm `policy.onnx`, file `.npz` chuyển động (`joint_pos`/`joint_vel` `(frames, 24)`, `body_quat_w` `(frames, N, 4)` wxyz, 50 Hz) và nhạc nếu có. Policy thiếu 4 trường metadata (`default_joint_pos`, `action_scale`, `joint_stiffness`, `joint_damping`) thì chép từ policy cùng hợp đồng bằng `controller/scripts/backfill_onnx_metadata.py`.
2. Gán vào slot trong `dance.yaml` (`dance_N`, `dance_speed_N`, `dance_volume_N`, `dance_trim_deg_N`). `dance_start_frame: -1` để tự chọn frame bắt đầu mượt nhất.
3. `make preflight`: phải thấy `Loaded Mimic N`. Cảnh báo `default_joint_pos LECH` nghĩa là điệu được train trên thế hệ policy đi bộ khác.
4. Từ máy dev: `make deploy-policy ARGS=--with-dance`.

**Hiện trạng (2026-10-08):** `dance.yaml` gán slot 4–8 cho 5 thư mục ngày 2026-10-05 (`xexe_goc_friction15_p1…p4`, `r1nhayngan1_2_goc_matched_friction_20k_20261005`) **không có trong repo**. Preflight bỏ qua các slot này, nên nút tương ứng không làm gì cho tới khi đưa artifact vào hoặc đổi slot.
