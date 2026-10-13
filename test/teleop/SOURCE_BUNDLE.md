# Nguồn gốc phần operator (Quest) trong thư mục này

Gộp từ bundle `HB/hb-teleop/` (do bạn copy sang máy này, **đã xoá sau khi gộp** — xem cuối file), tạo
**2026-08-18T17:06:44Z** từ repo `Happy-Baby-R1` branch `develop`
trên máy `ubuntu22-X670E-Taichi`.

Toàn vẹn bundle: `sha256sum -c SHA256SUMS` → **249/253 OK**. 4 file không đọc được
đều là file ẩn của git, không ảnh hưởng chức năng:
`hardware/high_level/.gitignore`, `hardware/teleop/.gitignore`,
`third_party/.../televuer/.git`, `third_party/.../televuer/.gitignore`.

## Bundle nằm đâu trong bố cục này

| Từ bundle (`Happy-Baby-R1/…`) | Về đây |
|---|---|
| `scripts/teleop/` (20 script) | `scripts/teleop/` — giữ nguyên tên để `ROOT = parents[2]` trỏ đúng vào thư mục `teleop/` này |
| `assets/R1.urdf` | `src/assets/R1.urdf` (đúng `kinematics.DEFAULT_URDF`), thêm symlink `assets/ -> src/assets` cho đường dẫn `"assets/R1.urdf"` trong profile |
| `experiments/r1_teleop/` | `experiments/r1_teleop/` (profile IK T001/T007) |
| `third_party/xr_teleoperate/` | `third_party/xr_teleoperate/` (televuer wrapper) |
| `docs/teleop`, `docs/operations`, `docs/safety`, `decisions/r1_teleop` | `docs/`, `docs/operations/`, `docs/safety/`, `docs/decisions/` |
| `PROJECT_MAP.md` | `docs/PROJECT_MAP_upstream.md` |
| `env_specs/` | `env/` (conda `tv` + `unitree_sim_env`) |
| `hardware/teleop/` | **đã có sẵn** ở đây từ trước; chỉ khác 3 file đã sửa theo chuẩn `controller` |
| `hardware/high_level/` | **KHÔNG gộp** — là bản C++ phân kỳ, `controller/` mới là bản đang chạy (xem mục dưới) |

## Không dùng `hardware/high_level/` của bundle

Bundle mang một bản `hardware/high_level` khác `HB/controller`: config còn là
`tuning.yaml` monolithic (chưa tách `audio/dance/gestures/...`), thiếu nhiều file audio.
`HB/controller` là bản đang deploy. **Nhưng** bundle có 3 thứ về teleop mà
`controller` chưa có — xem `controller/docs/PLAN_teleop_receiver_hardening.md`.

## Chạy

```bash
# đọc trước: docs/operations/r1_quest3_teleop_hardware.md, docs/safety/
./scripts/run_quest_teleop.sh          # dựng PYTHONPATH + ROBOT rồi gọi chuỗi upstream
./scripts/preflight.sh                 # kiểm tra tĩnh, chạy được ở máy dev
```

Hai conda env cần có: `tv` (bridge Quest) và `unitree_sim_env` (IK) — spec ở `env/`.

## Bundle đã xoá (2026-08-19)

Sau khi gộp, `HB/hb-teleop/` (105M) đã được xoá. Kiểm tra trước khi xoá:

- `_external_not_in_repo/policy_goc.onnx` **trùng sha256 tuyệt đối** (`ea46d2eb…`) với
  `controller/policies/flat/policy_goc.onnx` — chính là `flat_model` đang dùng. Không mất gì.
- `hardware/teleop/` đã là thư mục này; `hardware/high_level/` là bản C++ phân kỳ, không dùng
  (3 điểm cần lấy đã ghi ở `controller/docs/PLAN_teleop_receiver_hardening.md`, còn đường
  `RunZeroTorqueTeleop` ghi ở `PLAN_teleop_quest_pipeline.md` mục 3.1 — **đọc 2 file đó trước
  khi port**, vì mã gốc không còn ở máy này).
- `MANIFEST.txt` / `SHA256SUMS` / `README_BUNDLE.md`: nội dung cần giữ đã chép vào file này.

## Đã phân kỳ với upstream `Happy-Baby-R1`

| File | Thay đổi |
|---|---|
| `src/teleop/hardware/high_level_sidecar.py` | thứ tự đầu theo chuẩn `RobotSpec.hpp` (`[yaw, pitch]`), bỏ hoán vị trong `encode_target` |
| `src/teleop/hardware/run_teleop.py` | nhãn `HEAD_INDICES`/`STREAM_JOINT_NAMES` theo chuẩn trên; `pilot_offsets` giữ slot 30, sửa nhãn thành head **pitch** |
| `tests/test_high_level_sidecar.py` | assertion theo thứ tự mới |
| `scripts/teleop/run_r1_quest3_hardware_targets.py` | tách `MODEL_JOINT_NAMES` (thứ tự IK) khỏi `JOINT_NAMES` (thứ tự stream) + `STREAM_FROM_MODEL` khi emit |
| `scripts/run_quest_teleop.sh` | file mới, chỉ có ở HB |
