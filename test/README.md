# test/ — thí nghiệm, không deploy

Thư mục này giữ phần thí nghiệm/mô phỏng gộp từ workspace HB (2026-10-09) để dùng sau.
Nó **không** thuộc runtime robot: `integration/scripts/deploy_stack.sh` chỉ đồng bộ
`controller/`, `integration/`, `voice/`, `voice_presets/`, `teleop/`, và `make test`
không chạy test ở đây.

| Đường dẫn | Nội dung |
| --- | --- |
| `teleop/` | Bản chụp đầy đủ `HB/teleop` (pipeline laptop, mô phỏng IsaacLab/MuJoCo T001–T007, IK/kinematics, plot, `experiments/`, `docs/`, `env/`). `results/` chỉ có trên máy, không vào git. |
| `controller_lab/docs/` | Tài liệu thiết kế/PLAN của `high_level_2` (được comment trong code tham chiếu). |
| `controller_lab/motions/` | Capture chuyển động thô (`captures_*`, `getup_raw.npz`, `liedown_raw.npz`). |
| `controller_lab/policies/loco_teleop_lab/` | Artifact loco-teleop thí nghiệm. |
| `controller_lab/tools/` | `verify_crouch_com.py` (kiểm CoM trong MuJoCo), `dex3_usb_compat/` (shim USB cho dịch vụ Dex3). |
| `controller`, `integration` | Liên kết tới thư mục thật để script trong `test/teleop` tìm đúng `$ROOT/controller/...`. |

Lưu ý:

- Runtime chính thức của teleop là `teleop/` ở gốc repo. `test/teleop/src` là bản đông
  cứng, có thể lệch với runtime.
- `test/teleop/src/teleop/hardware/run_teleop.py` của HB có chế độ ghi lệnh động cơ trực
  tiếp (`--execute-pilot`, `--stream-stdin`). Runtime develop đã cố ý bỏ đường này; không
  dùng nó trên robot ngoài quy trình đã được duyệt trong `docs/safety.md`.
- Chạy test của bản chụp:
  `cd test/teleop && PYTHONPATH=src:scripts:scripts/teleop:.. <python có aiohttp/numpy> -m pytest tests -q`.
