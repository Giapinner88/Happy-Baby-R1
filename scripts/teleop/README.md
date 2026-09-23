# Entrypoint teleop R1

Lệnh chuẩn là `make teleop HOST_IP=<IP-workstation>`. Xem
[runbook mô phỏng](../../docs/teleop/r1_quest3_teleop_sim.md) hoặc
[runbook hardware](../../docs/teleop/r1_quest3_teleop_hardware.md) trước khi
chạy.

## Pipeline active

| File | Vai trò |
|---|---|
| `run_r1_baseline.py` | Launcher baseline upstream-only |
| `quest_bridge.py` | Quest → command JSONL trong environment `tv` |
| `robot_camera_vuer.py` | Adapter Vuer project-owned cho full-field camera và đổi view |
| `robot_camera_stream.py` | HTTP subscriber latest-only, decode JPEG ngoài control loop |
| `run_r1_camera_transport.py` | Quản lý gateway + SSH loopback tunnel + readiness/evidence |
| `record_robot_camera_zmq.py` | Ghi MP4 + receive timestamp từ TeleImager ZMQ |
| `test_r1_eye_camera.py` | Probe chỉ-đọc camera đầu R1 EDU qua `videohub` DDS |
| `fanout_command_stream.py` | Mirror non-blocking cùng command sang Isaac |
| `compare_sim_hardware_run.py` | So sánh sim/hardware theo shared sequence ID |
| `run_r1_upstream_ik_stream.py` | Initial-head anchor → vendor R1-A5 IK |
| `run_r1_quest3_live.py` | Áp joint targets trong Isaac |
| `run_r1_quest3_hardware.sh` | Launcher hardware foreground |
| `run_r1_quest3_hardware_targets.py` | Hardware envelope và joint ordering |
| `finalize_r1_run.py` | Sinh metrics/figures/video và chặn run thiếu artifact |
| `export_r1_quest3_hardware_bundle.sh` | Bundle source-only, không kết nối robot |

Code dùng chung thuộc `teleop/r1/`; sidecar/deploy package thuộc
`hardware/teleop/`. [Baseline config](../../experiments/r1_teleop/quest3_sim_v1/baseline/config/README.md)
khóa đối chứng vendor để source do dự án phát triển về sau không ghi đè mốc gốc.

Camera đầu R1 được bật mặc định trong `make teleop-hardware`: launcher chạy
gateway `videohub` trên robot, forward HTTP loopback bằng đúng target `ROBOT`
và ghi JPEG gốc vào `results/smoke/<run>/robot_camera_original`. Không nhập
camera IP/port riêng. View bắt đầu ở Quest passthrough; cạnh nhấn **A tay phải**
đổi sang robot view full-field, cạnh tiếp theo quay lại Quest. Video stale tự
gỡ nền robot và trả về passthrough. Cò phải vẫn chỉ là deadman; cò trái vẫn chỉ
là recalibration.

```bash
# Camera R1 + recording + Isaac mirror + hardware (mặc định)
make teleop-hardware HOST_IP=100.95.122.105 ROBOT=100.82.165.36

# Tắt riêng camera robot
HB_ROBOT_CAMERA=0 make teleop-hardware HOST_IP=100.95.122.105 ROBOT=100.82.165.36
```

`ROBOT_CAMERA_WEBRTC_URL=<offer-url>` vẫn là override tương thích và sẽ thay
gateway built-in. Simulation-only `make teleop` không tự mở SSH/camera robot.

Camera đầu của R1 EDU này không xuất hiện dưới `/dev/video*` trên Orin NX và
không dùng Argus. Kiểm tra nguồn ảnh thật trực tiếp trên robot bằng:

```bash
python3 scripts/teleop/test_r1_eye_camera.py \
  --interface eth10 \
  --output-dir /tmp/r1_eye_camera_test
```

Script gọi RPC `videohub` (`VideoClient.GetImageSample`) qua DDS, lưu JPEG cùng
`report.json`, không tạo publisher DDS và không gửi lệnh motor. Nó phải chạy
trên robot; địa chỉ Tailscale và cổng TCP không phải tham số camera.

## Diagnostics và phân tích

- `capture_quest_transport.py`: diagnostic transport; mặc định ghi vào
  `results/smoke/`, không phải baseline evidence.
- `replay_commands_as_live.py`, `make_preflight_command_stream.py`: replay/input
  tổng hợp, không thay thế live evidence.
- `solve_r1_baseline_upstream_ik.py`: chạy upstream solver trên trace đã ghi.
- `plot_r1_quest3_telemetry.py`, `plot_r1_baseline_dynamics.py`: derived analysis;
  không sửa run nguồn.

Launcher baseline tự cấp run ID và không ghi đè run cũ. Run thành công phải có
data, metrics, figures, video và manifest. Dùng `--help` của từng
script cho tham số cụ thể; môi trường và thao tác dừng thuộc hai runbook.
