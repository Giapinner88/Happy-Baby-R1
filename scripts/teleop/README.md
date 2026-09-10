# Entrypoint teleop R1

Lệnh chuẩn: `make teleop HOST_IP=<IP-workstation>`.
Xem [runbook sim](../../docs/teleop/r1_quest3_teleop_sim.md) hoặc
[runbook hardware](../../docs/teleop/r1_quest3_teleop_hardware.md) để cài đặt,
calibrate, chạy và dừng. `make help` liệt kê các lệnh.

## Đường live

| File | Vai trò |
|---|---|
| `run_t007_upper_body_pilot.py` | Launcher chuẩn; upstream mặc định, coupled opt-in |
| `quest_bridge.py` | Quest → command JSONL, environment `tv` |
| `run_r1_upstream_ik_stream.py` | Initial-head anchor → vendor IK, environment `tv` |
| `run_r1_quest3_live.py` | Áp q trong Isaac, environment `unitree_sim_env` |
| `run_r1_quest3_hardware.sh` | Launcher hardware |
| `run_r1_quest3_hardware_targets.py` | Giới hạn q và chuyển thứ tự khớp cho sidecar |

Simulator áp q upstream; hardware có thêm limiter, source alignment và envelope.
Code dùng chung thuộc `teleop/r1/`, robot sidecar thuộc `hardware/teleop/`.
[Config T007](../../experiments/r1_teleop/quest3_sim_v1/T007/config/README.md)
phân biệt baseline với các đối chứng.

## Capture và replay

- `capture_quest_transport.py`: capture T001-A; `run_t001_b_pilot.py`: sim head-only.
- `replay_commands_as_live.py`: phát lại trace; `make_preflight_command_stream.py`: input tổng hợp, không phải evidence.
- `solve_r1_t007_*.py`: giải offline; `run_r1_t007_mujoco_replay.py`: replay MuJoCo.
- `run_r1_t002_*` đến `run_r1_t006_*`: protocol đối chứng theo
  [study plan](../../experiments/r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md).

Launcher tự cấp run ID và không ghi đè run cũ. Capture thiếu `--evidence`
ghi vào `results/smoke/`; config snapshot của từng run là nguồn diễn giải kết quả.

## Phân tích

`plot_r1_quest3_telemetry.py` xuất position/velocity/acceleration đầu và wrist;
không lấy đạo hàm qua gap > 0.2 s. `render_r1_t007_mujoco_replay.py` render từ
run đã có, không giải lại controller; manifest lưu hash nguồn và video.
Dùng `--help` của từng script để chọn input/output; các công cụ này dùng `r1_env`.

Các giới hạn Isaac, cert, deadman và thao tác vận hành được ghi tập trung trong
runbook. Phương pháp và khác biệt evidence nằm ở
[docs/teleop](../../docs/teleop/README.md).
