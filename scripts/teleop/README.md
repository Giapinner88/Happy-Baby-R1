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
| `run_r1_upstream_ik_stream.py` | Initial-head anchor → vendor R1-A5 IK |
| `run_r1_quest3_live.py` | Áp joint targets trong Isaac |
| `run_r1_quest3_hardware.sh` | Launcher hardware foreground |
| `run_r1_quest3_hardware_targets.py` | Hardware envelope và joint ordering |
| `finalize_r1_run.py` | Sinh metrics/figures/video và chặn run thiếu artifact |
| `export_r1_quest3_hardware_bundle.sh` | Bundle source-only, không kết nối robot |

Code dùng chung thuộc `teleop/r1/`; sidecar/deploy package thuộc
`hardware/teleop/`. [Baseline config](../../experiments/r1_teleop/quest3_sim_v1/baseline/config/README.md)
khóa đối chứng vendor để source do dự án phát triển về sau không ghi đè mốc gốc.

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
