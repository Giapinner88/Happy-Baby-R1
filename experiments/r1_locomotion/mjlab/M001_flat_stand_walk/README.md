# M001 — R1 MJLab đứng và đi chậm trên mặt phẳng

M001 là pilot MJLab độc lập; nó không thay đổi task/profile default hay run
IsaacLab đang chạy. Đọc [`experiment.md`](experiment.md) trước khi chạy.

## Chuẩn bị và chỉnh config

Chạy từ repository root. M001 mặc định chọn GPU vật lý `1` theo
`execution.cuda_visible_devices` trong config; xác nhận GPU này đang rảnh:

```bash
cd "$(git rev-parse --show-toplevel)"
nvidia-smi -i 1
PYTHONNOUSERSITE=1 conda run --no-capture-output -n r1_env \
  python scripts/training/r1_mjlab_train.py Unitree-R1-Flat --help
```

Chỉ sửa [`config.json`](config.json) trước khi bắt đầu run; runner sẽ chép nó
thành snapshot bất biến. Các trường `training.*`, `command_distribution.*` và
`capture.*` là các núm chỉnh chính. Không giảm toàn bộ command range xuống
±0.10 m/s: MJLab hiện mask command có norm ≤0.10 thành zero. Vì vậy M001 train
ở ±0.25 m/s và đánh giá riêng tại 0, ±0.05, ±0.10 m/s.

Video dài 250 steps và được ghi mỗi 10.000 environment steps, nên một pilot
3.000 PPO iteration (24 steps/iteration) dự kiến có clip đầu và các clip tiến
trình tiếp theo. Không dùng một clip hay reward curve để kết luận locomotion.

## Validate, dry-run và chạy

```bash
python experiments/r1_locomotion/mjlab/M001_flat_stand_walk/validate_config.py

python experiments/r1_locomotion/mjlab/M001_flat_stand_walk/run.py --dry-run

python experiments/r1_locomotion/mjlab/M001_flat_stand_walk/run.py
```

Mỗi execution tạo ID UTC mới. Nếu cần đặt tên rõ seed/repeat, chọn ID chưa tồn
tại:

```bash
python experiments/r1_locomotion/mjlab/M001_flat_stand_walk/run.py \
  --run-id M001_flat_stand_walk_seed42_repeat1
```

Không chạy đồng thời với IsaacLab trên cùng GPU. Nếu cần chuyển M001 sang GPU
khác, sửa `execution.cuda_visible_devices` trong `config.json` **trước** khi
khởi tạo run; snapshot của run sẽ ghi lại lựa chọn đó. Ví dụ, để dùng GPU 0:

```bash
python experiments/r1_locomotion/mjlab/M001_flat_stand_walk/run.py
```

## Output cần có

```text
experiments/r1_locomotion/mjlab/runs/<run-id>/
├── resolved_config.json, metadata.json, status.json
├── experiment_config.json, experiment_runner_command.txt
├── logs/rsl_rl/r1_velocity/<upstream-run>/
│   ├── model_*.pt
│   ├── events.out.tfevents.*
│   ├── params/{env,agent}.yaml
│   └── videos/train/*.mp4
├── derived/training/{training_scalars.csv,plots/*.png,manifest.json}
└── evidence_completeness.json
```

`training_evidence` trong manifest phải là `complete`. Dù complete, checkpoint
vẫn chưa được collect/promote hoặc dùng ngoài experiment cho đến khi evaluation
có raw trace, metric CSV, video và manifest pass/fail.
