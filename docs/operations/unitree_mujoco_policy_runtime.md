# Unitree MuJoCo Policy Runtime
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-006
**Author:** Integration Lead
**Status:** Draft / Working

Tài liệu này chuẩn hóa cách chạy policy nội bộ của Happy-Baby-R1 trên Unitree MuJoCo mà vẫn giữ `third_party/unitree_mujoco` sạch theo upstream.

## 1. Ranh giới thư mục

| Thư mục | Vai trò |
| :--- | :--- |
| `third_party/unitree_mujoco` | Vendor upstream từ `https://github.com/unitreerobotics/unitree_mujoco.git`; không chứa script local |
| `sim/unitree_mujoco_policy` | Script vận hành nội bộ: policy runner, simulator glue, logger, replay helper |
| `data/models/unitree_mujoco_policy` | ONNX policy và motion CSV |
| `data/sim_state_logs` | CSV trạng thái sinh ra khi chạy policy |
| `scripts/run_unitree_mujoco_policy.py` | Launcher ghép simulator và policy trên cùng DDS domain/interface |

Không sửa trực tiếp `third_party/unitree_mujoco/simulate_python` để thêm logic policy. Nếu cần thay đổi hành vi chạy policy, sửa hoặc thêm file trong `sim/unitree_mujoco_policy`.

## 2. Điều kiện môi trường

Máy chạy policy cần có Conda env `r1_env` với các package chính:

```bash
python -c "import mujoco, onnxruntime, pygame, unitree_sdk2py; print('policy env OK')"
```

Nếu chạy DDS loopback trên máy local, dùng interface `lo` và domain riêng để tránh lẫn với buổi vận hành thật.

## 3. Chạy policy G1 mặc định

Từ root repo:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98.py \
  --domain-id 1 \
  --interface lo
```

Launcher sẽ tự chọn `data/models/unitree_mujoco_policy/policy98.onnx` cho `run98.py`.

## 4. Chạy policy khác

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run480.py \
  --policy-onnx policy480.onnx \
  --domain-id 1 \
  --interface lo
```

Với policy dance:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run_dance.py \
  --policy-onnx policy_dance.onnx \
  --motion-csv G1_Take_102.bvh_60hz.csv \
  --domain-id 1 \
  --interface lo
```

Tên file tương đối trong `--policy-onnx` và `--motion-csv` được resolve từ `data/models/unitree_mujoco_policy`.

## 5. Kiểm tra kết quả

Sau khi chạy, kiểm tra CSV mới:

```bash
ls -lh data/sim_state_logs
wc -l data/sim_state_logs/*.csv
```

Kỳ vọng:

* Policy log báo đã load ONNX từ `data/models/unitree_mujoco_policy`.
* Simulator và policy dùng cùng `DOMAIN_ID` và `INTERFACE`.
* CSV trong `data/sim_state_logs` có nhiều hơn 1 dòng.

## 6. Quy tắc an toàn

* Không dùng policy runtime này để điều khiển robot thật nếu chưa có checklist an toàn và người giám sát.
* Không dùng cùng DDS domain/interface với robot thật khi chỉ đang mô phỏng.
* Không commit CSV, ONNX, hoặc wandb runtime log trừ khi có lý do nghiên cứu rõ ràng.
* Trước khi nghi ngờ policy lỗi, kiểm tra trước DDS domain/interface và log `data/sim_state_logs`.

## 7. Liên kết

* Third-party build: [third-party_build.md](third-party_build.md)
* DDS implementation: [dds_implementation.md](dds_implementation.md)
* State/control simulation practice: [practice/08_state_control_sim.md](practice/08_state_control_sim.md)
