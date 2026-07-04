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
| `third_party/unitree_rl_mjlab` | Vendor upstream từ `https://github.com/unitreerobotics/unitree_rl_mjlab.git`; chứa policy ONNX mẫu, motion artifact, training/play/deploy code |
| `sim/unitree_mujoco_policy` | Script vận hành nội bộ: policy runner, simulator glue, logger, replay helper |
| `data/models/unitree_mujoco_policy` | ONNX policy và motion CSV |
| `data/sim_state_logs` | CSV trạng thái sinh ra khi chạy policy |
| `scripts/run_unitree_mujoco_policy.py` | Launcher ghép simulator và policy trên cùng DDS domain/interface |
| `scripts/run_unitree_mujoco_official_g1.py` | Launcher chạy controller C++ chính thức của `unitree_rl_mjlab` với MuJoCo bridge local |

Không sửa trực tiếp `third_party/unitree_mujoco/simulate_python` để thêm logic policy. Nếu cần thay đổi hành vi chạy policy, sửa hoặc thêm file trong `sim/unitree_mujoco_policy`.

## 2. Điều kiện môi trường

Máy chạy policy cần có Conda env `r1_env` với các package chính:

```bash
python -c "import mujoco, onnxruntime, pygame, unitree_sdk2py; print('policy env OK')"
```

Nếu chạy DDS loopback trên máy local, dùng interface `lo` và domain riêng để tránh lẫn với buổi vận hành thật.

## 3. Chuẩn bị policy của Unitree RL Mjlab

Sau khi clone `third_party/unitree_rl_mjlab`, symlink policy mẫu của Unitree về thư mục model chuẩn của dự án:

```bash
mkdir -p ~/Projects/Happy-Baby-R1/data/models/unitree_mujoco_policy
cd ~/Projects/Happy-Baby-R1

ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy98.onnx
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy.onnx

ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy_dance.onnx
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/exported/policy.onnx.data \
  data/models/unitree_mujoco_policy/policy.onnx.data
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/params/dance1_subject2.npz \
  data/models/unitree_mujoco_policy/dance1_subject2.npz
```

Kiểm tra ONNX load được:

```bash
load_ml
python - <<'PY'
from pathlib import Path
import onnxruntime as ort

for path in sorted(Path("data/models/unitree_mujoco_policy").glob("*.onnx")):
    session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    print(path.name)
    print("  inputs ", [(i.name, i.shape, i.type) for i in session.get_inputs()])
    print("  outputs", [(o.name, o.shape, o.type) for o in session.get_outputs()])
PY
```

Kỳ vọng hiện tại:

```text
policy98.onnx      obs [1, 98]  -> actions [1, 29]
policy.onnx        obs [1, 98]  -> actions [1, 29]
policy_dance.onnx  obs [1, 154] -> actions [1, 29]
```

## 4. Chạy G1 velocity bằng controller chính thức

Đây là đường ưu tiên khi cần kiểm tra policy mẫu của Unitree. Controller C++ dùng đúng FSM và `ManagerBasedRLEnv` của `unitree_rl_mjlab`, nên đáng tin hơn runner Python nội bộ.

Build controller:

```bash
cmake -S third_party/unitree_rl_mjlab/deploy/robots/g1 \
  -B third_party/unitree_rl_mjlab/deploy/robots/g1/build
cmake --build third_party/unitree_rl_mjlab/deploy/robots/g1/build -j"$(nproc)"
```

Chạy mô phỏng bằng auto FSM:

```bash
python3 scripts/run_unitree_mujoco_official_g1.py \
  --duration 20 \
  --interface lo \
  --auto-sim \
  --auto-passive-seconds 0.5 \
  --auto-fixstand-seconds 3.0 \
  --viewer
```

Kỳ vọng trong log:

```text
FSM: Change state from Passive to FixStand
FSM: Change state from FixStand to Velocity
```

`g1_ctrl` của Unitree hardcode DDS domain `0`, vì vậy launcher này cũng chạy simulator trên `DOMAIN_ID=0`. Dùng `--auto-sim` chỉ cho mô phỏng; khi chạy controller theo upstream với gamepad thật thì bỏ cờ này.

## 5. Chạy policy G1 velocity bằng runner Python thử nghiệm

**Thứ tự an toàn:** luôn bật policy/controller trước simulator. Nếu simulator chạy trước khi policy publish lệnh ổn định, robot trong MuJoCo có thể rơi ngay từ trạng thái spawn ban đầu.

Runner Python dưới đây dùng để debug/log nhanh, nhưng không phải bản exact của `unitree_rl_mjlab`. Nếu robot rung mạnh hoặc có xu hướng ngã, dừng runner Python và quay lại controller C++ ở mục 4.

Từ root repo:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98_2.py \
  --policy-onnx policy.onnx \
  --policy-warmup 2.0 \
  --policy-fade 2.0 \
  --policy-action-clip 0.6 \
  --policy-target-rate-limit 4.0 \
  --domain-id 1 \
  --interface lo
```

Launcher sẽ chạy `sim/unitree_mujoco_policy/unitree_mujoco2.py` và `run98_2.py` cùng `DOMAIN_ID=1`, `INTERFACE=lo`. `run98_2.py` là bản ưu tiên để đứng/đi ổn định vì nó dùng gait clock cố định 20ms, khóa state/cmd giữa các thread, giảm `gait_phase` về 0 khi không có lệnh điều khiển, có pha `FixStand` warmup trước khi bật ONNX policy, và có preset an toàn để fade/clip/rate-limit output ONNX. `run98.py` là bản đơn giản hơn; robot có thể rung vì gait phase vẫn chạy theo wall-clock ngay cả khi đang đứng yên.

```text
Dùng policy: policy.onnx
FixStand warmup 2.0s trước khi bật ONNX policy.
FixStand warmup xong. Bật ONNX policy.
[state-log] logging -> run98_2_<timestamp>.csv
```

Policy velocity trong `unitree_rl_mjlab` không được thiết kế để nhảy thẳng từ spawn sang state Velocity. Luồng deploy chính thức đi qua `Passive -> FixStand -> Velocity`; `FixStand` giữ/ramp robot về `default_joint_pos` trong khoảng 2 giây trước khi RL policy nhận quyền. Nếu robot vẫn rung ở đầu phiên, thử tăng:

```bash
--policy-warmup 3.0
```

Nếu robot rung mạnh hoặc có xu hướng ngã, dừng phiên bằng `Ctrl-C` rồi chạy preset thận trọng hơn:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98_2.py \
  --policy-onnx policy.onnx \
  --policy-warmup 3.0 \
  --policy-fade 4.0 \
  --policy-action-clip 0.35 \
  --policy-target-rate-limit 2.0 \
  --domain-id 1 \
  --interface lo \
  --policy-window
```

Nếu preset thận trọng vẫn rung/ngã, không tiếp tục tăng lực hoặc ép chạy lâu. Khi đó cần chạy controller C++ chính thức trong `third_party/unitree_rl_mjlab/deploy/robots/g1` để phân biệt lỗi policy/model với sai khác trong runtime Python.

Nếu không có gamepad, policy tự chuyển sang chế độ bàn phím. Mặc định launcher chạy headless nên cửa sổ pygame `GAMEPAD CONTROL` không hiện ra. Muốn điều khiển bằng bàn phím trong cửa sổ pygame, chạy:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98_2.py \
  --policy-onnx policy.onnx \
  --policy-warmup 2.0 \
  --policy-fade 2.0 \
  --policy-action-clip 0.6 \
  --policy-target-rate-limit 4.0 \
  --domain-id 1 \
  --interface lo \
  --policy-window
```

Nếu muốn thử mở cả MuJoCo viewer, thêm `--viewer` khi phiên desktop/display hỗ trợ. Không ép `MUJOCO_GL=egl` nếu máy không hỗ trợ EGL.

`scripts/run_unitree_mujoco_policy.py` mặc định start policy trước, chờ `--startup-wait`, rồi mới start simulator. Không đảo thứ tự này khi chạy humanoid policy.

Để tắt cửa sổ điều khiển, bấm nút đóng cửa sổ hoặc nhấn `Esc`. Policy sẽ thoát và launcher sẽ tự dừng simulator. Nếu chạy không có cửa sổ, dùng `Ctrl-C` trong terminal launcher.

## 6. Chạy policy khác

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run98.py \
  --policy-onnx policy98.onnx \
  --domain-id 1 \
  --interface lo
```

Với policy dance của `unitree_rl_mjlab`, ONNX có input shape `[1, 154]`. Chỉ chạy qua script dance khi observation builder của script khớp shape này:

```bash
python3 scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-script run_dance.py \
  --policy-onnx policy_dance.onnx \
  --domain-id 1 \
  --interface lo
```

Tên file tương đối trong `--policy-onnx` và `--motion-csv` được resolve theo
thứ tự:

1. `data/models/unitree_mujoco_policy`
2. `sim/unitree_mujoco_policy`
3. `third_party/unitree_mujoco/simulate_python`

Không dùng lại đường dẫn legacy `sim/mujoco_env`; source/vendor phải được truy
cập qua `third_party`.

## 7. Kiểm tra kết quả

Sau khi chạy, kiểm tra CSV mới:

```bash
ls -lh data/sim_state_logs
tail -n 5 data/sim_state_logs/run98_*.csv
```

Kỳ vọng:

* Policy log báo đã load ONNX từ `data/models/unitree_mujoco_policy`.
* Simulator và policy dùng cùng `DOMAIN_ID` và `INTERFACE`.
* CSV trong `data/sim_state_logs` có nhiều hơn 1 dòng.

## 8. Quy tắc an toàn

* Không dùng policy runtime này để điều khiển robot thật nếu chưa có checklist an toàn và người giám sát.
* Không dùng cùng DDS domain/interface với robot thật khi chỉ đang mô phỏng.
* Không commit CSV, ONNX, hoặc wandb runtime log trừ khi có lý do nghiên cứu rõ ràng.
* Trước khi nghi ngờ policy lỗi, kiểm tra trước DDS domain/interface và log `data/sim_state_logs`.
* Nếu runner Python làm robot rung/ngã, không tiếp tục tinh chỉnh bằng cách tăng gain hoặc tăng biên action. Chạy controller C++ chính thức trước để có mốc so sánh.

## 9. Liên kết

* Third-party build: [third-party_build.md](third-party_build.md)
* DDS implementation: [dds_implementation.md](dds_implementation.md)
* State/control simulation practice: [practice/08_state_control_sim.md](practice/08_state_control_sim.md)
