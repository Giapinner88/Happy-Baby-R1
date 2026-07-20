# Unitree MuJoCo Policy Runtime
**Project:** Unitree - Happy Baby R1
**Document ID:** HB-SPEC-006
**Status:** Working

Tài liệu này chuẩn hóa runtime MuJoCo local cho R1. `third_party/unitree_mujoco`
và `third_party/unitree_rl_mjlab` chỉ là nguồn tham chiếu/upstream, không sửa.
Mọi model, log, cache, kết quả chạy được đưa về `data/`.

## 1. Cấu trúc runtime

| Đường dẫn | Vai trò |
| :--- | :--- |
| `assets/mujoco/unitree_robots/r1/` | MJCF R1 local dùng chung cho MuJoCo runtime và MJLab training |
| `sim/unitree_mujoco_policy/config.py` | Chọn policy, scene, joint order, default pose, gains, action scale |
| `sim/unitree_mujoco_policy/policy_runner.py` | Runner policy duy nhất |
| `sim/unitree_mujoco_policy/unitree_mujoco2.py` | MuJoCo simulator bridge theo kiểu Unitree |
| `sim/unitree_mujoco_policy/unitree_sdk2py_bridge.py` | Cầu nối Unitree DDS lowcmd/lowstate |
| `scripts/bridge/run_unitree_mujoco_policy.py` | Launcher chạy policy runner và simulator cùng DDS domain/interface |
| `data/models/unitree_mujoco_policy/` | ONNX policy deploy |
| `data/runs/unitree_mujoco_policy/` | Log của launcher |
| `data/sim_state_logs/` | CSV state/action log từ policy runner |

Runtime này là R1-only. Không map policy sang G1 và không dùng lại các script
`run98.py`, `run98_2.py`, `run_dance.py`.

## 2. Policy hiện tại

Policy được chọn trong `sim/unitree_mujoco_policy/config.py`:

```python
POLICY_NAME = "r1_velocity"
```

`r1_velocity` bám theo cấu hình deploy R1 velocity của
`third_party/unitree_rl_mjlab/deploy/robots/r1/config/policy/velocity/v0/params/deploy.yaml`:

- `step_dt = 0.02`, tức control 50 Hz.
- Observation dim: `83`.
- Action dim: `24`.
- Observation gồm: gyro, projected gravity, velocity command, gait phase,
  joint position relative, joint velocity, last action.
- Action điều khiển đúng 24 joint R1 của training model. Runtime không còn
  dùng model 29 motor/dummy joint.
- Action scale khớp training: leg/waist `0.15`, ankle `0.3125`,
  shoulder pitch/roll `0.375`, shoulder yaw/elbow/wrist roll `0.4125`.

ONNX mặc định:

```text
data/models/unitree_mujoco_policy/r1_velocity.onnx
```

Sau khi train/export được policy tốt, copy hoặc symlink ONNX vào path trên.
Nếu muốn thêm policy R1 khác, thêm entry mới vào `POLICIES` trong `config.py`,
rồi đổi `POLICY_NAME`.

## 2.1. Đồng bộ model train và runtime

Model R1 chuẩn của workspace nằm tại:

```text
assets/mujoco/unitree_robots/r1/R1.xml
```

File này được sync từ R1 training model của `unitree_rl_mjlab`, rồi bổ sung
actuator/sensor cần cho Unitree MuJoCo bridge. Không sửa `third_party`.

Khi cần refresh asset local:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/assets/sync_r1_mujoco_asset.py
```

`scripts/training/r1_mjlab_train.py` và `scripts/training/r1_mjlab_play.py` monkey-patch task
`Unitree-R1-*` để MJLab dùng chính `assets/mujoco/unitree_robots/r1/R1.xml`.
Kỳ vọng compile hiện tại:

```text
nq=31, nv=30, nu=24, nsensor=77
```

## 3. Chạy simulator bridge không policy

Dùng lệnh này để kiểm tra riêng R1 model + DDS bridge:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/bridge/run_r1_unitree_mujoco_bridge.py \
  --scene scene_hanging.xml \
  --duration 20 \
  --interface lo \
  --domain-id 1 \
  --init-default-q
```

Kỳ vọng log có:

```text
[sim-config] robot=r1 ... hg_idl=True
[init-default-q] State da set theo DEFAULT_Q cua policy.
```

Đây chưa phải test policy. Nếu bridge pass, mới nối ONNX policy vào launcher.

## 4. Chạy R1 policy

Đảm bảo đã có:

```bash
ls -lh data/models/unitree_mujoco_policy/r1_velocity.onnx
```

Chạy smoke runtime:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/bridge/run_unitree_mujoco_policy.py \
  --duration 20 \
  --interface lo \
  --domain-id 1
```

Launcher sẽ:

1. Start `policy_runner.py`.
2. Chờ `--startup-wait`.
3. Start `unitree_mujoco2.py`.
4. Ghi `policy.log` và `sim.log` vào `data/runs/unitree_mujoco_policy/<timestamp>/`.
5. Ghi CSV state/action vào `data/sim_state_logs/`.

Mặc định runner bật policy trực tiếp giống deploy MJLab: không FixStand warmup,
không fade-in action, không raw action clip. Với R1 asset hiện tại, `DEFAULT_Q`
đơn lẻ không phải bộ đứng ổn định trên mặt phẳng; warmup/clip có thể làm robot
ngã trước khi policy kịp cân bằng.

Preset giảm tốc target nếu cần kiểm tra thận trọng hơn:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/bridge/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-target-rate-limit 2.0 \
  --interface lo \
  --domain-id 1
```

Thêm `--policy-window` nếu muốn điều khiển bằng bàn phím/gamepad qua pygame.
Thêm `--viewer` nếu muốn mở MuJoCo viewer.

## 5. Kiểm tra kết quả

Log launcher:

```bash
find data/runs/unitree_mujoco_policy -maxdepth 2 -type f -name '*.log' | sort | tail
```

CSV state/action:

```bash
ls -lh data/sim_state_logs
```

Nếu robot ngã ngay:

- Kiểm tra `policy.log` trước, đặc biệt input/output shape ONNX.
- Kiểm tra `sim.log` có đúng scene R1 local.
- Kiểm tra `sim.log` báo `nu=24`/sensor order R1 nếu vừa sync asset.
- Chạy lại bridge-only ở mục 3 để tách lỗi asset/bridge khỏi lỗi policy.
- Không dùng policy G1 để test R1 runtime.
