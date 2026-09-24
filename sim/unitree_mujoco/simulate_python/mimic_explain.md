# Quy trình xây dựng `run_mimic.py` — Deploy Motion Mimic Policy cho G1

## 1. Tổng quan

`run_mimic.py` là script deploy policy **motion mimic tracking** — policy học cách bắt chước một chuỗi chuyển động tham chiếu (từ motion capture hoặc keyframe animation).

Pipeline tổng quát:

```
Motion CSV
    │
    ▼ (csv_to_npz.py — mjlab)
Motion NPZ  ──────────────────────────────────────────────────────────┐
    │                                                                  │
    │  [joint_pos, joint_vel]        ← motion_command (58 dims)       │
    │  [body_quat_w] pelvis          ← tính anchor orientation        │
    ▼                                                                  │
Policy ONNX ◄── Observation (154 dims) ──────────────────────────────┘
    │
    ▼
Action (29 dims)  →  target_q = DEFAULT_Q + action × ACTION_SCALE  →  Motor CMD
```

---

## 2. Nguồn gốc Motion Data

### 2.1 Từ CSV đến NPZ

File CSV chứa dữ liệu raw theo format:

```
col 0-2  : base position (x, y, z)
col 3-6  : base quaternion (x, y, z, w)  ← xyzw
col 7-35 : 29 DOF joint positions (rad)
```

Script `csv_to_npz.py` của mjlab **chạy simulation** để:
1. Nạp từng frame CSV vào MuJoCo
2. Lấy `robot.data.joint_pos` (toàn bộ 29 DOF) và `robot.data.body_link_*` (tất cả bodies)
3. Lưu vào NPZ

> **Quan trọng:** `joint_pos` trong NPZ là **absolute joint angles** (rad), **không phải** relative to default.

### 2.2 Cấu trúc NPZ

| Key | Shape | Ý nghĩa |
|-----|-------|---------|
| `joint_pos` | `(T, 29)` | Góc khớp tuyệt đối (rad) |
| `joint_vel` | `(T, 29)` | Vận tốc khớp (rad/s) |
| `body_pos_w` | `(T, 30, 3)` | Vị trí world của 30 body links |
| `body_quat_w` | `(T, 30, 4)` | Quaternion world của 30 bodies **(wxyz)** |

**Body index 0 = pelvis** (root body của robot).

---

## 3. Cấu trúc Observation (154 dims)

Được xác định từ `deploy.yaml` với `has_state_estimation=False`:

| Tên term | Dims | Nguồn |
|----------|------|-------|
| `motion_command` | 58 | `joint_pos_ref (29) + joint_vel_ref (29)` từ NPZ |
| `motion_anchor_ori_b` | 6 | Relative rotation matrix (2 cột đầu) |
| `base_ang_vel` | 3 | IMU gyroscope (rad/s) |
| `joint_pos_rel` | 29 | `q_current - DEFAULT_Q` |
| `joint_vel_rel` | 29 | `dq_current` |
| `last_action` | 29 | Action của bước trước |
| **Total** | **154** | |

> Training với `has_state_estimation=False` loại bỏ `motion_anchor_pos_b` (3 dims) và `base_lin_vel` (3 dims) so với full config (160 dims).

---

## 4. Tính `motion_anchor_ori_b` (phần phức tạp nhất)

### 4.1 Anchor body là `torso_link`

Policy so sánh **torso_link** của robot với **torso_link** của motion tham chiếu, **không phải** pelvis.

### 4.2 Tính torso quaternion từ pelvis

`torso_link` không có IMU trực tiếp. Ta tính từ pelvis + góc 3 khớp eo theo logic trong `State_Mimic.cpp`:

```python
def torso_quat(pelvis_q_wxyz, waist_yaw, waist_roll, waist_pitch):
    q = quat_mul(pelvis_q, angle_axis(waist_yaw,   [0,0,1]))  # joint[12]
    q = quat_mul(q,        angle_axis(waist_roll,  [1,0,0]))  # joint[13]
    q = quat_mul(q,        angle_axis(waist_pitch, [0,1,0]))  # joint[14]
    return q
```

### 4.3 `init_quat` — bù lệch yaw khi bắt đầu

Khi robot và motion có hướng mặt khác nhau lúc bắt đầu:

```python
# Tính 1 lần lúc khởi động
robot_yaw_q = yaw_quat(robot_torso_q0)
ref_yaw_q   = yaw_quat(ref_torso_q0)
init_quat   = quat_mul(robot_yaw_q, quat_inv(ref_yaw_q))
```

### 4.4 Tính relative rotation (6 dims)

Từ `State_Mimic.cpp` lines 63-76:

```python
target_torso_q = quat_mul(init_quat, ref_torso_q_at_t)
q_rel          = quat_mul(quat_inv(target_torso_q), real_torso_q)
R              = rot_matrix(q_rel).T   # .T vì C++ dùng rot.transpose()

motion_anchor_ori_b = [R[0,0], R[0,1],
                       R[1,0], R[1,1],
                       R[2,0], R[2,1]]  # 6 values
```

Với **perfect tracking** (robot = ref, init_quat = identity): `q_rel = identity`, `R = I`, output = `[1,0, 0,1, 0,0]`.

---

## 5. Thông số motor (từ `deploy.yaml`)

```
DEFAULT_Q (29 joints):
  Left leg  : [-0.1, 0, 0, 0.3, -0.2, 0]
  Right leg : [-0.1, 0, 0, 0.3, -0.2, 0]
  Waist     : [0, 0, 0]
  Left arm  : [0.35, 0.18, 0, 0.87, 0, 0, 0]
  Right arm : [0.35,-0.18, 0, 0.87, 0, 0, 0]

ACTION_SCALE (29 joints):
  Legs  : 0.55, 0.35, 0.55, 0.35, 0.44, 0.44  (per leg)
  Waist : 0.55, 0.44, 0.44
  Arms  : 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07  (per arm)
```

`target_q = DEFAULT_Q + action × ACTION_SCALE`

---

## 6. Vòng lặp điều khiển (50Hz)

```python
while True:
    # 1. Đọc trạng thái robot
    q_real, dq_real = read_motor_state()
    pelvis_q = imu.quaternion   # wxyz
    gyro     = imu.gyroscope

    # 2. Frame motion tham chiếu
    frame_idx = int(t_elapsed / dt) % num_frames
    ref_q  = npz.joint_pos[frame_idx]
    ref_dq = npz.joint_vel[frame_idx]

    # 3. Tính torso quats
    real_torso   = torso_quat(pelvis_q,      q_real[12], q_real[13], q_real[14])
    ref_torso    = torso_quat(npz.root_quat[frame_idx], ref_q[12], ref_q[13], ref_q[14])
    target_torso = quat_mul(init_quat, ref_torso)
    q_rel        = quat_mul(quat_inv(target_torso), real_torso)
    R            = rot_matrix(q_rel).T

    # 4. Xây dựng observation
    obs = concat([
        ref_q, ref_dq,          # 58: motion_command
        R[0,0],R[0,1],R[1,0],R[1,1],R[2,0],R[2,1],  # 6: anchor_ori
        gyro,                   # 3: base_ang_vel
        q_real - DEFAULT_Q,     # 29: joint_pos_rel
        dq_real,                # 29: joint_vel_rel
        last_action,            # 29: last_action
    ])  # total = 154

    # 5. Infer policy
    action = onnx.run(obs)
    last_action = action.copy()

    # 6. Gửi motor
    target_q = DEFAULT_Q + action * ACTION_SCALE
    send_cmd(target_q)
    sleep(0.02)
```

---

## 7. Joint Mapping (29 DOF, identity)

| RL Index | Tên joint | Motor ID |
|----------|-----------|---------|
| 0-5 | left hip/knee/ankle | 0-5 |
| 6-11 | right hip/knee/ankle | 6-11 |
| 12 | waist_yaw | 12 |
| 13 | waist_roll | 13 |
| 14 | waist_pitch | 14 |
| 15-21 | left shoulder/elbow/wrist | 15-21 |
| 22-28 | right shoulder/elbow/wrist | 22-28 |

`JOINT_MAP = list(range(29))` — identity.

---

## 8. Các lỗi đã gặp và cách khắc phục

### 8.1 Action values bão hòa (±10)

**Nguyên nhân:** Policy chưa hội tụ hoặc observation sai thứ tự/chiều.

**Kiểm tra:** Chạy `test_policy.py`. Với perfect tracking, action ≈ `(ref_q - DEFAULT_Q) / ACTION_SCALE`.

### 8.2 Khớp gối không co được

**Nguyên nhân:** `motion_anchor_ori_b` tính sai — dùng pelvis quat thay vì torso quat.

**Fix:** Tính torso quat = `pelvis × AngleAxis(waist_yaw, Z) × AngleAxis(waist_roll, X) × AngleAxis(waist_pitch, Y)`.

### 8.3 Robot bị lệch hướng ngay từ đầu

**Nguyên nhân:** Thiếu `init_quat` bù lệch yaw giữa robot và motion.

**Fix:** Tính `init_quat = yaw(robot_torso_t0) × inv(yaw(ref_torso_t0))`.

### 8.4 CRC lỗi / hành động giật cục

**Nguyên nhân:** Race condition: publisher thread tính CRC khi main thread đang ghi `cmd`.

**Fix:** Dùng `threading.Lock()` cho tất cả truy cập vào `cmd` và `robot_state`.

---

## 9. Files liên quan

| File | Vai trò |
|------|---------|
| `run_mimic.py` | Deploy script standalone |
| `run_g1.py` | Unified controller (locomotion + mimic, gamepad switching) |
| `motions/motion_data.npz` | Motion reference data (50fps, 609 frames) |
| `policy_motion_data.onnx` | ONNX policy (input: 154 dims, output: 29 dims) |
| `test_policy.py` | Offline diagnostic với perfect tracking assumption |
| `check_npz.py` | Kiểm tra NPZ structure và so sánh với CSV |
| `unitree_rl_mjlab/.../params/deploy.yaml` | Config observation chính thức |
| `unitree_rl_mjlab/.../src/State_Mimic.cpp` | C++ reference implementation |

---

## 10. Flow chuẩn để tạo file deploy mimic mới (dành cho AI)

> **Đây là flow đã được kiểm chứng qua thực tế.** Khi người dùng muốn deploy một mimic policy mới, làm đúng theo từng bước.

---

### PHASE A — Chuẩn bị dữ liệu (trên máy train)

#### A1. Kiểm tra CSV gốc

CSV motion phải có đúng **36 cột** (với G1 29-DOF):
```
col 0-2  : base_pos (x, y, z)
col 3-6  : base_quat (x, y, z, w)  ← lưu ý xyzw không phải wxyz
col 7-35 : 29 joint_pos (rad)
```

> **Lỗi thường gặp:** CSV có 29 cột (thiếu base) hoặc 35 cột (thiếu 1 joint) → csv_to_npz sẽ báo lỗi hoặc tạo NPZ sai.

#### A2. Chuyển CSV → NPZ (yêu cầu có `torch`)

```bash
# Cài torch nếu chưa có (chỉ cần trên máy train)
pip install torch --no-cache-dir

# Chuyển 1 file
cd unitree_rl_mjlab
python scripts/csv_to_npz.py \
  --robot g1 \
  --input-file <đường_dẫn_tới_file.csv> \
  --output-name <tên_output>.npz
# → NPZ được lưu tại src/assets/motions/g1/<tên_output>.npz

# Chuyển hàng loạt
for file in <thư_mục_csv>/*.csv; do
  name=$(basename "$file" .csv)
  python scripts/csv_to_npz.py --robot g1 --input-file "$file" --output-name "${name}.npz"
done
```

> **Lưu ý:** `csv_to_npz.py` chạy MuJoCo simulation bên trong để tính `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`. Không thể thay thế bằng đọc CSV thủ công.

#### A3. Train policy

```bash
# GPU 0 — 1 motion
python scripts/train.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/<tên>.npz \
  --env.scene.num-envs=4096 \
  --agent.run-name=<tên>

# 2 GPU song song — 2 motion cùng lúc
# Terminal 1 (GPU 0):
CUDA_VISIBLE_DEVICES=0 python scripts/train.py ... --agent.run-name=motion_A

# Terminal 2 (GPU 1):
CUDA_VISIBLE_DEVICES=1 python scripts/train.py ... --agent.run-name=motion_B
```

> **Lưu ý `--gpu-ids`:** Tyro parser lỗi với `--gpu-ids 0`. Dùng `CUDA_VISIBLE_DEVICES=0` thay thế.

#### A4. Kiểm tra checkpoint bằng play.py

```bash
python scripts/play.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/<tên>.npz \
  --checkpoint_file=logs/rsl_rl/g1_tracking/<run_name>/model_<N>.pt \
  --num-envs 4
```

Robot múa mượt → checkpoint tốt → export ONNX.

#### A5. Export ONNX

```bash
python scripts/export.py Unitree-G1-Tracking-No-State-Estimation \
  --motion_file=src/assets/motions/g1/<tên>.npz \
  --checkpoint_file=logs/rsl_rl/g1_tracking/<run_name>/model_<N>.pt
# → tạo ra policy.onnx trong thư mục log
```

---

### PHASE B — Chuẩn bị deploy (trên máy deploy)

#### B1. Copy files vào thư mục deploy

```
unitree_mujoco/simulate_python/
├── policy_<tên>.onnx       ← copy từ logs/
└── motions/
    └── <tên>.npz           ← copy từ src/assets/motions/g1/
```

> **File NPZ dùng để deploy, không phải CSV.** CSV chỉ cần cho train.

#### B2. Kiểm tra ONNX input shape

```bash
cd unitree_mujoco/simulate_python
python -c "
import onnxruntime as ort
s = ort.InferenceSession('policy_<tên>.onnx', providers=['CPUExecutionProvider'])
print('Input:', s.get_inputs()[0].name, s.get_inputs()[0].shape)
print('Output:', s.get_outputs()[0].name, s.get_outputs()[0].shape)
"
```

| Shape input | Ý nghĩa |
|-------------|---------|
| `[1, 154]` | `has_state_estimation=False` (thông thường) |
| `[1, 160]` | `has_state_estimation=True` (thêm pos+lin_vel) |
| `[1, 29]` | output — target joint positions |

#### B3. Kiểm tra NPZ

```bash
python check_npz.py
```

Phải có đủ:
- `joint_pos (T, 29)` — absolute angles (rad)
- `joint_vel (T, 29)` — angular velocity (rad/s)
- `body_quat_w (T, N_bodies, 4)` — wxyz, body[0] = pelvis
- `fps` = 50 (tương đương `dt=0.02s`)

#### B4. Đọc `deploy.yaml` để xác nhận thông số

File tại `unitree_rl_mjlab/deploy/robots/g1/config/policy/<tên>/params/deploy.yaml`.

Kiểm tra bắt buộc:

```yaml
has_state_estimation: false      # → obs_dim = 154
joint_ids_map: [0,1,2,...,28]    # PHẢI là identity map
stiffness: [...]                  # 29 giá trị → KP
damping:   [...]                  # 29 giá trị → KD
default_joint_pos: [...]          # 29 giá trị → DEFAULT_Q
actions:
  JointPositionAction:
    scale: [...]                  # 29 giá trị → ACTION_SCALE
```

> **Quan trọng:** Nếu `joint_ids_map` KHÔNG phải identity, phải reorder tất cả arrays (KP, KD, DEFAULT_Q, ACTION_SCALE) theo map đó.

---

### PHASE C — Viết/Sửa file deploy

#### C1. Copy template từ `run_mimic.py` hiện có

Chỉ cần thay đổi 4 hằng số ở đầu file:

```python
POLICY_PATH  = "policy_<tên>.onnx"
NPZ_PATH     = "motions/<tên>.npz"

DEFAULT_Q    = np.array([...], dtype=np.float32)   # từ deploy.yaml
KP           = np.array([...], dtype=np.float32)
KD           = np.array([...], dtype=np.float32)
ACTION_SCALE = np.array([...], dtype=np.float32)
```

Nếu `obs_dim = 160`, thêm vào `build_mimic_obs()`:
- `motion_anchor_pos_b` (3 dims) — vị trí tương đối của anchor body
- `base_lin_vel` (3 dims) — từ state estimator

#### C2. Thêm vào `MODES` trong `run_g1.py`

```python
MODES = [
    {"name": "🚶 Locomotion",   "policy": "policy98.onnx",        "npz": None},
    {"name": "💃 Mimic Dance",  "policy": "policy_<tên_1>.onnx", "npz": "motions/<tên_1>.npz"},
    {"name": "🎾 Mimic <Tên>", "policy": "policy_<tên_2>.onnx", "npz": "motions/<tên_2>.npz"},
]
```

---

### PHASE D — Test

#### D1. Offline — test_policy.py

```bash
python test_policy.py
```

**Tiêu chuẩn pass:**
- Action gối (index 3, 9): `mean` gần `(ref_knee - 0.3) / 0.35`
- Không có joint nào bão hòa toàn phần (`min=-10` VÀ `max=+10`)
- `|action_mean|` toàn bộ joints < 5.0

> Nếu fail → policy chưa hội tụ hoặc obs order sai. Đừng deploy.

#### D2. Simulator test

```bash
# Terminal 1
python unitree_mujoco2.py

# Terminal 2
python run_mimic.py --policy policy_<tên>.onnx --npz motions/<tên>.npz
```

**Quan sát:**
- [ ] `[INIT] init_quat` ≈ `[1, 0, 0, 0]` (hoặc chỉ yaw ≠ 0)
- [ ] Robot đứng vững 3-5 giây đầu
- [ ] Khớp gối co/duỗi theo motion
- [ ] Không bị ngã trong 5 giây đầu

#### D3. Tích hợp unified controller

```bash
python run_g1.py
# Nhấn phím 2 hoặc 3... để switch sang mode mới
```

---

### Lỗi thực tế đã gặp và cách xử lý

| Lỗi | Nguyên nhân | Fix |
|-----|-------------|-----|
| `ModuleNotFoundError: torch` | `csv_to_npz.py` cần torch | `pip install torch --no-cache-dir` |
| `No space left on device` | Disk đầy khi cài torch | `pip cache purge` trước |
| `FileNotFoundError: *.csv not found` | Bash glob không expand trong for loop | Kiểm tra thư mục thực tế bằng `ls` |
| Robot đứng im không nhảy | Policy ONNX không khớp với NPZ | Đảm bảo dùng đúng cặp policy+npz |
| Khớp gối không co được | `motion_anchor_ori_b` tính sai | Dùng torso_quat (pelvis × waist joints), không dùng pelvis thẳng |
| Action bão hòa ±10 | Policy chưa hội tụ | Train thêm epochs hoặc dùng checkpoint mới hơn |
| `AttributeError: no attribute log_low_state` | SimStateLogger API khác | Bỏ logger hoặc dùng try/except |
| `UnboundLocalError: last_step_time` | Biến chưa khởi tạo | Thêm `last_step_time = time.perf_counter()` trước vòng lặp |

---

### Checklist nhanh

```
PHASE A (máy train):
  [ ] A1. CSV đúng 36 cột (base_pos 3 + base_quat 4 + joints 29)
  [ ] A2. csv_to_npz.py thành công → NPZ tại src/assets/motions/g1/
  [ ] A3. Train xong → checkpoint tại logs/rsl_rl/g1_tracking/
  [ ] A4. play.py xem được robot múa mượt
  [ ] A5. export.py tạo được policy.onnx

PHASE B (máy deploy):
  [ ] B1. Copy policy.onnx + <tên>.npz vào simulate_python/
  [ ] B2. ONNX input shape = [1, 154] hoặc [1, 160]
  [ ] B3. NPZ có đủ joint_pos, joint_vel, body_quat_w
  [ ] B4. Đọc deploy.yaml → lấy KP, KD, DEFAULT_Q, ACTION_SCALE, joint_ids_map

PHASE C (code):
  [ ] C1. Cập nhật hằng số trong run_mimic.py
  [ ] C2. Thêm mode vào run_g1.py

PHASE D (test):
  [ ] D1. test_policy.py pass (action không bão hòa)
  [ ] D2. Simulator: robot đứng được 5 giây
  [ ] D3. run_g1.py: switch mode thành công
```



