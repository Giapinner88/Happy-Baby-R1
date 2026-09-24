# Flat Terrain

## Mục Lục

1. [Tổng quan Pipeline](#1-tổng-quan-pipeline)
2. [Các file nguồn cần tham chiếu](#2-các-file-nguồn-cần-tham-chiếu)
3. [Thành phần 1: Giao tiếp DDS](#3-thành-phần-1-giao-tiếp-dds)
4. [Thành phần 2: Hằng số động lực học](#4-thành-phần-2-hằng-số-động-lực-học)
5. [Thành phần 3: Projected Gravity](#5-thành-phần-3-projected-gravity)
6. [Thành phần 4: Gait Phase](#6-thành-phần-4-gait-phase)
7. [Thành phần 5: Observation Vector](#7-thành-phần-5-observation-vector)
8. [Thành phần 6: Inference &amp; Action](#8-thành-phần-6-inference--action)
9. [Thành phần 7: Control Loop &amp; Timing](#9-thành-phần-7-control-loop--timing)
10. [Thành phần 8: Fall Detection &amp; Logger](#10-thành-phần-8-fall-detection--logger)
11. [Checklist chuyển giao sang robot khác](#11-checklist-chuyển-giao-sang-robot-khác)

---

## 1. Tổng quan Pipeline

```
┌─────────────┐    ┌──────────────┐    ┌───────────┐    ┌──────────────┐
│ Robot/Sim   │───>│ Đọc State    │───>│ Xây Obs   │───>│ ONNX Model   │
│ (LowState)  │    │ (q, dq, IMU) │    │ (83 dims) │    │ (Inference)  │
└─────────────┘    └──────────────┘    └───────────┘    └──────┬───────┘
                                                              │ action (24 dims)
┌─────────────┐    ┌──────────────┐    ┌───────────┐          │
│ Robot/Sim   │<───│ Gửi Command  │<───│ Scale     │<─────────┘
│ (LowCmd)    │    │ (DDS)        │    │ Action    │
└─────────────┘    └──────────────┘    └───────────┘
```

**Tần số:** 50 Hz (mỗi vòng lặp = 0.02s = `step_dt`).
**Nguồn gốc `step_dt`:** `timestep × decimation = 0.005 × 4 = 0.02s` (file `velocity_env_cfg.py` dòng 424, 429).

---

## 2. Các File Nguồn Cần Tham Chiếu

Khi viết file run cho **bất kỳ robot nào**, bạn cần mở các file sau trong project training (`unitree_rl_mjlab`):

| # | File                                             | Lấy gì                                                             |
| - | ------------------------------------------------ | -------------------------------------------------------------------- |
| 1 | `src/assets/robots/unitree_r1/r1_constants.py` | KP, KD, DEFAULT_Q, ACTION_SCALE, thứ tự khớp XML                  |
| 2 | `src/tasks/velocity/velocity_env_cfg.py`       | Thứ tự Observation (`actor_terms`), `timestep`, `decimation` |
| 3 | `src/tasks/velocity/config/r1/env_cfgs.py`     | Flat xóa`height_scan` → obs nhỏ hơn; Rough giữ nguyên        |
| 4 | `src/tasks/velocity/mdp/observations.py`       | Công thức`phase()` (period, stand_mask)                          |
| 5 | `deploy/robots/r1/.../deploy.yaml`             | `joint_ids_map`, xác nhận lại mọi tham số                     |
| 6 | `src/tasks/velocity/config/r1/rl_cfg.py`       | `obs_normalization` (nếu True → ONNX đã tích hợp sẵn)       |

Trong project simulator (`unitree_mujoco`):

| # | File                                         | Mục đích                                              |
| - | -------------------------------------------- | -------------------------------------------------------- |
| 7 | `unitree_robots/r1/unitree_r1/xmls/r1.xml` | Thứ tự joint trong XML (phải khớp với training XML) |
| 8 | `simulate_python/unitree_sdk2py_bridge.py` | Tham khảo mapping SDK ↔ Sim (29 motor của SDK)        |

---

## 3. Thành Phần 1: Giao Tiếp DDS

### 3.1 Unitree SDK2 DDS là gì?

DDS (Data Distribution Service) là giao thức pub/sub mà Unitree dùng để giao tiếp giữa máy tính điều khiển và robot (hoặc simulator). Có 2 kênh:

- **`rt/lowstate`** (Subscribe): Nhận trạng thái (vị trí/vận tốc khớp, IMU).
- **`rt/lowcmd`** (Publish): Gửi lệnh điều khiển (góc mục tiêu, Kp, Kd).

### 3.2 Khởi tạo

```python
ChannelFactoryInitialize(1, "lo")   # domain_id=1, interface="lo" (loopback cho sim)
pub = ChannelPublisher("rt/lowcmd", LowCmd_)
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(state_handler, 10)         # callback gọi mỗi khi có state mới
```

> **Lưu ý khi chạy robot thật:** Đổi `"lo"` thành tên interface mạng thật (ví dụ `"eth0"`).

### 3.3 Publisher Thread

Publisher chạy ở thread riêng, gửi lệnh liên tục mỗi 2ms (500Hz) để robot không bị mất kết nối:

```python
def dds_publisher_loop(pub):
    crc_calc = CRC()
    while True:
        with cmd_lock:
            cmd.crc = crc_calc.Crc(cmd)  # Checksum bắt buộc
            pub.Write(cmd)
        time.sleep(0.002)
```

### 3.4 State Callback (đa luồng)

```python
def state_handler(msg: LowState_):
    global robot_state
    with state_lock:        # Threading Lock — tránh race condition
        robot_state = msg
```

---

## 4. Thành Phần 2: Hằng Số Động Lực Học

### 4.1 Nguồn gốc — Lấy từ đâu?

Tất cả lấy từ file **`r1_constants.py`** trong project training:

#### A. KP (Stiffness) và KD (Damping)

```python
# Trong r1_constants.py:
R1_ACTUATOR_LEG = BuiltinPositionActuatorCfg(
    stiffness=100.0,   # ← KP cho hip_pitch, hip_roll, hip_yaw, knee
    damping=2.0,        # ← KD
    effort_limit=60.0,
)
R1_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
    stiffness=40.0,    # ← KP cho ankle_pitch, ankle_roll
    damping=2.0,
    effort_limit=50.0,
)
# ... tương tự cho WAIST (100/2), ARM (40/2), WRIST (20/1)
```

**Quy tắc lấy:** `KP[i] = stiffness của actuator chứa khớp i`, `KD[i] = damping`.

#### B. DEFAULT_Q (Tư thế Home)

```python
# Trong r1_constants.py:
HOME_KEYFRAME = EntityCfg.InitialStateCfg(
    joint_pos={
        ".*_hip_pitch_joint": -0.1,
        ".*_knee_joint": 0.3,
        ".*_ankle_pitch_joint": -0.2,
        ".*_shoulder_pitch_joint": 0.35,
        ".*_elbow_joint": 0.87,
        "left_shoulder_roll_joint": 0.18,
        "right_shoulder_roll_joint": -0.18,
    },
)
```

Tất cả khớp không được liệt kê có giá trị mặc định = **0.0**.

**Quy tắc:** Lấy từng giá trị, sắp xếp theo **thứ tự joint trong XML** (không phải thứ tự SDK).

#### C. ACTION_SCALE

```python
# Trong r1_constants.py dòng 176-184:
R1_ACTION_SCALE[n] = 0.25 * effort_limit / stiffness
```

**Công thức:**

```
ACTION_SCALE[i] = 0.25 × effort_limit / stiffness
```

| Nhóm khớp                        | effort | stiffness | Scale            |
| ---------------------------------- | ------ | --------- | ---------------- |
| Leg (hip, knee)                    | 60     | 100       | **0.150**  |
| Ankle                              | 50     | 40        | **0.3125** |
| Waist                              | 60     | 100       | **0.150**  |
| Arm (shoulder pitch/roll)          | 60     | 40        | **0.375**  |
| Wrist (shoulder_yaw, elbow, wrist) | 33     | 20        | **0.4125** |

### 4.2 JOINT_IDS_MAP — Ánh xạ Policy → SDK

Robot thật (Unitree SDK) có **29 motor** nhưng policy chỉ điều khiển **24 khớp**. Ngoài ra, thứ tự joint trong XML (training) khác với thứ tự motor trên phần cứng SDK.

**Lấy từ `deploy.yaml`:**

```yaml
joint_ids_map: [0,1,2,3,4,5, 6,7,8,9,10,11, 12,13, 15,16,17,18,19, 22,23,24,25,26]
```

**Ý nghĩa:** Policy output index `i` → gửi đến motor SDK index `JOINT_IDS_MAP[i]`.

> **R1:** Không hoán đổi eo. Policy/XML và R1 SDK đều dùng `12=waist_roll`,
> `13=waist_yaw`. Quy tắc đảo `13,12` là của G1 và không được áp dụng cho R1.

---

## 5. Thành Phần 3: Projected Gravity

### 5.1 Tại sao cần?

Model RL cần biết "phương thẳng đứng" so với thân robot để giữ thăng bằng. Thay vì truyền quaternion 4D, ta chiếu vector trọng lực thế giới `g_world = [0, 0, -1]` về hệ tọa độ body.

### 5.2 Công thức toán học

Cho quaternion `q = [w, x, y, z]` (quy ước Unitree SDK: w trước).

Ma trận xoay `R` từ quaternion:

```
R = | 1-2(y²+z²)   2(xy-wz)    2(xz+wy)  |
    | 2(xy+wz)     1-2(x²+z²)  2(yz-wx)  |
    | 2(xz-wy)     2(yz+wx)    1-2(x²+y²) |
```

Projected gravity = `R^T @ [0, 0, -1]` = **cột thứ 3 của R, đổi dấu**:

```
gx =  2(wy - xz)        = -2(xz - wy)
gy = -2(yz + wx)         = -2(yz + wx)
gz =  2(x² + y²) - 1    = -(1 - 2(x²+y²))
```

### 5.3 Code

```python
def compute_projected_gravity(quat):
    w, x, y, z = quat
    gx = 2 * (w * y - x * z)
    gy = -2 * (y * z + w * x)
    gz = 2 * (x**2 + y**2) - 1
    return np.array([gx, gy, gz], dtype=np.float32)
```

### 5.4 Kiểm tra nhanh

Khi robot đứng thẳng: `quat ≈ [1, 0, 0, 0]` → `projected_gravity ≈ [0, 0, -1]` ✓

---

## 6. Thành Phần 4: Gait Phase

### 6.1 Tại sao cần?

Cung cấp "nhịp đồng hồ sinh học" cho robot biết khi nào nên nhấc chân trái / chân phải. Không có gait phase → robot không biết xen kẽ bước.

### 6.2 Nguồn gốc

Từ `observations.py` hàm `phase()`:

```python
global_phase = (episode_time % period) / period      # ∈ [0, 1)
phase = [sin(2π × global_phase), cos(2π × global_phase)]
```

Với `period = 0.6s` (từ `velocity_env_cfg.py` dòng 74).

### 6.3 Stand Mask (khi đứng yên)

```python
if norm(command) < 0.1:
    phase = [0.0, 0.0]    # Không cần nhịp bước khi đứng
```

Nguồn: `observations.py` dòng 52-53.

### 6.4 Code trong file run

```python
phase_ratio = (gait_time % 0.6) / 0.6
gait_phase = np.array([
    np.sin(2 * np.pi * phase_ratio),
    np.cos(2 * np.pi * phase_ratio)
], dtype=np.float32)

# Stand mask
if np.linalg.norm(smoothed_commands) < 0.1:
    gait_phase = np.zeros(2, dtype=np.float32)
```

### 6.5 Gait Scale (fade in/out)

Thêm `gait_scale` để chuyển tiếp mượt giữa đứng ↔ đi:

```python
gait_phase *= gait_scale   # gait_scale: 0→1 khi bắt đầu đi, 1→0 khi dừng
```

---

## 7. Thành Phần 5: Observation Vector

### 7.1 Thứ tự — LẤY TỪ ĐÂU?

Mở file `velocity_env_cfg.py`, nhìn vào dict `actor_terms` (dòng 58-91). Thứ tự key trong dict Python 3.7+ được bảo toàn:

```python
actor_terms = {
    "base_ang_vel":       ...,   # gyro — 3 dims
    "projected_gravity":  ...,   # 3 dims
    "command":            ...,   # [vx, vy, yaw] — 3 dims
    "phase":              ...,   # [sin, cos] — 2 dims
    "joint_pos":          ...,   # q_rel — N_JOINTS dims
    "joint_vel":          ...,   # dq — N_JOINTS dims
    "actions":            ...,   # last_action — N_JOINTS dims
    "height_scan":        ...,   # 187 dims (CHỈ CÓ Ở ROUGH!)
}
```

### 7.2 Flat vs Rough

**Flat** (`unitree_r1_flat_env_cfg` dòng 190):

```python
del cfg.observations["actor"].terms["height_scan"]   # Xóa height_scan!
```

→ Obs = 3 + 3 + 3 + 2 + 24 + 24 + 24 = **83 dims**

**Rough:** Giữ `height_scan` (187 dims):
→ Obs = 83 + 187 = **270 dims**

### 7.3 Code xây dựng obs cho Flat

```python
q_rel = q_current - DEFAULT_Q

obs = np.concatenate([
    gyro,                 # 3  — từ imu_state.gyroscope
    projected_gravity,    # 3  — tính từ quaternion
    smoothed_commands,    # 3  — [vx, vy, yaw_rate]
    gait_phase,           # 2  — [sin, cos]
    q_rel,                # 24 — vị trí khớp tương đối
    dq_current,           # 24 — vận tốc khớp
    last_action           # 24 — action của bước trước
]).astype(np.float32)     # TỔNG: 83
```

### 7.4 Observation Normalization

Trong `rl_cfg.py`: `obs_normalization=True`. Điều này có nghĩa running mean/std được **tích hợp sẵn vào model ONNX** khi export. Bạn **KHÔNG CẦN** tự normalize trong file run.

### 7.5 Noise

Training thêm noise vào obs (ví dụ gyro ±0.2). File run **KHÔNG thêm noise** — noise chỉ dùng trong training để tăng robustness.

---

## 8. Thành Phần 6: Inference & Action

### 8.1 Inference ONNX

```python
session = ort.InferenceSession("policy.onnx", providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name   # thường là "obs"

obs_tensor = np.expand_dims(obs, axis=0)    # [83] → [1, 83]
action = session.run(None, {input_name: obs_tensor})[0][0]   # [24]
```

### 8.2 Action → Target Position

**Công thức PD control** trong MuJoCo/phần cứng:

```
τ = Kp × (q_target - q_current) + Kd × (dq_target - dq_current)
```

Trong đó `dq_target = 0` (chỉ điều khiển vị trí).

**Action scaling:**

```python
q_target = DEFAULT_Q + action * ACTION_SCALE
```

Ý nghĩa: Neural network xuất ra action ∈ [-∞, +∞] (thực tế thường [-3, 3]). ACTION_SCALE chuyển thành radian offset so với tư thế home.

### 8.3 Gửi xuống motor

```python
with cmd_lock:
    for i in range(NUM_JOINTS):
        sdk_idx = JOINT_IDS_MAP[i]
        cmd.motor_cmd[sdk_idx].q = float(q_target[i])
        # Kp, Kd đã set lúc init — không cần set lại mỗi vòng
```

### 8.4 Lưu last_action

```python
last_action = action.copy()   # Dùng cho obs của bước tiếp theo
```

---

## 9. Thành Phần 7: Control Loop & Timing

### 9.1 Tần số 50Hz

```python
CTRL_DT = 0.02   # = timestep(0.005) × decimation(4)

while True:
    step_start = time.perf_counter()
    # ... toàn bộ logic ...
    elapsed = time.perf_counter() - step_start
    if CTRL_DT - elapsed > 0:
        time.sleep(CTRL_DT - elapsed)
```

### 9.2 Đọc state từ DDS

```python
with state_lock:
    rs = robot_state
    for i in range(NUM_JOINTS):
        sdk_idx = JOINT_IDS_MAP[i]
        q_current[i] = rs.motor_state[sdk_idx].q
        dq_current[i] = rs.motor_state[sdk_idx].dq
    gyro[:] = np.array(rs.imu_state.gyroscope)
    quat[:] = np.array(rs.imu_state.quaternion)
```

> **Chú ý:** Đọc `q` và `dq` qua `JOINT_IDS_MAP` để chuyển từ SDK index → policy index.

### 9.3 Lọc lệnh EMA (Exponential Moving Average)

```python
alpha = 0.1
smoothed_commands = alpha * target + (1 - alpha) * smoothed_commands
```

Tránh giật cục khi bấm phím. `alpha` nhỏ = mượt hơn nhưng trễ hơn.

---

## 10. Thành Phần 8: Fall Detection & Logger

### Fall Detection (sơ lược)

Dùng IMU để phát hiện ngã:

- `projected_gravity[2]` (gz) gần 0 hoặc dương → robot nằm ngang/lật
- `gyro` lớn đột ngột → robot đang quay nhanh

Khi phát hiện ngã → set `Kp=0, Kd=0, tau=0` cho tất cả khớp (ngắt momen, robot "mềm" để tránh hỏng).

### Logger (sơ lược)

Ghi `q`, `dq`, `action`, `gyro`, `projected_gravity` mỗi step vào CSV để phân tích sau.

---

## 11. Checklist Chuyển Giao Sang Robot Khác

Khi đổi từ R1 sang G1, H1 hay robot khác:

| #  | Thay đổi                 | Cách lấy                                                                |
| -- | -------------------------- | ------------------------------------------------------------------------- |
| 1  | `NUM_JOINTS`             | Đếm số joint trong XML (trừ floating_base)                            |
| 2  | `KP_ARRAY`, `KD_ARRAY` | Từ`*_constants.py` → mỗi actuator group có stiffness/damping riêng |
| 3  | `DEFAULT_Q`              | Từ`HOME_KEYFRAME` trong `*_constants.py`                             |
| 4  | `ACTION_SCALE`           | Công thức`0.25 × effort_limit / stiffness` cho từng khớp           |
| 5  | `JOINT_IDS_MAP`          | Từ`deploy.yaml` — kiểm tra kỹ có swap index nào không            |
| 6  | Obs dimension              | Flat: đếm dims không có height_scan; Rough: thêm height_scan dims    |
| 7  | `period` trong gait      | Từ`velocity_env_cfg.py` → `phase` params                            |
| 8  | `step_dt`                | `timestep × decimation`                                                |
| 9  | File ONNX                  | Export từ checkpoint tương ứng                                        |
| 10 | Quy ước quaternion       | Unitree SDK dùng`[w,x,y,z]`, một số framework dùng `[x,y,z,w]`    |

> **LƯU Ý QUAN TRỌNG:** Nếu bạn thay đổi BẤT KỲ tham số nào trong training (stiffness, default_q, period...) rồi train lại, bạn PHẢI cập nhật lại file run tương ứng. Policy học với bộ tham số nào thì phải chạy với đúng bộ đó.

---

*Tài liệu này bao phủ toàn bộ kiến thức cần thiết cho Flat terrain. Phần Rough terrain (thêm height_scan / ray casting) và Mimic (motion tracking) sẽ được viết trong các file riêng khi bạn yêu cầu.*
