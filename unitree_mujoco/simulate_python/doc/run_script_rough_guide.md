# Rough Terrain


## Mục Lục
1. [Tổng quan khác biệt Flat vs Rough](#1-tổng-quan-khác-biệt-flat-vs-rough)
2. [Height Scan — Lý thuyết](#2-height-scan--lý-thuyết)
3. [Ray Casting — Cách hoạt động](#3-ray-casting--cách-hoạt-động)
4. [Xây dựng lưới quét (Grid Pattern)](#4-xây-dựng-lưới-quét-grid-pattern)
5. [Triển khai Ray Casting trong file run](#5-triển-khai-ray-casting-trong-file-run)
6. [Observation Vector 270 dims](#6-observation-vector-270-dims)
7. [SportModeState — Lấy vị trí pelvis](#7-sportmodestate--lấy-vị-trí-pelvis)
8. [Checklist bổ sung cho Rough](#8-checklist-bổ-sung-cho-rough)

---

## 1. Tổng Quan Khác Biệt Flat vs Rough

| Đặc điểm | Flat | Rough |
|---|---|---|
| Obs dimension | **83** | **270** (83 + 187) |
| Height scan | ❌ Không có | ✅ 187 điểm quét |
| Model ONNX | `policy_r1.onnx` | `policy_r1_270.onnx` |
| Cần MuJoCo model | ❌ | ✅ Cần load `scene.xml` cho ray casting |
| Cần `SportModeState` | ❌ | ✅ Cần vị trí pelvis trong world frame |
| Terrain | Mặt phẳng | Địa hình gồ ghề, bậc thang |

---

## 2. Height Scan — Lý Thuyết

### 2.1 Tại sao cần?
Trên mặt phẳng, robot không cần biết hình dạng mặt đất. Nhưng trên địa hình gồ ghề, robot phải "nhìn thấy" mặt đất phía trước để:
- Biết có bậc thang hay không → nhấc chân cao hơn.
- Biết có hố hay không → tránh bước xuống.
- Biết độ dốc → điều chỉnh trọng tâm.

### 2.2 Cách hoạt động trong training
Trong training (`velocity_env_cfg.py` dòng 43-52), mjlab dùng sensor `RayCastSensorCfg`:
```python
terrain_scan = RayCastSensorCfg(
    frame=ObjRef(type="body", name="pelvis", entity="robot"),
    ray_alignment="yaw",          # Lưới quay theo hướng nhìn (yaw) của robot
    pattern=GridPatternCfg(
        size=(1.6, 1.0),          # 1.6m dọc × 1.0m ngang
        resolution=0.1,           # Mỗi 0.1m một điểm
    ),
    max_distance=5.0,             # Tia quét tối đa 5m xuống dưới
    exclude_parent_body=True,     # Bỏ qua va chạm với chính robot
)
```

**Kết quả:** Mỗi step, sensor trả về 187 giá trị = **chiều cao tương đối** của mặt đất so với pelvis tại mỗi điểm lưới.

### 2.3 Scale
Trong `velocity_env_cfg.py` dòng 89:
```python
scale = 1 / terrain_scan.max_distance   # = 1/5.0 = 0.2
```
Nghĩa là: `height_scan_obs[i] = relative_height[i] × 0.2`

---

## 3. Ray Casting — Cách Hoạt Động

### 3.1 Khái niệm
Ray casting = "bắn tia" từ trên xuống dưới tại mỗi điểm lưới. Tia chạm mặt đất → tính khoảng cách → suy ra chiều cao mặt đất.

```
         Robot (pelvis)
            │
    ┌───────┼───────┐
    │  ↓  ↓  ↓  ↓  ↓  │   ← 187 tia bắn xuống
    │  │  │  │  │  │  │
────┘──┘──┘──┘──┘──┘──└──── mặt đất
    h₁ h₂ h₃ h₄ h₅ h₆
```

### 3.2 Công thức
Tại mỗi điểm lưới `(dx, dy)` trong local frame:

```
ray_start = [pelvis_x + dx_world, pelvis_y + dy_world, pelvis_z + 0.5]
ray_dir   = [0, 0, -1]   (bắn thẳng xuống)

dist = mj_ray(model, data, ray_start, ray_dir, ...)
ground_z = ray_start_z - dist
relative_height = ground_z - pelvis_z
```

Trong đó `dx_world, dy_world` là tọa độ đã xoay theo yaw:
```
dx_world = dx × cos(yaw) - dy × sin(yaw)
dy_world = dx × sin(yaw) + dy × cos(yaw)
```

### 3.3 Tại sao cần xoay theo yaw?
Vì config training dùng `ray_alignment="yaw"` — lưới quét luôn hướng theo mũi robot, không theo trục thế giới. Nếu robot quay 90°, lưới cũng quay 90°.

### 3.4 Trích xuất yaw từ quaternion
```python
w, x, y, z = quat   # [w, x, y, z] — quy ước Unitree SDK
yaw = arctan2(2(wz + xy), 1 - 2(y² + z²))
```

---

## 4. Xây Dựng Lưới Quét (Grid Pattern)

### 4.1 Nguồn gốc kích thước
Từ training: `GridPatternCfg(size=(1.6, 1.0), resolution=0.1)`
- Trục X (dọc theo robot): từ -0.8m đến +0.8m → 17 điểm
- Trục Y (ngang robot): từ -0.5m đến +0.5m → 11 điểm
- Tổng: 17 × 11 = **187 điểm**

### 4.2 Code tạo lưới
```python
xs = np.arange(-0.8, 0.8 + 1e-5, 0.1)   # 17 giá trị: -0.8, -0.7, ..., 0.8
ys = np.arange(-0.5, 0.5 + 1e-5, 0.1)   # 11 giá trị: -0.5, -0.4, ..., 0.5
xv, yv = np.meshgrid(xs, ys, indexing='ij')
local_grid = np.stack([xv.flatten(), yv.flatten()], axis=-1)   # shape: (187, 2)
```

> **LƯU Ý:** `indexing='ij'` rất quan trọng! Nó đảm bảo thứ tự flatten khớp với cách mjlab sắp xếp dữ liệu. Nếu dùng `'xy'` sẽ bị lệch hoàn toàn.

---

## 5. Triển Khai Ray Casting Trong File Run

### 5.1 Nạp mô hình MuJoCo ngầm
Cần một bản sao MuJoCo model chứa **cả robot và địa hình** để bắn tia:
```python
mj_model = mujoco.MjModel.from_xml_path("../unitree_robots/r1/scene.xml")
mj_data = mujoco.MjData(mj_model)
```

### 5.2 Đồng bộ trạng thái robot vào model ngầm
Trước khi bắn tia, phải cập nhật vị trí robot trong model ngầm cho khớp với simulator:
```python
mj_data.qpos[0:3] = p_pelvis        # Vị trí pelvis (x, y, z)
mj_data.qpos[3:7] = quat            # Quaternion
for i in range(NUM_JOINTS):
    qpos_adr = mj_model.jnt_qposadr[joint_id]
    mj_data.qpos[qpos_adr] = q_current[i]
mujoco.mj_forward(mj_model, mj_data)  # Cập nhật hình học
```

### 5.3 Loại bỏ va chạm với chính robot
Training dùng `exclude_parent_body=True`. Trong file run, ta phải tự kiểm tra:
```python
def is_robot_body(body_id):
    pelvis_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    curr_id = body_id
    while curr_id > 0:
        if curr_id == pelvis_id:
            return True
        curr_id = mj_model.body_parentid[curr_id]
    return False
```

Khi tia chạm vào bộ phận robot → bỏ qua, tiếp tục bắn xuyên qua:
```python
def cast_ray_to_ground(ray_start, ray_dir):
    for _ in range(5):   # Tối đa 5 lần xuyên qua
        dist = mujoco.mj_ray(mj_model, mj_data, current_start, ray_dir, None, 1, -1, geomid)
        if dist < 0:
            return -1.0
        if not is_robot_body(mj_model.geom_bodyid[geomid[0]]):
            return total_dist + dist
        else:
            current_start += (dist + 1e-3) * ray_dir  # Tiến qua
    return -1.0
```

### 5.4 Tính relative height và scale
```python
for idx in range(187):
    dx, dy = local_grid[idx]
    dx_world = dx * cos(yaw) - dy * sin(yaw)
    dy_world = dx * sin(yaw) + dy * cos(yaw)
    
    ray_start = [pelvis_x + dx_world, pelvis_y + dy_world, pelvis_z + 0.5]
    dist = cast_ray_to_ground(ray_start, [0, 0, -1])
    
    if dist >= 0:
        ground_z = ray_start[2] - dist
        relative_height = ground_z - pelvis_z
    else:
        relative_height = -5.0   # Không chạm gì → coi như hố sâu
    
    height_scan[idx] = np.clip(relative_height, -5.0, 5.0) * 0.2   # scale = 1/max_distance
```

---

## 6. Observation Vector 270 dims

```python
obs = np.concatenate([
    gyro,                 # 3
    projected_gravity,    # 3
    smoothed_commands,    # 3
    gait_phase,           # 2
    q_rel,                # 24
    dq_current,           # 24
    last_action,          # 24
    height_scan           # 187  ← THÊM SO VỚI FLAT
]).astype(np.float32)     # TỔNG: 270
```

> **Thứ tự `height_scan` nằm CUỐI CÙNG** vì trong `actor_terms` dict, key `"height_scan"` là key cuối cùng.

---

## 7. SportModeState — Lấy Vị Trí Pelvis

### 7.1 Tại sao cần?
Ray casting cần biết **vị trí tuyệt đối** của pelvis trong thế giới (x, y, z). `LowState` chỉ cung cấp dữ liệu khớp và IMU, **không có** vị trí thế giới.

### 7.2 Nguồn dữ liệu
Subscribe thêm kênh `rt/sportmodestate`:
```python
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

sub_sport = ChannelSubscriber("rt/sportmodestate", SportModeState_)
sub_sport.Init(sport_state_handler, 10)
```

`sport_state.position` chứa `[x, y, z]` của base robot trong world frame.

### 7.3 Lưu ý khi chạy robot thật
Trên robot thật, `SportModeState` có thể không khả dụng (tùy firmware). Trong trường hợp đó, cần dùng:
- State estimation từ IMU + chân tiếp đất (giống cách Unitree deploy C++ làm).
- Hoặc hệ thống SLAM/VIO ngoài.

---

## 8. Checklist Bổ Sung Cho Rough

Ngoài 10 mục trong [Flat Checklist](run_script_flat_guide.md#11-checklist-chuyển-giao-sang-robot-khác):

| # | Thay đổi | Cách lấy |
|---|----------|----------|
| 11 | Grid size & resolution | Từ `GridPatternCfg` trong `velocity_env_cfg.py` |
| 12 | `max_distance` (scale) | Từ `RayCastSensorCfg.max_distance` |
| 13 | `ray_alignment` | `"yaw"` = xoay theo robot, `"gravity"` = xoay theo trọng lực |
| 14 | `exclude_parent_body` | Luôn True → cần hàm `is_robot_body()` |
| 15 | `indexing` meshgrid | Phải là `'ij'` để khớp với mjlab |
| 16 | Frame gốc bắn tia | Tên body trong `sensor.frame.name` (R1 = `"pelvis"`) |
| 17 | File `scene.xml` | Phải chứa **cùng địa hình** mà robot sẽ chạy |
| 18 | Nguồn vị trí pelvis | `SportModeState` (sim) hoặc state estimation (robot thật) |

---

*Phần tiếp theo (Mimic / Motion Tracking) sẽ được viết khi bạn yêu cầu.*
