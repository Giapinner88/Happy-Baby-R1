# Mimic (Motion Tracking) Terrain


## Mục Lục
1. [Khác Biệt Cốt Lõi So Với Velocity Task](#1-khác-biệt-cốt-lõi-so-với-velocity-task)
2. [Hiểu Về Reference Motion File](#2-hiểu-về-reference-motion-file)
3. [Vector Observation Cho Tracking](#3-vector-observation-cho-tracking)
4. [Toán Học Hệ Tọa Độ (Anchor Frame)](#4-toán-học-hệ-tọa-độ-anchor-frame)
5. [Cấu Trúc Vòng Lặp File Run Tracking](#5-cấu-trúc-vòng-lặp-file-run-tracking)
6. [Checklist Chuyển Giao Tracking](#6-checklist-chuyển-giao-tracking)

---

## 1. Khác Biệt Cốt Lõi So Với Velocity Task

Trong bài toán Tracking, **mục tiêu của robot không phải là đi theo lệnh Vận tốc (Vx, Vy, Yaw), mà là khớp tư thế của nó với một file ảnh động (animation) theo từng khung hình (frame).**

| Đặc điểm | Velocity Task (Flat/Rough) | Tracking Task (Mimic) |
|---|---|---|
| **Lệnh (Command)** | Vector 3D `[vx, vy, yaw]` từ Joystick. | Target Joint Pos + Target Joint Vel (VD: 48 dims cho 24 khớp). |
| **Gait Phase** | Có `[sin, cos]` để dạy nhịp bước. | ❌ Không có (nhịp bước lấy từ file gốc). |
| **Projected Gravity** | Cần để định hướng. | ❌ Thường không dùng trực tiếp, thay vào đó là `anchor_ori` (độ lệch hướng). |
| **Observation** | 83 dims hoặc 270 dims. | ~135 dims (tùy thuộc có dùng State Estimation không). |
| **Nguồn dữ liệu mục tiêu**| Joystick. | File `.npz` chứa chuỗi mocap. |

---

## 2. Hiểu Về Reference Motion File

### 2.1 File Mocap (.npz)
Khi train Mimic, mjlab dùng một class `MotionLoader` để nạp dữ liệu từ file `.npz` (chứa các mảng numpy). File này chứa:
- `joint_pos`: mảng `(N, 24)`
- `joint_vel`: mảng `(N, 24)`
- `body_pos_w`: mảng `(N, num_bodies, 3)` (vị trí các bộ phận trong world)
- `body_quat_w`: mảng `(N, num_bodies, 4)` (góc xoay các bộ phận trong world)
Trong đó `N` là tổng số khung hình (time steps).

### 2.2 Tần số phát (Playback)
Nếu file motion được ghi ở 50Hz, mỗi vòng lặp `run` (0.02s) bạn chỉ cần tiến lên 1 frame (tăng index lên 1).

Trong file run python, bạn cần dùng lệnh `np.load()` nạp file này vào RAM trước vòng lặp `while`.

---

## 3. Vector Observation Cho Tracking

Dựa vào `tracking_env_cfg.py` của mjlab, `actor_terms` bao gồm:

1. **`command` (48 dims):** Là mảng nối liền của `target_joint_pos` (24) và `target_joint_vel` (24) ở khung hình hiện tại.
2. **`motion_anchor_pos_b` (3 dims):** Khoảng cách tương đối từ *robot anchor* hiện tại đến *target anchor*.
3. **`motion_anchor_ori_b` (6 dims):** Chênh lệch góc xoay (biểu diễn bằng 2 cột đầu của ma trận xoay 3x3 → 6 số) giữa *robot anchor* và *target anchor*.
4. **`base_lin_vel` (3 dims):** Vận tốc tuyến tính thân robot (Lấy từ State Estimator hoặc Simulator).
5. **`base_ang_vel` (3 dims):** Vận tốc góc từ IMU (Gyro).
6. **`joint_pos` (24 dims):** Vị trí khớp thực tế.
7. **`joint_vel` (24 dims):** Vận tốc khớp thực tế.
8. **`actions` (24 dims):** Lệnh output của step trước.

**Tổng cộng:** 48 + 3 + 6 + 3 + 3 + 24 + 24 + 24 = **135 dimensions** (áp dụng cho robot 24 khớp như R1).

---

## 4. Toán Học Hệ Tọa Độ (Anchor Frame)

### 4.1 Anchor là gì?
"Anchor" thường là gốc tọa độ của robot (pelvis hoặc torso). Tracking tính toán mọi lỗi (vị trí/góc) dựa trên hệ quy chiếu cục bộ của thân robot.

### 4.2 Tính `motion_anchor_pos_b`
Lấy vị trí của mục tiêu trong world (`target_pos_w`), chiếu về hệ tọa độ của robot hiện tại (`robot_pos_w`, `robot_quat_w`):
```python
delta_pos_w = target_pos_w - robot_pos_w
# Biến đổi vector từ hệ world sang hệ local bằng quaternion nghịch đảo
robot_quat_inv = quat_inv(robot_quat_w)
motion_anchor_pos_b = quat_apply(robot_quat_inv, delta_pos_w)
```

### 4.3 Tính `motion_anchor_ori_b`
Độ lệch xoay giữa robot hiện tại và target, đổi thành Rotation Matrix:
```python
# Lệch quaternion
delta_quat = quat_mul(quat_inv(robot_quat_w), target_quat_w)

# Biến quaternion thành ma trận xoay 3x3
R = matrix_from_quat(delta_quat)

# Chỉ lấy 2 cột đầu tiên rồi dải phẳng (6 số)
motion_anchor_ori_b = R[:, :2].flatten()
```
*Lưu ý: Việc lấy 2 cột của ma trận xoay (6D representation) là kỹ thuật phổ biến trong RL để tránh điểm kỳ dị (gimbal lock) của Euler và giải quyết vấn đề quaternion song trùng.*

---

## 5. Cấu Trúc Vòng Lặp File Run Tracking

```python
# Khởi tạo
motion_data = np.load("reference_motion.npz")
total_frames = motion_data["joint_pos"].shape[0]
frame_idx = 0

while True:
    # 1. Đọc frame hiện tại
    target_q = motion_data["joint_pos"][frame_idx]
    target_dq = motion_data["joint_vel"][frame_idx]
    target_pos_w = motion_data["body_pos_w"][frame_idx, anchor_idx]
    target_quat_w = motion_data["body_quat_w"][frame_idx, anchor_idx]

    # 2. Đọc State robot
    robot_q, robot_dq, robot_gyro, robot_quat_w = read_from_dds()
    robot_pos_w, robot_lin_vel_w = read_from_sport_mode_state_or_estimator()

    # 3. Tính toán các term phức tạp
    command = np.concatenate([target_q, target_dq])
    
    delta_pos = target_pos_w - robot_pos_w
    anchor_pos_b = apply_quat_inv(robot_quat_w, delta_pos)
    
    delta_quat = quat_multiply(quat_inverse(robot_quat_w), target_quat_w)
    R = quat2mat(delta_quat)
    anchor_ori_b = np.concatenate([R[:,0], R[:,1]]) # 6D
    
    # 4. Gộp Observation (135 dims)
    obs = np.concatenate([
        command,             # 48
        anchor_pos_b,        # 3
        anchor_ori_b,        # 6
        robot_lin_vel_w,     # 3
        robot_gyro,          # 3
        robot_q - DEFAULT_Q, # 24
        robot_dq,            # 24
        last_action          # 24
    ]).astype(np.float32)

    # 5. Inference
    action = session.run(None, {"obs": [obs]})[0][0]
    last_action = action
    
    # 6. PD Control (Scale và gửi xuống motor)
    q_target = DEFAULT_Q + action * ACTION_SCALE
    send_to_dds(q_target)
    
    # 7. Tăng frame (Loop lại từ đầu nếu hết)
    frame_idx = (frame_idx + 1) % total_frames
    
    time.sleep(0.02)
```

---

## 6. Checklist Chuyển Giao Tracking

Khi viết hoặc sửa file run Mimic, bắt buộc phải kiểm tra:

| # | Khác Biệt | Giải pháp |
|---|---|---|
| 1 | **Nguồn `robot_pos_w`** | Nếu chạy trên Simulator, lấy từ MuJoCo hoặc `SportModeState`. Nếu chạy phần cứng thật, **BẮT BUỘC** phải có State Estimator (Kalman Filter) để tính `base_lin_vel` và `base_pos_w`. |
| 2 | **Chỉ số Anchor Frame** | Xác định xem file train đặt anchor là `pelvis` hay `torso_link` (thường nằm ở `anchor_body_name` trong file cfg). |
| 3 | **Format Motion File** | Phải đảm bảo file `.npz` lúc infer dùng cùng hệ trục, cùng quy ước quaternion (w, x, y, z hay x, y, z, w) như lúc train. |
| 4 | **Không dùng Joystick** | Có thể map Joystick để Play/Pause/Tua ngược cái animation, thay vì gửi lệnh vận tốc. |
| 5 | **State Estimation Overrides** | Đôi khi tác giả dùng `has_state_estimation = False` (ẩn `anchor_pos` và `lin_vel` khỏi obs). Phải check kỹ `env_cfgs.py` (như dòng 75 của g1_23dof) xem Observation có bị lược bớt không. |
