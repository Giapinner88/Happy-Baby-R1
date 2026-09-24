# Tài liệu phát triển bộ điều khiển Mimic (Motion Tracking) trong C++

Tài liệu này hướng dẫn chi tiết cách thiết lập, tính toán toán học và triển khai bộ điều khiển theo dõi chuyển động (Mimic/Motion Tracking) từ mô hình ONNX đã huấn luyện bằng IsaacLab/mjlab sang môi trường C++ MuJoCo.

---

## 1. Cấu trúc Vector Quan sát (Observation Vector - 129 Chiều)

Đối với robot Unitree R1 (24 khớp hoạt động), mạng Neural Network (ONNX Policy) yêu cầu vector đầu vào 129 chiều được cấu trúc theo thứ tự sau:

| Index đầu-cuối | Thành phần | Kích thước | Mô tả |
|---|---|---|---|
| `[0..23]` | `target_joint_pos` | 24 | Góc khớp mục tiêu từ frame hiện tại của file mocap. |
| `[24..47]` | `target_joint_vel` | 24 | Vận tốc khớp mục tiêu từ frame hiện tại của file mocap. |
| `[48..53]` | `motion_anchor_ori_b` | 6 | Độ lệch hướng xoay của thân trên (Torso) biểu diễn dạng 6D. |
| `[54..56]` | `gyroscope` | 3 | Tốc độ quay góc đo từ cảm biến IMU (x, y, z). |
| `[57..80]` | `q_rel` | 24 | Vị trí khớp thực tế của robot trừ đi `DEFAULT_JOINT_POS`. |
| `[81..104]` | `dq` | 24 | Vận tốc khớp thực tế của robot. |
| `[105..128]` | `last_action` | 24 | Lệnh điều khiển (action) được xuất ra ở chu kỳ trước đó. |

---

## 2. Các bước xử lý toán học cốt lõi

### 2.1 Căn chỉnh hướng gốc lúc Reset
Để tránh robot bị xoay giật đột ngột khi bắt đầu chạy nếu hướng xuất phát của robot và chuyển động mẫu lệch nhau, ta tính toán ma trận căn chỉnh hướng ban đầu (`init_quat_`):
1. Lấy hướng ban đầu của Torso robot thực tế: `robot_yaw`.
2. Lấy hướng ban đầu của Torso trong file mẫu (Frame 0): `ref_yaw`.
3. Tính sai lệch góc xoay quanh trục thẳng đứng Z (Yaw):
   $$init\_quat = robot\_yaw \cdot ref\_yaw^{T}$$

### 2.2 Động học eo của robot R1 (Waist Kinematics)
Hướng thực tế của Torso robot được tính toán từ hướng của Pelvis (đo bằng IMU) kết hợp với chuyển động xoay của 2 khớp eo:
- Khớp **Waist Roll** (xoay quanh trục X).
- Khớp **Waist Yaw** (xoay quanh trục Z).

Chuỗi động học nối tiếp từ Pelvis lên Torso:
$$R_{torso} = R_{pelvis} \cdot R_{roll}(X) \cdot R_{yaw}(Z)$$

*Lưu ý quan trọng:* Trong cấu hình phần cứng SDK R1, motor 12 là **Waist Yaw** và motor 13 là **Waist Roll**. Cần gán chính xác góc của các động cơ này vào các trục xoay tương ứng.

### 2.3 Tính toán sai lệch hướng Torso tương đối (`motion_anchor_ori_b`)
Tại mỗi chu kỳ điều khiển:
1. Xác định hướng mục tiêu từ file NPZ (`ref_torso`).
2. Căn chỉnh hướng mục tiêu theo yaw gốc:
   $$R_{ref\_aligned} = init\_quat \cdot R_{ref\_torso}$$
3. Tính toán ma trận xoay tương đối từ mục tiêu đến hướng thực tế của robot:
   $$R_{rel} = R_{ref\_aligned}^{T} \cdot R_{real\_torso}$$
4. Trích xuất 6 phần tử đầu tiên từ hai cột đầu của ma trận $R_{rel}$ (Rotation Matrix) để làm đầu vào cho mô hình:
   $$\text{obs}[48..53] = [R_{rel}(0,0), R_{rel}(0,1), R_{rel}(1,0), R_{rel}(1,1), R_{rel}(2,0), R_{rel}(2,1)]$$

---

## 3. Tổng hợp các bài học kinh nghiệm sửa lỗi (Gotchas)

### 3.1 Bộ lọc chuyển tiếp (Ramp-up Filter) tránh giật khớp
* **Vấn đề:** Khi bắt đầu kích hoạt chính sách bắt chước chuyển động (Dance Mode), tư thế hiện tại của robot thường khác xa với Frame 0 của bài múa, dẫn đến robot giật mạnh và ngắt động cơ bảo vệ.
* **Giải pháp:** Áp dụng bộ lọc nội suy tuyến tính trong **1.0 giây đầu tiên** (50 chu kỳ với dt=0.02s) để chuyển tiếp mượt mà từ tư thế ban đầu của robot sang tư thế múa.

### 3.2 Đồng bộ khớp MuJoCo trước khi Raycast (Rough Mode)
* **Vấn đề:** Các tia quét địa hình (`mj_ray`) dùng để đo độ cao mặt đất va chạm với chính chân của robot, khiến robot nhận diện sai độ cao địa hình.
* **Giải pháp:** Trước khi gọi hàm tính toán va chạm và bắn tia quét, cần đồng bộ góc khớp thực tế của robot vào trạng thái MuJoCo:
  ```cpp
  for (int i = 0; i < 24; ++i) {
      mj_data_->qpos[7 + i] = robot_state.motor_state()[joint_idx_in_idl[i]].q();
  }
  mj_forward(mj_model_, mj_data_);
  ```

### 3.3 Sử dụng định dạng file `.npz`
* Không dùng file `.csv` cho các tác vụ Motion Tracking do file CSV thiếu các thông tin hướng xoay quaternion thế giới của các link xương (`body_quat_w`) được xuất ra từ bộ giải động học ngược (IK) trong IsaacLab.
