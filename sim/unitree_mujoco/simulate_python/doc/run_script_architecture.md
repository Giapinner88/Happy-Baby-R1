# Kiến trúc File Run ONNX cho Robot Unitree R1

Tài liệu này giải thích chi tiết cấu trúc, luồng hoạt động và các thành phần cấu thành nên một file `run_r1_*.py` hoàn chỉnh dùng để chạy model ONNX (được train bằng Reinforcement Learning) trên robot thực hoặc qua Simulator (thông qua Unitree DDS).

---

## 1. Tổng quan Luồng Hoạt Động (The Pipeline)

Một file run hoàn chỉnh hoạt động theo một vòng lặp thời gian thực (real-time control loop), thường chạy ở tần số 50Hz (chu kỳ 0.02s). Luồng dữ liệu như sau:

1. **Đọc trạng thái (Read State):** Nhận dữ liệu `LowState` từ Robot/Simulator qua giao thức DDS (bao gồm góc khớp, vận tốc khớp, IMU).
2. **Xử lý tín hiệu (Pre-processing):** Lọc lệnh điều khiển từ người dùng, tính toán toán học (ví dụ: `projected_gravity`, `gait_phase`).
3. **Đóng gói Observation (Construct Obs):** Ghép các dữ liệu thành một vector duy nhất theo ĐÚNG THỨ TỰ mà môi trường training yêu cầu.
4. **Suy luận Mô hình (Inference):** Đẩy vector Observation vào model ONNX để lấy ra vector Action.
5. **Chuyển đổi Lệnh (Post-processing):** Scale Action thành góc mục tiêu (`q_target`) và gán các hệ số động lực học (`Kp`, `Kd`).
6. **Gửi lệnh (Send Command):** Đóng gói thành `LowCmd` và gửi xuống Robot/Simulator qua DDS.

---

## 2. Các Thành Phần Chi Tiết Trong Code

### Phần 1: Khai báo thư viện và kết nối DDS
```python
import numpy as np
import onnxruntime as ort
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
```
* **`numpy`**: Xử lý mảng và toán học ma trận cho Observation.
* **`onnxruntime`**: Thư viện dùng để nạp và chạy (inference) model `.onnx` bằng CPU/GPU.
* **`unitree_sdk2py`**: Bộ SDK giao tiếp với robot qua DDS. `ChannelPublisher` gửi lệnh (`LowCmd`), `ChannelSubscriber` nhận trạng thái (`LowState`).

### Phần 2: Hằng số Động Lực Học và Mapping (Cực kỳ Quan trọng)
Đây là phần quyết định việc model có hoạt động đúng vật lý như lúc training hay không.

#### A. Tham số Điều Khiển (PD Control)
```python
KP_ARRAY = np.array([100.0, ...], dtype=np.float32)
KD_ARRAY = np.array([2.0, ...], dtype=np.float32)
```
* Robot được điều khiển bằng thuật toán PD (Proportional-Derivative). 
* `Kp` (Độ cứng - Stiffness) và `Kd` (Độ giảm chấn - Damping) phải **khớp 100%** với cấu hình trong file `r1_constants.py` của môi trường RL (`mjlab`). Nếu lệch, robot sẽ tính toán sai momen xoắn (torque) dẫn đến ngã hoặc đi chậm.

#### B. Tư thế Mặc định (Default Posture)
```python
DEFAULT_Q = np.array([-0.1, 0.0, 0.0, 0.3, ...], dtype=np.float32)
```
* Là tư thế "Home" của robot (các góc khớp ở trạng thái đứng nghỉ). 
* RSL-RL sử dụng tọa độ tương đối: `q_rel = q_current - DEFAULT_Q`.

#### C. Hệ số tỷ lệ Hành động (Action Scale)
```python
ACTION_SCALE = np.array([0.150, ...], dtype=np.float32)
```
* Đầu ra của neural network thường trong khoảng `[-1, 1]`. `ACTION_SCALE` là hệ số chuyển đổi giá trị này thành radian.
* Công thức của mjlab thường là: `Action Scale = 0.25 * effort_limit / stiffness`.

#### D. Mapping Khớp (Joint Mapping)
```python
JOINT_IDS_MAP = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, ...]
```
* Với R1, policy/XML và SDK cùng dùng `12=waist_roll`, `13=waist_yaw`.
* Mapping vẫn cần thiết vì các khoảng trống IDL ở tay (`14`, `20`, `21`) và đầu.

### Phần 3: Nhận Dữ Liệu và Toán Học Xử Lý

#### A. Callback nhận `LowState`
```python
def state_handler(msg: LowState_):
    global robot_state
    with state_lock:
        robot_state = msg
```
* Đây là hàm chạy ngầm độc lập (asynchronous), liên tục cập nhật trạng thái mới nhất từ robot vào biến toàn cục `robot_state`. Do chạy đa luồng, phải dùng `state_lock` (Threading Lock) để tránh xung đột dữ liệu lúc đọc/ghi.

#### B. Tính toán Trọng lực Chiếu (Projected Gravity)
```python
def compute_projected_gravity(quat):
    # R^T @ [0, 0, -1]
    ...
```
* Robot cần biết "đâu là dưới đất" để giữ thăng bằng. Model RL không nhận trực tiếp Quaternion mà nhận **Projected Gravity** (vector trọng lực thế giới `[0,0,-1]` được chiếu về hệ tọa độ cục bộ của thân robot).
* **Toán học:** Sử dụng phép xoay ma trận nghich đảo (`quat_apply_inverse`). Nếu sai công thức (ngược dấu), robot sẽ hiểu nhầm là nó đang lộn ngược và tự bẻ gập người lại.

#### C. Tính toán Chu kỳ Bước (Gait Phase)
```python
phase_ratio = (gait_time % 0.6) / 0.6
gait_phase = np.array([np.sin(2 * np.pi * phase_ratio), np.cos(2 * np.pi * phase_ratio)])
```
* Cung cấp "nhịp điệu" cho robot. `0.6s` là một chu kỳ bước hoàn chỉnh. Khi robot đứng yên (Lệnh Command = 0), `gait_phase` sẽ được set về `[0, 0]`.

### Phần 4: Vòng Lặp Điều Khiển Chính (Control Loop)

Là trái tim của chương trình, lặp lại liên tục:
```python
while True:
    step_start = time.perf_counter()
    ...
```

1. **Đọc State mới nhất:** Trích xuất `q` (vị trí khớp), `dq` (vận tốc khớp), `gyro` (vận tốc góc IMU) và `quat` (quaternion).
2. **Lọc Command:** Lấy lệnh từ tay cầm (Gamepad) hoặc bàn phím (`Vx, Vy, Yaw`) và áp dụng bộ lọc EMA (Exponential Moving Average) để tránh lệnh bị giật cục.
3. **Xếp Observation Vector:**
```python
obs = np.concatenate([
    gyro,                 # 3 chiều
    projected_gravity,    # 3 chiều
    smoothed_commands,    # 3 chiều
    gait_phase,           # 2 chiều
    q_rel,                # 24 chiều (vị trí tương đối)
    dq_current,           # 24 chiều
    last_action           # 24 chiều (hành động của bước trước)
]) # Tổng: 83 chiều
```
* **YÊU CẦU TỐI THƯỢNG:** Thứ tự và số lượng chiều của mảng này phải KHỚP 100% với cấu hình `actor_terms` trong file `velocity_env_cfg.py` lúc train. Nếu đảo lộn, model sẽ xuất ra giá trị rác.

4. **Inference ONNX:**
```python
ort_inputs = {input_name: np.array([obs], dtype=np.float32)}
action = session.run(None, ort_inputs)[0][0]
```
5. **Đóng gói Command:**
```python
q_target = DEFAULT_Q + action * ACTION_SCALE
last_action = action
# Lưu vào biến cmd.motor_cmd và gửi đi
```
6. **Sleep bù giờ:** Tính toán xem các bước trên mất bao nhiêu thời gian, và gọi `time.sleep()` phần còn lại để đảm bảo đúng tần số (ví dụ: `0.02s` mỗi vòng lặp).

### Phần 5: Modules Bổ Sung (Add-ons)
* **Fall Detector:** Sử dụng IMU (gia tốc/vận tốc góc) để phát hiện robot bị ngã, lật để tự động ngắt momen (`Kp = 0, Kd = 0`), tránh hỏng hóc phần cứng.
* **Logger:** Ghi lại trạng thái vào file `.csv` để phân tích sau khi chạy xong.

---

## 3. Các Lỗi Phổ Biến Thường Gặp
1. **Robot cúi gập người / không duỗi chân:** Thường do load nhầm version ONNX, hoặc tham số `KP`, `DEFAULT_Q` bị lệch so với lúc train.
2. **Robot giật mạnh rồi lật:** Do `ACTION_SCALE` quá lớn, hoặc `projected_gravity` bị ngược dấu trục Z/Y.
3. **Mismatch Dimension Error:** Báo lỗi lúc chạy `session.run()`, do số lượng Observation lúc code file run không khớp với model ONNX (VD: code 83 chiều nhưng model 98 chiều).
