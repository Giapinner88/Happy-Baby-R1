# Hướng dẫn sử dụng rosbag2 chuyên nghiệp (Unitree R1)
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-OPS-001
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Approved / Final

Tài liệu này cung cấp kiến thức từ cơ bản đến nâng cao về **rosbag2** – công cụ "hộp đen" quan trọng để ghi, phát lại và chuyển đổi dữ liệu trong hệ thống Robot R1.

---

## 1. Tổng quan về rosbag2

Trong hệ sinh thái ROS 2, **rosbag2** cho phép ghi lại toàn bộ dữ liệu (messages) từ các topics và lưu thành file để sử dụng lại.

### Mục đích sử dụng

* Debug lỗi mà không cần robot thật
* Phân tích control & sensor
* Tạo dataset cho AI (Imitation Learning, Diffusion Policy)
* Benchmark model (offline)

Trong humanoid lab, **rosbag2 = data backbone của toàn bộ pipeline AI**.

---

## 2. Các lệnh cơ bản (Essentials)

### A. Ghi dữ liệu (Recording)

Không khuyến khích ghi toàn bộ topic vì file rất nặng.

```bash
ros2 bag record -a
```

Khuyến khích ghi có chọn lọc:

```bash
ros2 bag record \
  /rt/lowstate \
  /rt/imu \
  /rt/joint_states \
  -o <ten_file>
```

### Naming convention

`YYYYMMDD_R1_<REAL/SIM>_<TASK>_<ID>_<STATUS>`

Ví dụ:

`20260420_R1_REAL_WalkTest_001_Success`

### B. Kiểm tra thông tin (Inspection)

```bash
ros2 bag info <bag_folder>
```

Hiển thị:

* Duration
* Message count
* Frequency (Hz)
* Topic list

### C. Phát lại dữ liệu (Playback)

```bash
ros2 bag play <bag_folder>
```

---

## 3. Ghi dữ liệu chuyên nghiệp (Best Practice)

### 3.1. Không ghi tất cả topics

Chỉ ghi:

* Control
  * `/rt/lowstate`
  * `/rt/joint_states`
* Sensor
  * `/rt/imu`
* Optional
  * `/camera/*` (ghi riêng)

### 3.2. Sử dụng định dạng MCAP (bắt buộc)

```bash
ros2 bag record /topic --storage mcap
```

Ưu điểm:

* Nhanh hơn SQLite
* Ít lỗi hơn khi replay
* Tương thích Foxglove

### 3.3. Nén dữ liệu

```bash
ros2 bag record \
  /rt/lowstate \
  --compression-mode file \
  --compression-format zstd
```

### 3.4. QoS (lỗi phổ biến nhất)

Nếu replay không ra dữ liệu, thường do QoS mismatch.

Tạo file `qos.yaml`:

```yaml
/rt/imu:
  reliability: best_effort
  history: keep_last
  depth: 5
```

Record:

```bash
ros2 bag record /rt/imu --qos-profile-overrides-path qos.yaml
```

---

## 4. Playback nâng cao (Debug chuyên sâu)

### 4.1. Chạy chậm / nhanh

```bash
ros2 bag play bag --rate 0.5
ros2 bag play bag --rate 2.0
```

### 4.2. Loop

```bash
ros2 bag play bag --loop
```

### 4.3. Chỉ replay một số topic

```bash
ros2 bag play bag --topics /rt/imu
```

### 4.4. Remap topic

```bash
ros2 bag play bag \
  --remap /rt/imu:=/imu_test
```

### 4.5. Đồng bộ thời gian (rất quan trọng)

```bash
ros2 param set /your_node use_sim_time true
```

Nếu không, sẽ xảy ra:

* Lệch timestamp
* Model học sai

---

## 5. Workflow chuẩn trong Lab

**Bước 1:** Kiểm tra hệ thống

```bash
ros2 topic list
```

**Bước 2:** Ghi dữ liệu

```bash
cd data/raw/

ros2 bag record \
  /rt/lowstate \
  /rt/imu \
  -o 20260420_R1_REAL_WalkTest_001
```

**Bước 3:** Kiểm tra

```bash
ros2 bag info <bag>
```

**Bước 4:** Nén & lưu trữ

```bash
tar -czvf bag.tar.gz <bag_folder>
```

**Bước 5:** Phân loại

```text
data/
 ├── raw/
 ├── processed/
 └── failed/
```

---

## 6. Convert rosbag2 sang format khác

### 6.1. Convert storage (SQLite ↔ MCAP)

```bash
ros2 bag convert \
  -i input_bag \
  -o output_bag \
  --output-storage mcap
```

### 6.2. Filter topic khi convert

```bash
ros2 bag convert \
  -i input_bag \
  -o imu_only \
  --topics /rt/imu
```

### 6.3. Convert sang dataset AI (quan trọng)

rosbag2 không export trực tiếp, cần Python script.

Pipeline:

`rosbag2 → read → deserialize → convert → save`

### 6.4. Ví dụ Python extract

```python
from rosbag2_py import SequentialReader
import cv2

# pseudo code
reader = SequentialReader()
reader.open(...)

while reader.has_next():
    topic, data, t = reader.read_next()

    if topic == "/camera/image_raw":
        image = convert_to_cv2(data)
        cv2.imwrite(f"frame_{t}.png", image)
```

### 6.5. Output dataset chuẩn

```text
dataset/
 ├── images/
 ├── labels/
 ├── imu.csv
 └── joints.csv
```

---

## 7. Áp dụng cho AI / Humanoid

Pipeline chuẩn:

```text
rosbag2
   ↓
extract
   ↓
dataset
   ↓
train (diffusion / imitation)
   ↓
replay validate
```

---

## 8. Các lỗi thường gặp

* Drop frame → SSD chậm
* Replay không có data → QoS sai
* Sai chuyển động khi replay → time sync sai
* Dataset unusable → timestamp không đồng bộ

---

## 9. Pro Tips (Kinh nghiệm lab)

* Ghi camera riêng để tránh nghẽn mạng
* Không ghi topic debug
* Trigger recording chỉ ghi khi robot bắt đầu chạy
* Đồng bộ clock, cực quan trọng với humanoid

---

## 10. Topic quan trọng của Unitree R1

| Topic | Ý nghĩa | Tần số |
| :--- | :--- | :--- |
| `/rt/lowstate` | Trạng thái motor, pin | 200–500 Hz |
| `/rt/imu` | Gia tốc + gyro | 100–200 Hz |
| `/rt/joint_states` | Góc khớp | ~100 Hz |

---

## 11. Insight quan trọng (Level Lab)

rosbag2 không chỉ là log

→ nó là data infrastructure

Sai từ bước record

→ toàn bộ AI pipeline sai

---

## 12. Hướng phát triển tiếp theo

* Xây tool extract tự động
* Sync multi-sensor
* Build dataset cho diffusion policy
* Streaming trực tiếp từ bag → model

## 13. Tài liệu liên quan

* Quy ước đặt tên file: [naming_convention.md](naming_convention.md)
* Mẫu log kiểm thử: [../templates/test_log_template.md](../templates/test_log_template.md)
* An toàn phần mềm: [../safety/software_safety_rules.md](../safety/software_safety_rules.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)