# Thực hành 01 - Bản đồ hệ thống
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-001
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này giúp bạn trả lời được 4 câu hỏi:

1. Robot nhận lệnh từ đâu.
2. ROS 2 nằm ở đâu trong luồng vận hành.
3. Dữ liệu được ghi lại bằng gì.
4. Phần third-party nào là nền tảng, phần nào là lớp tích hợp của dự án.

## 2. Bức tranh tổng thể

```text
Operator
  -> ROS 2 / launch / node
  -> unitree_ros2 wrapper
  -> unitree_sdk2 / unitree_sdk2_python
  -> Robot / Simulator
  -> telemetry / logs / rosbag2
```

## 3. Các lớp cần nhớ

### 3.1. Lớp vận hành

* Người vận hành khởi chạy workflow.
* Kiểm tra an toàn, mạng, và trạng thái robot.
* Ghi log buổi test.

### 3.2. Lớp middleware

* ROS 2 là lớp trung gian để publish/subscribe.
* CycloneDDS chịu trách nhiệm truyền thông thời gian thực qua mạng.
* Đây là nơi các message của hệ thống đi qua.

### 3.3. Lớp third-party

* `third_party/unitree_sdk2`: thư viện lõi C++.
* `third_party/unitree_ros2`: wrapper ROS 2.
* `third_party/unitree_sdk2_python`: binding Python.
* `third_party/unitree_mujoco`: simulator/reference upstream, giữ sạch và không chứa logic vận hành local.

### 3.4. Lớp mô phỏng/policy nội bộ

* `sim/unitree_mujoco_policy`: script chạy policy, logger, replay helper, simulator glue nội bộ.
* `scripts/bridge/run_unitree_mujoco_policy.py`: launcher ghép simulator và policy trên cùng DDS domain/interface.

### 3.5. Lớp dữ liệu

* `data/rosbags`: chứa dữ liệu thu từ các buổi chạy.
* `data/processed`: chứa dữ liệu đã làm sạch hoặc chuẩn hóa.
* `data/models`: chứa model hoặc checkpoint.
* `data/models/unitree_mujoco_policy`: chứa ONNX policy và motion CSV cho MuJoCo.
* `data/sim_state_logs`: chứa CSV trạng thái sinh ra khi chạy policy mô phỏng.

## 4. Bài tập nhanh

1. Mở [README.md](../../../README.md) và tìm các mục mô tả phần cứng, middleware, và workflow.
2. Đối chiếu với cây thư mục ở trên để xác định thư mục nào thuộc runtime, thư mục nào thuộc tài liệu, và thư mục nào thuộc dữ liệu.
3. Tự viết lại bằng một câu ngắn: "Nếu tôi bấm run một node ROS 2, dữ liệu sẽ đi qua những lớp nào?"

## 5. Kết quả mong đợi

Bạn nên có thể mô tả hệ thống bằng một chuỗi ngắn:

* Operator -> ROS 2 -> Wrapper -> SDK -> Robot/Simulator -> Log
* MuJoCo policy runtime -> DDS loopback -> Simulator -> CSV state log

Nếu chưa mô tả được, hãy quay lại đọc phần third-party build và phần network/DDS.
