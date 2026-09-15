# Thực hành 03 - Dòng dữ liệu và DDS
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-003
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này giúp bạn hiểu dữ liệu đi từ robot hoặc simulator vào hệ thống ghi log như thế nào.

## 2. Mô hình luồng dữ liệu

```text
Robot / Simulator
  -> DDS / ROS 2 topics
  -> ROS 2 nodes
  -> rosbag2 / log files
  -> analysis / replay / training
```

## 3. Các điểm cần hiểu

### 3.1. DDS làm gì

* DDS là lớp truyền thông nền.
* Nó giúp các node ROS 2 trao đổi dữ liệu theo thời gian thực.
* Nếu DDS lỗi, node có thể vẫn chạy nhưng dữ liệu sẽ không tới đúng nơi.

### 3.2. ROS 2 topics làm gì

* Topic là kênh dữ liệu.
* Node publish dữ liệu ra topic.
* Node khác subscribe topic đó để xử lý tiếp.

### 3.3. rosbag2 làm gì

* rosbag2 ghi lại topic để replay.
* Đây là công cụ quan trọng nhất để debug sau buổi test.
* Nếu không ghi bag, rất khó tái hiện lỗi ngắn hạn.

## 4. Bài tập thực hành

1. Xác định một topic bạn mong đợi trong buổi test.
2. Ghi ra:
   * topic này chứa loại dữ liệu gì
   * ai publish
   * ai subscribe
   * dữ liệu được dùng để làm gì
3. Chọn một buổi chạy mẫu và mô tả dữ liệu sẽ được lưu vào đâu.
4. Viết một câu giải thích vì sao rosbag2 nên được kiểm tra ngay sau test.

## 5. Checklist tự kiểm

* Tôi có phân biệt được node, topic và bag chưa?
* Tôi có biết dữ liệu nào là realtime và dữ liệu nào là hậu kiểm chưa?
* Tôi có biết khi nào cần replay bag thay vì chạy lại robot chưa?

## 6. Liên kết

* Thiết lập mạng/DDS: [../network_setup_checklist.md](../network_setup_checklist.md)
* Quy trình rosbag2: [../rosbag2_operation.md](../rosbag2_operation.md)
* Mẫu log test: [../../templates/test_log_template.md](../../templates/test_log_template.md)
