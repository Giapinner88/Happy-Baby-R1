# Thực hành 05 - Cầu nối third-party
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-005
**Author:** Integration Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này giúp bạn hiểu vì sao hệ thống phải tách rõ giữa third-party code và code của dự án.

## 2. Ba lớp cần phân biệt

### 2.1. Core SDK

* `unitree_sdk2`
* Chịu trách nhiệm giao tiếp mức thấp.
* Thường được build hoặc install riêng.

### 2.2. ROS 2 wrapper

* `unitree_ros2`
* Chuyển dữ liệu của SDK thành message/topic của ROS 2.

### 2.3. Python binding

* `unitree_sdk2_python`
* Phục vụ AI, điều khiển mức cao, và kịch bản nghiên cứu.

## 3. Bài tập quan sát

1. Đọc [../third-party_build.md](../third-party_build.md).
2. Tóm tắt bằng một câu cho mỗi lớp ở trên.
3. Trả lời câu hỏi: nếu thiếu wrapper ROS 2 thì hệ thống sẽ khó làm gì?
4. Trả lời câu hỏi: nếu thiếu Python binding thì nhóm AI sẽ bị hạn chế ở đâu?

## 4. Kết quả mong đợi

Bạn phải thấy được rằng:

* Third-party không phải là nơi bạn sửa logic sản phẩm chính.
* Code của dự án phải gọi đúng ranh giới tích hợp.
* Mỗi lớp có trách nhiệm riêng và không nên trộn lẫn.

## 5. Liên kết

* Quy trình build third-party: [../third-party_build.md](../third-party_build.md)
* Bản đồ hệ thống: [01_system_map.md](01_system_map.md)
