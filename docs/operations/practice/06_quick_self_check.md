# Thực hành 06 - Tự kiểm nhanh sau khi đọc tài liệu
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-006
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Đây là bài tự kiểm cuối cùng để chắc chắn bạn đã nắm được các khối chính của hệ thống.

## 2. Câu hỏi tự kiểm

1. `build/`, `install/`, `log/` dùng để làm gì?
2. `third_party/` chứa loại mã nguồn nào?
3. `sim/unitree_mujoco_policy/` khác gì với `third_party/unitree_mujoco/`?
4. Khi test robot, vì sao phải có log và rosbag?
5. ROS 2 khác gì so với thư viện core SDK?
6. Khi nào nên đọc lại `SOP_v0.md` thay vì chạy tiếp?

## 3. Cách chấm nhanh

* Trả lời được 5/6 câu: đã hiểu luồng cơ bản.
* Trả lời được 6/6 câu: có thể bắt đầu làm việc với buổi test ngắn có giám sát.
* Trả lời dưới 5/6 câu: nên đọc lại [README.md](../../../README.md) và bộ thực hành từ đầu.

## 4. Liên kết

* Lộ trình thực hành: [README.md](README.md)
* Quy trình vận hành: [../SOP_v0.md](../SOP_v0.md)
* Trang chỉ mục an toàn: [../../safety/safety_rules.md](../../safety/safety_rules.md)

## 5. Kết luận

Nếu bạn có thể giải thích hệ thống bằng 6 khối sau, coi như đã nắm được nền tảng:

* Vận hành
* Middleware
* Third-party
* Runtime mô phỏng/policy
* Dữ liệu
* An toàn
