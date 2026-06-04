# Mẫu log kiểm thử
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-TPL-001
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Approved / Final

Mẫu này giúp bạn quản lý dữ liệu (Data Organization) một cách chuyên nghiệp.

## 1. Thông tin chung

* ID buổi test: `YYYYMMDD_R1_<SIM/REAL>_<TaskName>_<Attempt>_<Success/Fail/Abort>`
* Ngày thực hiện:
* Nhân sự: Operator: ________ | Safety Lead: ________
* Môi trường: (`SIM` hoặc `REAL`)

## 2. Cấu hình hệ thống

* Phiên bản Code/Branch: (Git hash hoặc tên branch)
* Control Mode: (Position / Velocity / Torque control)
* Control Frequency: (Ví dụ: 500Hz High-level)

## 3. Nội dung thử nghiệm

1. Mục tiêu: Mô tả ngắn gọn task cần thực hiện (ví dụ: test stand_up).
2. Các bước thực hiện:
	* Bước 1...
	* Bước 2...
3. Kết quả (Result): (`Success`, `Fail`, hoặc `Abort`).
4. Vấn đề phát sinh (Issues): Ghi lại các lỗi phần mềm, lỗi kết nối DDS hoặc sai lệch giữa Sim và Real.
5. Next Steps: Hướng xử lý cho lần test tiếp theo.

## 4. Liên kết dữ liệu (Data Links)

* Link Rosbag2: (Đường dẫn trong `data/rosbags/` nếu có)
* Link Video: (Đường dẫn tới clip ghi hình từ Tripod/Camera)

## 5. Tài liệu liên quan

* Quy ước đặt tên file: [../operations/naming_convention.md](../operations/naming_convention.md)
* Quy trình rosbag2: [../operations/rosbag2_operation.md](../operations/rosbag2_operation.md)
* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
