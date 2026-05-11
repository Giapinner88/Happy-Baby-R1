# Thực hành 02 - Khởi động và chạy một buổi vận hành
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-PRAC-002
**Author:** Operation & Data Lead
**Status:** Draft / Working

## 1. Mục tiêu

Bài này mô phỏng một buổi làm việc chuẩn để bạn hiểu trình tự vận hành tối thiểu trước khi chạm vào robot thật.

## 2. Kịch bản thực hành

### Pha A - Chuẩn bị môi trường

1. Mở terminal mới.
2. Kích hoạt môi trường phù hợp với nhiệm vụ.
3. Kiểm tra workspace và nhánh git đang dùng.
4. Xác nhận các thư mục build/install/log đang ở trạng thái dự kiến.

### Pha B - Khởi động middleware

1. Nạp môi trường ROS 2 của workspace.
2. Kiểm tra cấu hình mạng theo [../network_setup_checklist.md](../network_setup_checklist.md).
3. Xác nhận node hoặc launch file có thể được resolve đúng.

### Pha C - Chạy một phiên test ngắn

1. Chọn một mục tiêu rõ ràng, ví dụ kiểm tra kết nối hoặc chạy node demo.
2. Ghi lại thời điểm bắt đầu.
3. Ghi lại trạng thái hệ thống trước khi chạy.
4. Chạy test trong thời gian ngắn.
5. Ghi lại kết quả và bất thường nếu có.

### Pha D - Kết thúc buổi chạy

1. Dừng node / launch.
2. Lưu log buổi test.
3. Nếu có dữ liệu rosbag2, lưu đúng thư mục quy định.
4. Trả hệ thống về trạng thái nghỉ.

## 3. Bài tập kiểm tra

Hãy tự trả lời các câu sau:

1. Tôi đang dùng môi trường nào: simulation hay hardware thật?
2. Tôi có thể giải thích vì sao phải kiểm tra mạng trước khi chạy không?
3. Sau khi dừng test, dữ liệu nào cần được lưu lại ngay?
4. Bước nào là điểm an toàn cuối cùng trước khi chạm vào robot?

## 4. Ghi chú thực hành

* Nếu làm trên simulation, ưu tiên kiểm tra logic và dữ liệu.
* Nếu làm trên hardware, ưu tiên an toàn và xác nhận trạng thái trước mỗi hành động.
* Không bỏ qua bước ghi log, vì đây là nguồn truy vết chính khi hệ thống có lỗi.

## 5. Liên kết

* Quy trình vận hành: [../SOP_v0.md](../SOP_v0.md)
* Mẫu log test: [../../templates/test_log_template.md](../../templates/test_log_template.md)
* Quy trình rosbag2: [../rosbag2_operation.md](../rosbag2_operation.md)
