# SOP_v0 - Quy trình vận hành robot
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SOP-000
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Draft / Working

Tài liệu này chỉ mô tả quy trình vận hành. Các quy tắc an toàn chi tiết nằm trong [../safety/safety_rules.md](../safety/safety_rules.md).

## I. Mục đích

* Thiết lập quy trình chuẩn để vận hành robot Unitree R1.
* Đảm bảo tính nhất quán trong việc thu thập dữ liệu phục vụ Imitation Learning.

## II. Phạm vi áp dụng

* Áp dụng cho mọi buổi test, chạy demo, ghi log và vận hành robot trong lab.
* Các bước chuẩn bị an toàn về người, robot, nguồn và phần mềm phải được xác nhận trước khi vào quy trình dưới đây.

## III. Quy trình vận hành

### 1. Trước khi khởi động (Pre-flight Checklist)

* Kiểm tra ngoại quan: Khớp, pin, dây cáp.
* Kiểm tra kết nối mạng (Ethernet/DDS) theo [network_setup_checklist.md](network_setup_checklist.md).
* Xác nhận tên file log và video theo quy ước trong [naming_convention.md](naming_convention.md).
* Xác nhận kế hoạch test và mẫu log theo [../templates/test_log_template.md](../templates/test_log_template.md).

### 2. Trong khi vận hành

* Tuân thủ đúng kịch bản thử nghiệm đã đề ra.
* Ghi nhận các mốc thay đổi trạng thái chính của robot trong quá trình test.
* Nếu cần dừng khẩn cấp hoặc phát hiện tình huống bất thường, chuyển ngay sang quy trình an toàn trong [../safety/safety_rules.md](../safety/safety_rules.md).

### 3. Ghi hình

* Thực hiện ghi hình theo kịch bản test đã định.
* Đặt video vào đúng cấu trúc thư mục quy định.

### 4. Sau khi vận hành

* Đưa robot về trạng thái nghỉ theo quy trình an toàn trong [../safety/hardware_safety_rules.md](../safety/hardware_safety_rules.md).
* Lưu trữ dữ liệu log và video vào đúng cấu trúc thư mục quy định.
* Ghi chú lại thông tin test trong [test_log_template.md](../templates/test_log_template.md).

## IV. Tài liệu liên quan

* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
* An toàn phần cứng: [../safety/hardware_safety_rules.md](../safety/hardware_safety_rules.md)
* An toàn phần mềm: [../safety/software_safety_rules.md](../safety/software_safety_rules.md)
* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Quy ước đặt tên file: [naming_convention.md](naming_convention.md)
* Mẫu log kiểm thử: [../templates/test_log_template.md](../templates/test_log_template.md)
* Bố cục phòng lab: [../Lab%20setup/laboratory_layout.md](../Lab%20setup/laboratory_layout.md)