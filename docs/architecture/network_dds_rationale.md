# Chuẩn hóa mạng và CycloneDDS

**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-ARCH-001
**Author:** Operation & Data Lead (Nguyễn Việt Anh)
**Status:** Draft

Tài liệu này giải thích vì sao cấu hình mạng và CycloneDDS là một phần của kiến trúc vận hành robot, không chỉ là bước kết nối Internet thông thường.

## 1. Mục đích

Thiết lập mạng đúng giúp Host, Robot và các công cụ logging/data pipeline trao đổi ổn định trong mọi tình huống thử nghiệm. Với robot humanoid, độ trễ, mất gói hoặc sai dải IP đều có thể làm gián đoạn điều khiển hoặc ghi dữ liệu.

## 2. Các tác dụng cốt lõi

### 2.1. Đảm bảo địa chỉ cố định

Trong môi trường phát triển, IP động (DHCP) rất dễ làm thay đổi địa chỉ của robot sau mỗi lần khởi động. Cấu hình static IP giúp máy tính điều khiển và robot luôn nhận diện nhau ở cùng một dải mạng.

### 2.2. Truyền dữ liệu thời gian thực qua CycloneDDS

Robot R1 sử dụng DDS chạy trên nền Ethernet. CycloneDDS giúp tối ưu độ trễ và độ ổn định của các gói tin control, đặc biệt trong các vòng lặp điều khiển tần số cao.

### 2.3. Phục vụ ghi nhật ký dữ liệu

Mạng ổn định cho phép ghi các topic quan trọng bằng rosbag2 mà không làm nghẽn luồng điều khiển. Đây là nền tảng để tạo dataset cho AI, debug và replay sau này.

### 2.4. Kích hoạt chế độ an toàn

Kết nối mạng chuẩn là đường dẫn để gửi lệnh giảm chấn, zero torque hoặc dừng khẩn cấp từ xa khi robot có dấu hiệu mất kiểm soát.

## 3. Kết luận

Nếu mạng được cấu hình sai, robot sẽ phản ứng chậm hoặc không thể giao tiếp ổn định với phần điều khiển và logging. Vì vậy, checklist thiết lập mạng là một phần bắt buộc của kiến trúc hệ thống.

Tham chiếu thao tác chi tiết: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md).

## 4. Tài liệu liên quan

* Thiết lập mạng/DDS: [../operations/network_setup_checklist.md](../operations/network_setup_checklist.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
* Quy trình vận hành: [../operations/SOP_v0.md](../operations/SOP_v0.md)