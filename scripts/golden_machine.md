# Golden Machine Selection & Specification

**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-001
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Proposed

## 1. Mục đích (Purpose)

Tài liệu này xác định cấu hình phần cứng tiêu chuẩn ("Golden Machine") cho dự án nghiên cứu robot hình người Unitree R1. Việc chuẩn hóa hạ tầng phần cứng này nhằm:
* Đảm bảo tính tương thích và hiệu suất tuyệt đối với các engine mô phỏng nặng như **Isaac Lab** và **MuJoCo**.
* Giảm thiểu các biến số và sai số cấu hình trong quá trình chuyển giao thuật toán từ giả lập ra thực tế (**Sim-to-Real**).
* Tối ưu hóa thời gian biên dịch cho toàn bộ workspace ROS2 và các kiến trúc điều khiển phức tạp.

## 2. Thông số kỹ thuật nền tảng (Hardware Context)

[cite_start]Dự án hiện đang vận hành và nhắm mục tiêu phát triển trên phiên bản **Unitree R1 EDU U2**[cite: 267, 268]. [cite_start]Mẫu humanoid này được trang bị 26 bậc tự do (DOF), bao gồm cả hệ thống khớp cổ và đầu linh hoạt[cite: 268]. [cite_start]Nền tảng tính toán tích hợp (onboard computer) cung cấp mức hiệu năng xấp xỉ 100 TOPS [cite: 268][cite_start], cho phép xử lý trực tiếp các dự án AI và thị giác máy tính nâng cao ngay trên phần cứng của robot[cite: 268].

## 3. Thông số kỹ thuật máy trạm đề xuất (Technical Specifications)

Để đồng bộ với hệ điều hành Ubuntu 22.04 LTS và tương thích với yêu cầu của R1, cấu hình Golden Machine được lựa chọn như sau:

| Thành phần | Thông số kỹ thuật chi tiết | Ghi chú |
| :--- | :--- | :--- |
| **CPU** | Intel Core i9-13900K hoặc AMD Ryzen 9 7950X | Tối thiểu 16 nhân/32 luồng hỗ trợ biên dịch và xử lý song song. |
| **GPU** | NVIDIA GeForce RTX 4080 hoặc 4090 (16GB+ VRAM) | Bắt buộc sử dụng kiến trúc Ada Lovelace để tối ưu Isaac Lab. |
| **RAM** | 64GB DDR5 5200MHz (hoặc cao hơn) | Đảm bảo băng thông bộ nhớ không bị nghẽn khi chạy mô phỏng cùng lúc với ROS2. |
| **Storage** | 2TB NVMe PCIe Gen4 SSD | Cung cấp tốc độ đọc/ghi cao phục vụ việc lưu trữ dataset và ghi log (rosbag2). |
| **Network** | Dual Gigabit Ethernet + WiFi 6 | Cổng LAN chuyên dụng để đảm bảo băng thông giao tiếp DDS với R1. |
| **OS** | Ubuntu 22.04.x LTS (Jammy Jellyfish) | Phiên bản hệ điều hành tiêu chuẩn hỗ trợ `unitree_sdk2`. |

## 4. Lý do lựa chọn (Justification & Rationale)

### 4.1. Khả năng đáp ứng Isaac Lab (NVIDIA Omniverse)
Động lực học của nền tảng 26 DOF trên R1 phức tạp hơn rất nhiều so với robot bốn chân truyền thống. **Isaac Lab** đòi hỏi GPU NVIDIA có dung lượng VRAM lớn và sức mạnh từ nhân Tensor để có thể xử lý các phép toán mô phỏng va chạm và render vật lý theo thời gian thực. Kiến trúc RTX 40-series đảm bảo quá trình đào tạo Reinforcement Learning diễn ra ổn định mà không gặp hiện tượng tràn bộ nhớ (Out-Of-Memory).

### 4.2. Hiệu năng xử lý Điều khiển & Mô phỏng
Các tác vụ cốt lõi như biên dịch mã C/C++ cho bộ điều khiển cấp thấp, chạy MuJoCo ở tần số cao để kiểm chứng động học, và xử lý luồng dữ liệu liên tục từ robot yêu cầu tài nguyên tính toán đơn nhân và đa nhân cực kỳ mạnh mẽ. Kiến trúc CPU thế hệ mới giúp giảm thiểu tối đa thời gian chờ đợi giữa các chu kỳ phát triển.

### 4.3. Kiến trúc mạng (Network Architecture)
Bo mạch chủ tích hợp Dual Ethernet là yêu cầu kỹ thuật bắt buộc. Một cổng sẽ được cấu hình Static IP để thiết lập kết nối trực tiếp đến Jetson Orin NX của R1 thông qua giao thức **CycloneDDS**. Thiết lập này giúp loại bỏ hoàn toàn độ trễ và nhiễu từ các thiết bị mạng nội bộ khác. Cổng thứ hai sẽ đảm nhiệm kết nối Internet phục vụ cho việc clone repository và quản lý dependencies.

## 5. Danh mục phần mềm đi kèm (Standard Software Stack)

Sau khi cài đặt hệ điều hành, Golden Machine cần được thiết lập tuần tự theo môi trường chuẩn:
1.  **Môi trường Python:** Thiết lập `pyenv` hoặc `virtualenv` với phiên bản **Python 3.10.12** làm môi trường mặc định (tương thích tối đa với Exudyn và SDK).
2.  **ROS2 Environment:** Cài đặt bản phân phối ROS2 Humble Desktop Full.
3.  **GPU Drivers:** Tích hợp NVIDIA Driver (Stable) và CUDA Toolkit 12.x.
4.  **Middleware:** Cấu hình QoS cho CycloneDDS thông qua file XML được chia sẻ nội bộ.