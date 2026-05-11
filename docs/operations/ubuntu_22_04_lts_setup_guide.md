# Chuẩn hóa hệ điều hành: Ubuntu 22.04 LTS Setup Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SOP-001
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

## 1. Cấu hình BIOS (Pre-installation)
Trước khi tiến hành cài đặt, việc can thiệp vào BIOS của máy trạm là bắt buộc để đảm bảo các driver độc quyền (Proprietary Drivers) như NVIDIA hoạt động ổn định.
1. Khởi động máy và nhấn `F1` hoặc `F2` (tùy mainboard/workstation) để vào BIOS.
2. Điều hướng đến tab **Security** > **Secure Boot**.
3. **Disable Secure Boot**. *Lưu ý: Nếu không tắt tính năng này, nhân Linux (Kernel) sẽ từ chối tải driver NVIDIA và các module mạng tùy chỉnh cho giao tiếp DDS.*
4. Nhấn `F10` để lưu cấu hình và khởi động lại.

## 2. Thiết lập ngôn ngữ và phân vùng (Locale & Partitioning)
Sử dụng USB Bootable Ubuntu 22.04 LTS (Jammy Jellyfish). Tại màn hình GRUB, chọn **"Try or Install Ubuntu"**.

### 2.1. Thiết lập Locale chuẩn
Sự thiếu đồng nhất về Locale thường xuyên gây ra lỗi crash khi compile các gói ROS2 hoặc khi xử lý các chuỗi ký tự UTF-8 trong log dữ liệu (rosbag).
* **Language:** Chọn **English (US)**. Tuyệt đối không chọn ngôn ngữ cài đặt là Tiếng Việt để tránh tình trạng đường dẫn thư mục bị dịch (ví dụ: `Desktop` thành `Màn hình nền`), gây lỗi đường dẫn trong script.
* **Keyboard Layout:** English (US).
* **Timezone:** Asia / Ho Chi Minh.

### 2.2. Chiến lược phân vùng (Manual Partitioning)
Tuyệt đối không sử dụng tùy chọn *“Erase disk and install Ubuntu”* mặc định. Với ổ cứng tiêu chuẩn 2TB NVMe PCIe Gen4 của dự án, chọn **“Something else”** để tạo bảng phân vùng thủ công (GPT).

Phân bổ không gian đĩa như sau:

| Partition Type | Mount Point | Size (GB) | Format | Chức năng cốt lõi |
| :--- | :--- | :--- | :--- | :--- |
| **EFI** | `/boot/efi` | `1 GB` | FAT32 | Chứa Bootloader (GRUB). |
| **Swap** | `[swap]` | `64 GB` | Swap | Dung lượng bằng RAM thực. Đảm bảo hệ thống không bị tràn bộ nhớ khi build các package C++ lớn bằng `-j16` hoặc khi train mô hình RL. |
| **Root** | `/` | `300 GB` | EXT4 | Không gian riêng cho OS, ROS2 Humble, CUDA Toolkit, và các phần mềm hệ thống. |
| **Home/Data**| `/home` | Phần còn lại | EXT4 | Chứa Repository, Dataset, Rosbag, và các model MuJoCo/Isaac. Tách biệt với Root để bảo toàn dữ liệu nếu OS bị lỗi. |

## 3. Cập nhật hệ thống & Packages cơ bản
Ngay sau khi quá trình cài đặt hoàn tất và khởi động lại vào màn hình Desktop, mở Terminal (`Ctrl + Alt + T`) để tiến hành cập nhật nhân và cài đặt các trình biên dịch thiết yếu.

```bash
# 1. Cập nhật danh sách gói và nâng cấp toàn bộ hệ thống
sudo apt update && sudo apt upgrade -y

# 2. Cài đặt các công cụ xây dựng cốt lõi (Build Essentials)
sudo apt install -y build-essential cmake gcc make linux-headers-$(uname -r)

# 3. Cài đặt các công cụ mạng và quản lý tiến trình cơ bản
sudo apt install -y git curl wget htop net-tools terminator
```

## 4. Tích hợp NVIDIA Graphics Driver
Máy trạm trang bị NVIDIA RTX 4080/4090 yêu cầu driver được cài đặt đúng cách để kích hoạt kiến trúc CUDA cho Isaac Lab. Có hai cách tiếp cận:

**Cách 1: Sử dụng GUI (Khuyến nghị để đảm bảo an toàn)**
1. Mở ứng dụng **Software & Updates**.
2. Chuyển sang tab **Additional Drivers**.
3. Đợi hệ thống quét phần cứng, chọn driver độc quyền (proprietary, tested) mới nhất (Khuyến nghị **nvidia-driver-535** hoặc cao hơn cho dòng RTX 40-series).
4. Nhấn **Apply Changes**, sau khi hoàn tất, nhấn **Restart**.

**Cách 2: Sử dụng Terminal (Dành cho môi trường không có GUI)**
```bash
# Cài đặt tự động driver NVIDIA tương thích và ổn định nhất
sudo ubuntu-drivers autoinstall
sudo reboot
```

Sau khi khởi động lại, kiểm tra trạng thái GPU để xác nhận cài đặt thành công:
```bash
nvidia-smi
```
*(Kết quả mong đợi: Hiển thị bảng giám sát NVIDIA-SMI với thông tin đầy đủ về Driver Version, CUDA Version và tình trạng tiêu thụ VRAM).*

* **Tham khảo:** [Ubuntu Lenovo](../ts_p360_ubuntu_22.04_lts_installation_guide.pdf)

## 5. Tài liệu liên quan

* Hướng dẫn môi trường dev: [development_environment_setup_guide.md](development_environment_setup_guide.md)
* Golden Machine: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)