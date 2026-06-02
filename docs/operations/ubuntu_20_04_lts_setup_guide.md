# Chuẩn hóa hệ điều hành: Ubuntu 20.04 LTS Setup Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SOP-001
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này chuẩn hóa bước cài đặt **Ubuntu 20.04 LTS (Focal Fossa)** cho máy đang dùng trong dự án. Baseline ROS 2 tương ứng là **ROS 2 Foxy**.

> **Lưu ý hỗ trợ:** ROS 2 Foxy đã EOL. Baseline này dùng để đồng bộ với máy Ubuntu 20.04 hiện tại. Nếu cần ROS 2 Humble, hãy nâng OS lên Ubuntu 22.04 hoặc dùng container/VM.

## 1. Cấu hình BIOS trước khi cài

1. Khởi động máy và nhấn `F1` hoặc `F2` tùy mainboard/workstation để vào BIOS.
2. Vào **Security** > **Secure Boot**.
3. Tắt **Secure Boot** để tránh lỗi tải driver NVIDIA hoặc module mạng.
4. Nhấn `F10` để lưu cấu hình và khởi động lại.

## 2. Thiết lập ngôn ngữ và phân vùng

Sử dụng USB bootable Ubuntu 20.04 LTS. Tại màn hình GRUB, chọn **Try or Install Ubuntu**.

### 2.1. Locale chuẩn

- **Language:** English (US). Không chọn Tiếng Việt để tránh đường dẫn thư mục bị dịch.
- **Keyboard Layout:** English (US).
- **Timezone:** Asia / Ho Chi Minh.

Sau khi cài xong, kiểm tra UTF-8:

```bash
locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### 2.2. Chiến lược phân vùng

Không dùng tùy chọn **Erase disk and install Ubuntu** nếu máy có dữ liệu cần giữ. Với ổ 1-2TB NVMe, dùng **Something else** để tạo bảng phân vùng thủ công.

| Partition Type | Mount Point | Size | Format | Chức năng |
| :--- | :--- | :--- | :--- | :--- |
| EFI | `/boot/efi` | `1 GB` | FAT32 | Bootloader GRUB |
| Swap | `[swap]` | `32-64 GB` | Swap | Dự phòng khi build hoặc chạy mô phỏng nặng |
| Root | `/` | `250-300 GB` | EXT4 | OS, ROS 2 Foxy, driver, CUDA, công cụ hệ thống |
| Home/Data | `/home` | Phần còn lại | EXT4 | Repo, dataset, rosbag2, model, log |

## 3. Cập nhật hệ thống và package cơ bản

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y build-essential cmake gcc g++ make linux-headers-$(uname -r)
sudo apt install -y git curl wget htop net-tools terminator software-properties-common gnupg lsb-release
```

## 4. NVIDIA graphics driver

Với máy có GPU NVIDIA, ưu tiên driver từ **Additional Drivers** của Ubuntu 20.04 để giảm rủi ro lệch kernel/module.

### Cách 1: GUI

1. Mở **Software & Updates**.
2. Chọn tab **Additional Drivers**.
3. Chọn driver NVIDIA proprietary tested hoặc bản ổn định mới nhất mà Ubuntu đề xuất.
4. Nhấn **Apply Changes** và reboot.

### Cách 2: Terminal

```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

Kiểm tra:

```bash
nvidia-smi
```

## 5. Cài ROS 2 Foxy

ROS 2 Foxy là bản phù hợp với Ubuntu 20.04 Focal qua deb packages.

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-py
```

Khởi tạo `rosdep`:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Kiểm tra:

```bash
source /opt/ros/foxy/setup.bash
ros2 --help
```

## 6. Tài liệu liên quan

* Hướng dẫn môi trường dev: [development_environment_setup_guide.md](development_environment_setup_guide.md)
* Golden Machine: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
* PDF Lenovo Ubuntu 22.04 cũ chỉ dùng để tham khảo thao tác BIOS/phân vùng nếu cần: [../ts_p360_ubuntu_22.04_lts_installation_guide.pdf](../ts_p360_ubuntu_22.04_lts_installation_guide.pdf)
