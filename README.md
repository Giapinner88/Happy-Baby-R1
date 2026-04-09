# Unitree R1 - ROS2 Integration & Control Workspace

> **Confidentiality Notice:** This is currently a closed-source enterprise project. Do not distribute without authorization.

## 1. Project Overview
Repository này chứa toàn bộ mã nguồn tích hợp, điều khiển và mô phỏng cho nền tảng robot Unitree R1. Hệ thống được xây dựng xoay quanh kiến trúc **ROS2 Humble**, đóng vai trò là middleware kết nối giữa low-level SDK của Unitree và các module high-level (Simulation, Machine Learning, Voice Pipeline).

## 2. Prerequisites (Yêu cầu hệ thống)
Hệ thống yêu cầu tuân thủ nghiêm ngặt các phiên bản phần mềm sau để đảm bảo tính đồng nhất (Golden Machine Standard):
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware:** ROS2 Humble Hawksbill
* **DDS Implementation:** Eclipse CycloneDDS (Recommended cho Unitree SDK)
* **Language:** Python 3.10+ / C++ 17

## 3. Quick Start (Run in 5 Minutes)
Dành cho developers mới setup môi trường lần đầu.

### 3.1. Clone & Install Dependencies
```bash
# 1. Clone repository
git clone [https://github.com/your-org/unitree_r1_workspace.git](https://github.com/your-org/unitree_r1_workspace.git)
cd unitree_r1_workspace

# 2. Setup môi trường cơ bản (Cài ROS2, CycloneDDS và các tool cần thiết)
chmod +x scripts/setup_env.sh
./scripts/setup_env.sh
```

### 3.2. Build Workspace
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Repository Structure
Giải thích ngắn gọn để các Lead khác biết code của họ nên đặt ở đâu:
* `src/unitree_ros2/`: Tích hợp ROS2 packages và DDS configs.
* `src/unitree_controllers/`: Algorithms điều khiển phần cứng.
* `src/unitree_simulation/`: Môi trường giả lập (MuJoCo, Isaac Lab).
* `scripts/`: Scripts hỗ trợ setup và build tự động.
* `docs/`: Chứa SOP, Test logs và kiến trúc giao tiếp (Hardware Comm).

## 5. Contribution Guidelines
* Không push code trực tiếp lên `main` hoặc `develop`.
* Luôn tạo branch mới từ `develop` theo format: `feature/<name>` hoặc `bugfix/<name>`.
* Code mới bắt buộc phải chạy qua môi trường Simulator trước khi được deploy lên hardware thực tế.


### Minh họa bổ sung: Cấu trúc file `setup_env.sh`
Với vai trò Integration Lead, một file script tự động hóa là minh chứng rõ nhất cho việc setup chuẩn mực. Bạn nên tạo một file `scripts/setup_env.sh` chứa các lệnh cài đặt cơ bản nhất cho Ubuntu 22.04:

```bash
#!/bin/bash
# Ví dụ cơ bản cho setup_env.sh

echo "[Integration] Cập nhật hệ thống..."
sudo apt update && sudo apt upgrade -y

echo "[Integration] Cài đặt các công cụ cơ bản..."
sudo apt install -y build-essential git python3-pip curl

# Chỗ này sau sẽ bổ sung thêm các lệnh tự động setup ROS2 Humble keys/repos 
# và cài đặt ros-humble-rmw-cyclonedds-cpp

echo "[Integration] Setup hoàn tất! Hãy tiến hành cài đặt ROS2 Humble theo tài liệu chính thức."
```
