# Unitree R1 - Happy Baby Project (G1 Humanoid Research)

[![Project Status: Active](https://img.shields.io/badge/Project%20Status-Active-brightgreen)](#)
[![Hardware: Unitree R1 EDU U2](https://img.shields.io/badge/Hardware-Unitree%20R1%20EDU%20U2-orange)](#)
[![Middleware: ROS2 Humble](https://img.shields.io/badge/Middleware-ROS2%20Humble-blueviolet)](#)
[![Build: Clang 14](https://img.shields.io/badge/Build-Clang%2014-lightgrey)](#)

> **⚠️ Confidentiality Notice:** This is currently a closed-source enterprise project. Do not distribute without authorization.

-----

## 1\. Project Overview

**Happy Baby Project** là dự án nghiên cứu trọng điểm tại AiRA-Lab, tập trung vào việc làm chủ công nghệ điều khiển và vận hành Robot hình người **Unitree G1**. Dự án được thiết kế để giải quyết bài toán Sim-to-Real, chuyển đổi các chính sách điều khiển (control policies) từ môi trường mô phỏng vật lý độ trung thực cao sang thực thể robot.

[](asset/fig/Unitree_R1_Specs-729x1024.jpg)

### Core Technologies

  * **Hardware:** Unitree G1 (26 DOF), Jetson Orin NX Onboard Computer.
  * **Simulation:** MuJoCo (Physics), Isaac Lab (Reinforcement Learning).
  * **Communication:** ROS2 Humble, CycloneDDS (Optimized for Real-time).
  * **Dev Stack:** Python 3.10 (JAX/PyTorch), C++17 (Low-level Control).

[](asset/fig/draw.png)

-----

## 2\. Technical Objectives

  * **System Hardening:** Thiết lập "Golden Machine" chuẩn hóa trên Ubuntu 22.04, đảm bảo tính đồng nhất 100% về môi trường giữa các máy trạm phát triển.
  * **Real-time Pipeline:** Tối ưu hóa giao tiếp DDS qua mạng Ethernet để đạt tần số điều khiển High-level 500Hz và Low-level 1000Hz.
  * **Unified SDK:** Tích hợp `unitree_sdk2_python` liền mạch giữa phần cứng thật và các simulator.
  * **Deployment Automation:** Tự động hóa quy trình deploy code lên Jetson Orin NX thông qua các script launcher và systemd services.

-----

## 3\. Hardware & Software Specifications

### Onboard Computer (Jetson Orin NX)

| Parameter | Specification |
| :--- | :--- |
| **CPU** | 8-core Arm® Cortex®-A78AE |
| **GPU** | 1024-core NVIDIA Ampere (32 Tensor Cores) |
| **Memory** | 16GB LPDDR5 |
| **Default IP** | `192.168.123.164` (Standard Unitree Config) |
| **OS** | Ubuntu 22.04 LTS |

### Software Prerequisites

  * **Middleware:** ROS2 Humble Hawksbill.
  * **DDS Provider:** CycloneDDS (Required for Unitree SDK2 connectivity).
  * **Python Env:** `pyenv` / `virtualenv` (Python 3.10.12).
  * **Drivers:** CUDA 12.x & cuDNN (cho các tác vụ AI/Isaac Lab).

-----

## 4\. Repository Structure

```text
.
├── .github/                        # (DevOps) Workflows CI/CD, PR Templates
├── docs/                           # (Documentation) Tài liệu dự án
│   ├── hardware/                   # Specs của R1, sơ đồ mạch, pin
│   ├── safety/                     # SOPs An toàn, Emergency Procedures
│   └── architecture/               # Sơ đồ luồng dữ liệu, Network configs
├── scripts/                        # (Tooling) Các script tự động hóa hệ thống
│   ├── env_setup/                  # Script cài Zsh, Conda, Clang
│   └── network/                    # Script cấu hình Static IP, CycloneDDS
├── src/                            # (ROS2 Workspace) Mã nguồn lõi trên robot
│   ├── r1_bringup/                 # Launch files tổng (chạy sim hoặc real)
│   ├── r1_description/             # URDF, Meshes, cấu hình động học
│   ├── r1_controllers/             # (C++) NMPC, LQR, Low-level control (1000Hz)
│   ├── r1_hardware_interface/      # (C++) Bridge kết nối với unitree_sdk2
│   └── r1_messages/                # Định nghĩa Custom ROS2 Interfaces/Messages
├── sim/                            # (Simulation) Môi trường giả lập
│   ├── mujoco_env/                 # (Python/C++) Scripts chạy MuJoCo, test Control
│   └── isaac_lab_env/              # (Python) Môi trường RL, training tasks, configs
├── ai_modules/                     # (AI/ML) Các module trí tuệ nhân tạo độc lập
│   ├── voice_interaction/          # STT, TTS, LLM Prompts (Vietnamese pipeline)
│   └── vision/                     # Xử lý ảnh từ Stereo Camera của R1
├── data/                           # (Data Pipeline) LƯU Ý: Thư mục này bị ignore bởi Git
│   ├── rosbags/                    # Log file ROS2 từ các buổi test
│   ├── models/                     # Checkpoints của RL/LLM
│   └── datasets/                   # Dữ liệu thu thập từ LeRobot (Imitation Learning)
├── third_party/                    # (Dependencies) Thư viện bên ngoài (Submodules)
│   ├── unitree_sdk2/               # C/C++ SDK gốc từ Unitree
│   └── unitree_ros2/               # ROS2 Wrapper do Unitree cung cấp
├── .gitignore                      # Ignore build/, install/, log/, data/
├── requirements.txt                # Khóa phiên bản Python dependencies (Mujoco, numpy...)
└── README.md                       # Trang chủ của Repository
```

-----

## 5\. System Setup & Integration Guide

### 5.1. Network Configuration

Để đảm bảo giao tiếp ổn định với G1, máy tính điều khiển (Host) phải được cấu hình Static IP cùng dải mạng với Robot.

```bash
# Example Setup for Ethernet Interface
sudo nmcli connection modify "Wired connection 1" \
    ipv4.addresses 192.168.123.100/24 \
    ipv4.method manual
sudo nmcli connection up "Wired connection 1"
```

### 5.2. Workspace Initialization

Việc build workspace phải tuân thủ trình tự để đảm bảo các gói `unitree_ros2` được link đúng với middleware:

```bash
# Clone with submodules
git clone --recursive <repo_url>
cd happy-baby-g1/src

# Install dependencies
rosdep update
rosdep install --from-paths . --ignore-src -y

# Build with symlink
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

-----

## 6\. Branching & Development Workflow

Chúng tôi áp dụng mô hình **Git Flow** nghiêm ngặt để bảo vệ tính ổn định của hệ thống:

1.  **`main`**: Chỉ chứa mã nguồn đã pass kiểm tra trên robot thật.
2.  **`develop`**: Nhánh tích hợp chính.
3.  **`feature/*`**: Các nhánh phát triển tính năng mới (e.g., `feature/voice-integration`).
4.  **`sim/*`**: Các nhánh dành riêng cho tuning mô phỏng (e.g., `sim/mujoco-contact-fix`).

> **Rule:** Mọi Pull Request (PR) vào nhánh `develop` bắt buộc phải kèm theo video/log xác nhận đã chạy thành công trong mô phỏng.

### 6.1. Commit & Push Sync

Để đồng bộ làm việc giữa các thành viên, mọi người thống nhất quy trình sau trước khi đẩy code lên GitHub:

1.  Luôn cập nhật nhánh đang làm việc trước khi sửa: `git pull --rebase origin <branch>`.
2.  Chỉ commit khi thay đổi đã được kiểm tra cục bộ và không còn file tạm, log rác, hoặc dữ liệu lớn không cần thiết.
3.  Viết commit message ngắn gọn, theo kiểu hành động rõ ràng, ví dụ: `fix: update DDS reconnect logic`.
4.  Push đúng nhánh đang phụ trách: `git push origin <branch>`.
5.  Nếu có xung đột, ưu tiên báo nhóm và xử lý trên nhánh làm việc, không force push nếu chưa được thống nhất.
6.  Khi hoàn tất một task lớn, tạo Pull Request vào `develop` và ghi rõ nội dung thay đổi, test đã chạy, và người review nếu có.

> **Team rule:** Không commit trực tiếp lên `main`. Mọi thay đổi phải đi qua `feature/*` hoặc `sim/*`, sau đó mới hợp nhất về `develop`.

-----

## 7\. Project Roles & Contact

| Name | Role | Core Responsibility |
| :--- | :--- | :--- |
| **Nguyễn Trọng Giáp** | **Integration Lead** | Workspace, Network, DDS, Onboard Deployment |
| **Phạm Ngọc Khánh** | **Simulation Lead** | Locomotion, MuJoCo/Isaac Lab, Sim-to-Real |
| **Nguyễn Việt Anh** | **Operation Lead** | Data Pipeline, Voice Interaction, Safety SOP |

-----

## 8\. Safety First

Robot G1 có động lực học phức tạp và sức mạnh lớn. Tất cả thành viên phải tuân thủ:

1.  Luôn có người trực nút Emergency Stop khi vận hành.
2.  Tuyệt đối không vận hành robot một mình.
3.  Tuân thủ bảng mã lỗi và quy trình xử lý sự cố trong `docs/safety/Emergency_Procedure.md`.

-----

*© 2026 AiRA-Laboratory. All Rights Reserved.*