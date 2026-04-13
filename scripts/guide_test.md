# Development Environment & Workflow Setup Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-003 (Extended Setup Guide)
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này cung cấp hướng dẫn Step-by-Step để khởi tạo môi trường phát triển (Development Environment) từ một máy chủ Ubuntu 22.04 LTS trắng. Mục tiêu là thiết lập một workflow kết hợp giữa **Miniconda** (quản lý Python), **Clang/CMake** (trình biên dịch C++) và **Zsh** (Terminal), đảm bảo sự cô lập an toàn giữa ROS2 system và các thư viện AI/Simulation.

---

## 1. Quản lý môi trường Python với Miniconda

Mặc dù `pyenv` khả thi, **Miniconda** được ưu tiên trong các dự án Robotics kết hợp AI (như Isaac Lab) vì khả năng quản lý các pre-compiled binaries (như CUDA toolkit, cuDNN) xuất sắc hơn, giúp tránh việc phá vỡ môi trường Python mặc định của hệ điều hành mà ROS2 đang phụ thuộc.

### 1.1. Cài đặt Miniconda
Thực thi các lệnh sau để tải và cài đặt Miniconda (phiên bản Linux 64-bit):

```bash
# Tải script cài đặt Miniconda3
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh

# Thực thi script (nhấn Enter và 'yes' khi được hỏi)
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3

# Khởi tạo conda cho shell hiện tại
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

### 1.2. Tạo và cấu hình Virtual Environment
Khởi tạo môi trường có tên `r1_env` với chuẩn Python 3.10.12:

```bash
# Tạo môi trường
conda create -n r1_env python=3.10.12 -y

# Kích hoạt môi trường
conda activate r1_env

# Cài đặt các thư viện cốt lõi cho mô phỏng và SDK
pip install numpy scipy
pip install mujoco==3.1.2
```
*Lưu ý:* Khi làm việc với ROS2, tuyệt đối **không** cài đặt các gói như `rclpy` thông qua `pip` bên trong Conda. Phải để `rclpy` sử dụng gói system-level của ROS2.

---

## 2. Tiêu chuẩn C++ Build System với Clang & CMake

Mặc dù GCC là mặc định trên Ubuntu, dự án quy định sử dụng **Clang** làm trình biên dịch (Compiler) chính. Clang cung cấp thời gian biên dịch nhanh hơn, thông báo lỗi (Error Messages) trực quan, chi tiết hơn, và bộ công cụ phân tích tĩnh (`clang-tidy`, `clang-format`) xuất sắc, rất phù hợp cho việc phát triển Low-level control yêu cầu độ an toàn bộ nhớ cao.

### 2.1. Cài đặt bộ công cụ Clang (LLVM Toolchain)
```bash
sudo apt update
sudo apt install -y clang clang-format clang-tidy lld
```
*(Ghi chú: `lld` là linker của LLVM, hoạt động nhanh hơn nhiều so với linker `ld` mặc định của GNU).*

### 2.2. Cấu hình ROS2 Workspace sử dụng Clang
Để ép hệ thống build `colcon` và `CMake` sử dụng Clang thay vì GCC, bạn cần export các biến môi trường trước khi build.

```bash
# Khai báo biến môi trường cho Compiler
export CC=clang
export CXX=clang++

# Di chuyển vào workspace và build
cd ~/happy-baby-g1/src
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
*Giải thích: Cờ `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` sẽ sinh ra file `compile_commands.json`, file này rất cần thiết cho các công cụ như `clangd` (Language Server) hoạt động trên VSCode, giúp tự động hoàn thiện code (Auto-completion) chính xác.*

---

## 3. Terminal Workflow với Zsh

Chuyển đổi sang Zsh kết hợp với Oh My Zsh và các plugins hỗ trợ gõ lệnh thông minh sẽ tiết kiệm đáng kể thời gian thao tác.

### 3.1. Cài đặt Zsh và Plugins
```bash
# Cài đặt Zsh
sudo apt install -y zsh

# Cài đặt Oh My Zsh
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# Cài đặt các plugins cần thiết (Autosuggestions & Syntax Highlighting)
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```

### 3.2. Cấu hình file `~/.zshrc`
Mở file `~/.zshrc` bằng nano hoặc vi, thêm các plugins và cấu hình custom sau:

```zsh
# 1. Kích hoạt Plugins
plugins=(git zsh-autosuggestions zsh-syntax-highlighting)

# 2. ROS2 & Conda Auto-Setup (Chỉ khởi tạo khi cần để tránh làm chậm terminal mở mới)
alias load_conda="conda activate r1_env"
alias load_ros="source /opt/ros/humble/setup.zsh && source ~/happy-baby-g1/install/local_setup.zsh"
alias setup_dev="load_conda && load_ros"

# 3. Colcon Build Shortcuts (Tích hợp sẵn Clang)
alias cb="CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_SHARED_LINKER_FLAGS='-fuse-ld=lld' -DCMAKE_EXE_LINKER_FLAGS='-fuse-ld=lld'"
alias cb_pkg="CC=clang CXX=clang++ colcon build --symlink-install --packages-select"

# 4. Git Shortcuts cho Workflow
alias gcmsg="git commit -m"
alias gpr="git pull --rebase"

# 5. Network (Unitree R1 Onboard Computer)
alias r1_ssh="ssh unitree@192.168.123.164"
export CYCLONEDDS_URI="file:///home/$USER/happy-baby-g1/config/cyclonedds_config.xml"
```
Sau khi lưu file, chạy `source ~/.zshrc` hoặc khởi động lại terminal.

---

## 4. Initial Test (Kịch bản kiểm tra hệ thống)

Sau khi thiết lập xong, Integration Lead phải chạy kịch bản kiểm tra này để đảm bảo "Golden Machine" đã sẵn sàng.

### Test 1: Kiểm tra Python Environment
```bash
# Bật terminal mới
load_conda
python -c "import mujoco; print('MuJoCo Version:', mujoco.__version__)"
# Kết quả mong đợi: Không có lỗi import, in ra phiên bản 3.1.2.
```

### Test 2: Kiểm tra Compiler
```bash
clang++ --version
# Kết quả mong đợi: Hiển thị thông tin phiên bản LLVM/Clang (Ubuntu clang version 14.x.x).
```

### Test 3: Dummy Build (Biên dịch thử nghiệm)
Tạo một package C++ trống và dùng alias để build:
```bash
cd ~/happy-baby-g1/src
ros2 pkg create --build-type ament_cmake dummy_test_pkg
cb_pkg dummy_test_pkg
# Kết quả mong đợi: Build thành công (Finished), terminal báo thời gian hoàn thành (thường < 1s với lld).
```