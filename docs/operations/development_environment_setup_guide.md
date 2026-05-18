# Development Environment & Workflow Setup Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-003 (Extended Setup Guide)
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này cung cấp hướng dẫn Step-by-Step để khởi tạo môi trường phát triển (Development Environment) từ một máy chủ Ubuntu 22.04 LTS trắng. Mục tiêu là thiết lập một workflow kết hợp giữa **Miniconda** (quản lý Python), **Clang/CMake** (trình biên dịch C++) và **Zsh** (Terminal), đảm bảo sự cô lập an toàn tuyệt đối giữa ROS 2 system và các thư viện AI/Simulation.

## 1. Khởi tạo Workspace và Quản lý Python với Miniconda

Môi trường phát triển yêu cầu sự phân tách rạch ròi. **Miniconda** được ưu tiên sử dụng để quản lý các pre-compiled binaries (như CUDA, cuDNN) cho thuật toán học máy, nhưng phải được kiểm soát chặt chẽ để không phá vỡ trình thông dịch Python mặc định của hệ điều hành mà ROS 2 phụ thuộc.

### 1.1. Khởi tạo cấu trúc thư mục Workspace
Trước tiên, thiết lập không gian lưu trữ mã nguồn vật lý cho toàn bộ dự án. Mở Terminal và chạy tuần tự các lệnh sau:
```bash
cd ~/Projects
git clone --recursive https://github.com/Giapinner88/Happy-Baby-R1.git
cd Happy-Baby-R1/src

# Cài đặt công cụ quản lý dependencies của ROS 2
sudo apt install python3-rosdep2 -y
rosdep update
rosdep install --from-paths . --ignore-src -y
```

### 1.2. Cài đặt và vô hiệu hóa Auto-Activate của Miniconda
Thực thi các lệnh sau để cài đặt và ngăn chặn Conda tự động can thiệp vào terminal:
```bash
# Tải và thực thi script cài đặt
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3

# Khởi tạo cho shell
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh

# QUAN TRỌNG: Ngăn chặn tự động kích hoạt môi trường base
~/miniconda3/bin/conda config --set auto_activate_base false
```
> **Lưu ý cực kỳ quan trọng:** Sau khi cài đặt, nếu bạn thấy tiền tố `(base)` xuất hiện ở đầu dòng lệnh trong Terminal (nghĩa là môi trường mặc định của Conda đang được kích hoạt), bạn **bắt buộc phải chạy lệnh `conda deactivate`** để thoát ra hoàn toàn trước khi tiến hành bất kỳ thao tác nào với ROS 2.

### 1.3. Khởi tạo Virtual Environment
Tạo không gian cô lập `r1_env` với chuẩn Python 3.10.12 (tương thích nguyên bản với Ubuntu 22.04):
```bash
conda create -n r1_env python=3.10.12 -y
conda activate r1_env
conda install numpy scipy mujoco==3.1.2
conda deactivate
```
*Nguyên tắc tối thượng:* Tuyệt đối không cài đặt các gói ROS (như `rclpy`) thông qua `pip` bên trong Conda. `rclpy` phải sử dụng package system-level.

## 1.4 Cài đặt ROS 2 Humble & Cấu hình Repositories
Để tránh lỗi phân giải tên miền (như sự cố PPA Zotero) và lỗi không tìm thấy gói `colcon`, cần chuẩn hóa danh sách nguồn của Ubuntu:

```bash
# 1. Dọn dẹp các PPA ngoại lai gây nhiễu (nếu có)
sudo rm -f /etc/apt/sources.list.d/zotero*.list

# 2. Thêm GPG Key và Repository chuẩn của ROS 2
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Cài đặt ROS 2 Desktop và công cụ biên dịch Colcon
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
```

## 2. Tiêu chuẩn C++ Build System với Clang & CMake

Dự án quy định sử dụng **Clang** làm Compiler chính. Clang cung cấp thời gian biên dịch nhanh hơn, thông báo lỗi trực quan và bộ phân tích tĩnh (`clang-tidy`) tối ưu cho Low-level control.

### 2.1. Cài đặt LLVM Toolchain
```bash
sudo apt update
sudo apt install -y clang clang-format clang-tidy lld
```
*(Ghi chú: `lld` là linker của LLVM, cung cấp tốc độ liên kết vượt trội so với GNU `ld`).*

### 2.2. Cấu hình ROS 2 Workspace sử dụng Clang
Việc ép `colcon` sử dụng Clang thay vì GCC mặc định được thực hiện thông qua khai báo biến môi trường:
```bash
cd ~/Projects/Happy-Baby-R1
export CC=clang
export CXX=clang++

# Biên dịch workspace lần đầu (kể cả khi chưa có code) để tạo cấu trúc
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

## 3. Terminal Workflow với Zsh

Chuyển đổi sang Zsh kết hợp Oh My Zsh giúp tăng tốc độ thao tác lệnh, nhưng đòi hỏi việc quy hoạch alias phải tuân thủ nghiêm ngặt nguyên tắc cô lập môi trường.

### 3.1. Cài đặt Core và Plugins
```bash
sudo apt install -y zsh
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

# Bổ sung plugins hỗ trợ gõ lệnh
git clone [https://github.com/zsh-users/zsh-autosuggestions](https://github.com/zsh-users/zsh-autosuggestions) ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone [https://github.com/zsh-users/zsh-syntax-highlighting.git](https://github.com/zsh-users/zsh-syntax-highlighting.git) ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```

### 3.2. Thiết lập `~/.zshrc` an toàn
 
**Nguyên tắc: Không bao giờ gộp môi trường AI và ROS 2.**

Mở file `~/.zshrc` bằng lệnh `nano ~/.zshrc`. Tìm dòng `plugins=(git)` và  paste toàn bộ khối cấu hình này ở cuối file:

```zsh
# 1. Kích hoạt Plugins
plugins=(git zsh-autosuggestions zsh-syntax-highlighting) #Thay thế  plugins=(git)

# 2. Phân lập môi trường rõ ràng (Lựa chọn 1 trong 2 khi làm việc)
alias load_ml="conda activate r1_env"
alias load_ros="source /opt/ros/humble/setup.sh && [ -f ~/Projects/Happy-Baby-R1/install/local_setup.zsh ] && source ~/Projects/Happy-Baby-R1/install/local_setup.zsh || true"

# 3. Trình biên dịch C++ (Gắn cứng Clang để tránh nhầm lẫn)
alias cb="CC=clang CXX=clang++ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_SHARED_LINKER_FLAGS='-fuse-ld=lld' -DCMAKE_EXE_LINKER_FLAGS='-fuse-ld=lld' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
alias cb_pkg="cb --packages-select"

# 4. Quản lý mã nguồn
alias gcmsg="git commit -m"
alias gpr="git pull --rebase"

# 5. Mạng giao tiếp phần cứng
alias r1_ssh="ssh unitree@192.168.123.164" #config lại sau khi có hệ thật
export CYCLONEDDS_URI="file:///home/$USER/Projects/Happy-Baby-R1/config/cyclonedds_config.xml"
```

Sau khi lưu file, khởi chạy lại terminal bằng lệnh: exec zsh.

### 3.3. Thiết lập CycloneDDS Middleware
 
Để giải quyết bài toán giao tiếp Real-time giữa Host và Onboard Computer, hệ thống sử dụng CycloneDDS làm Middleware chính. Cấu hình này mở khóa luồng Multicast nội bộ.

Thực thi lệnh sau để ghi tệp cấu hình:
```bash
cat << 'EOF' > ~/Projects/Happy-Baby-R1/config/cyclonedds_config.xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="[https://cdds.io/config](https://cdds.io/config)" xmlns:xsi="[http://www.w3.org/2001/XMLSchema-instance](http://www.w3.org/2001/XMLSchema-instance)" xsi:schemaLocation="[https://cdds.io/config](https://cdds.io/config) [https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd](https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd)">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>
        </General>
        <Internal>
            <WatermarkPings>false</WatermarkPings>
        </Internal>
    </Domain>
</CycloneDDS>
EOF
```

*Luu y:* Neu log CycloneDDS bao `unknown element` cho `WatermarkPings`, hay bo dong nay khoi file XML hoac cap nhat theo schema phu hop version hien tai.

## 4. Kịch bản kiểm thử (Initial Test)

Integration Lead phải thực thi tuần tự 3 kịch bản sau trên hệ thống mới. Nếu bất kỳ test nào thất bại, hệ thống chưa sẵn sàng.

### Test 1: Kiểm định tính độc lập của Python
Bật một terminal hoàn toàn mới.
```bash
# Đảm bảo không có chữ (base) ở đầu. Nếu có, chạy `conda deactivate`.
# Trạng thái chưa nạp môi trường -> Phải báo lỗi (Bảo vệ thành công system Python)
python3 -c "import mujoco" 

# Trạng thái thuật toán -> Phải thành công
exec zsh # Để quay lại bash : exec bash
load_ml
python ~/Projects/Happy-Baby-R1/test/test_ai_env.py
conda deactivate  
```

### Test 2: Xác thực Compiler
```bash
clang++ --version
# Output bắt buộc: Ubuntu clang version 14.x.x
```

### Test 3: Trình liên kết và ROS 2 Build
Mở một terminal mới (hoặc gõ `conda deactivate` nhiều lần cho đến khi biến mất hoàn toàn tên môi trường ở đầu dòng lệnh).
```bash
exec zsh
load_ros
cd ~/Projects/Happy-Baby-R1/src
ros2 pkg create --build-type ament_cmake dummy_test_pkg

# Tạo file mã nguồn giả lập để kích hoạt Clang
mkdir -p dummy_test_pkg/src
cat << 'EOF' > dummy_test_pkg/src/main.cpp
int main() { return 0; }
EOF
echo "add_executable(dummy_node src/main.cpp)" >> dummy_test_pkg/CMakeLists.txt

# Biên dịch kiểm thử
cd ~/Projects/Happy-Baby-R1
cb_pkg dummy_test_pkg

# Output bắt buộc: Finished <<< dummy_test_pkg. Thời gian build phải cực ngắn.
# Kiểm tra định tuyến trình biên dịch:
cat build/dummy_test_pkg/compile_commands.json | grep clang
# Output bắt buộc phải chứa đường dẫn /usr/bin/clang++
```

### Test 4: Giao tiếp Middleware DDS
Kiểm tra khả năng luân chuyển gói tin thời gian thực qua CycloneDDS. 
*Lưu ý: Đảm bảo `cyclonedds_config.xml` đã được khởi tạo theo hướng dẫn của README.md.*

```bash
# Thực thi (Yêu cầu trả về log "DDS Nhận: OK")
load_ros
python3 ~/Projects/Happy-Baby-R1/test/test_dds_node.py
```

## 7. Tài liệu liên quan

* Hướng dẫn cài Ubuntu: [ubuntu_22_04_lts_setup_guide.md](ubuntu_22_04_lts_setup_guide.md)
* Golden Machine: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Quy trình rosbag2: [rosbag2_operation.md](rosbag2_operation.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)