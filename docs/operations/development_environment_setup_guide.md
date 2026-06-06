# Development Environment & Workflow Setup Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-003 (Extended Setup Guide)
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này hướng dẫn khởi tạo môi trường phát triển trên **Ubuntu 20.04 LTS (Focal Fossa)**. Baseline ROS 2 của máy hiện tại là **ROS 2 Foxy Fitzroy**, vì đây là bản ROS 2 có deb packages chính thức cho Ubuntu 20.04.

> **Lưu ý hỗ trợ:** ROS 2 Foxy đã hết vòng đời hỗ trợ chính thức. Dự án vẫn dùng Foxy vì máy đang chạy Ubuntu 20.04. Nếu cần một bản ROS 2 còn được hỗ trợ dài hạn hơn, hướng đi đúng là nâng OS lên Ubuntu 22.04 để dùng Humble, hoặc chạy môi trường Humble trong container/VM riêng.

## 1. Khởi tạo workspace và quản lý Python với Miniconda

Môi trường phát triển cần tách rõ hai lớp:

- **ROS 2 system environment:** dùng Python hệ thống của Ubuntu 20.04, mặc định là Python 3.8.
- **AI/Simulation Conda environment:** dùng cho MuJoCo, SDK Python, training hoặc thử nghiệm thuật toán. Không cài `rclpy` bằng `pip` trong Conda.

### 1.1. Khởi tạo cấu trúc thư mục workspace

```bash
mkdir -p ~/Projects
cd ~/Projects
git clone --recursive https://github.com/Giapinner88/Happy-Baby-R1.git
cd Happy-Baby-R1
```

Cài công cụ quản lý dependency của ROS 2:

```bash
sudo apt update
sudo add-apt-repository universe -y
sudo apt install -y python3-rosdep python3-vcstools python3-pip
python3 -m pip install --user -U colcon-common-extensions
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro foxy
```

Hai package test trong workspace là [dummy_test_pkg](../../src/dummy_test_pkg/package.xml) và [integration_test_pkg](../../src/integration_test_pkg/package.xml). Chúng là package ROS 2 mẫu cho Foxy; lint test được để ở chế độ tùy chọn để `rosdep install` không bị chặn nếu máy chưa có đủ bộ test packages từ ROS build farm.

### 1.2. Cài đặt Miniconda và tắt auto-activate

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3

~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
~/miniconda3/bin/conda config --set auto_activate_base false
```

Sau khi cài đặt, mở terminal mới. Nếu thấy tiền tố `(base)`, chạy:

```bash
conda deactivate
```

### 1.3. Khởi tạo môi trường Conda cho AI/Simulation

Với Ubuntu 20.04 + ROS 2 Foxy, ưu tiên Python 3.8 để giảm sai lệch ABI khi làm việc gần hệ sinh thái ROS/DDS.

```bash
conda create -n r1_env python=3.8.10 -y
conda activate r1_env
conda install -y numpy scipy
pip install mujoco==3.1.2
conda deactivate
```

Nguyên tắc bắt buộc:

- Không cài `rclpy`, `ros2cli` hoặc các package ROS 2 system-level bằng `pip` trong Conda.
- Khi chạy node ROS 2, dùng terminal `load_ros`.
- Khi chạy SDK Python hoặc thuật toán không phụ thuộc `rclpy`, dùng terminal `load_ml`.
- Khi chạy policy MuJoCo nội bộ, dùng `r1_env` và launcher [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md); không đặt script policy hoặc ONNX vào `third_party/unitree_mujoco`.

## 2. Cài ROS 2 Foxy trên Ubuntu 20.04

### 2.1. Thiết lập locale

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### 2.2. Thêm repository ROS 2

Trước khi thêm repo, xác nhận máy đang là Ubuntu 20.04/Focal:

```bash
. /etc/os-release
echo "$UBUNTU_CODENAME"
# Kỳ vọng: focal
```

```bash
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

# Dọn các entry ROS 2 cũ/trùng. Nếu còn entry cũ không có signed-by,
# apt update có thể báo NO_PUBKEY F42ED6FBAB17C654 dù key mới đã được cài.
sudo rm -f /etc/apt/sources.list.d/ros2-latest.list
sudo rm -f /etc/apt/sources.list.d/ros2.list
sudo sed -i.bak '/packages\.ros\.org\/ros2\/ubuntu/d' /etc/apt/sources.list

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor --yes -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.3. Cài ROS 2 Foxy và công cụ build

Sau khi ghi lại key hoặc source list, luôn chạy `sudo apt update` lại trước khi cài package ROS. Nếu bỏ qua bước này, apt vẫn dùng cache cũ và sẽ báo `Unable to locate package`.

```bash
sudo apt update
sudo apt install -y ros-foxy-desktop
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-py
```

Nếu `sudo apt update` vẫn báo lỗi public key:

```bash
grep -R "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/*.list
ls -l /usr/share/keyrings/ros-archive-keyring.gpg
```

Kết quả đúng chỉ nên còn một dòng repo ROS 2, có `signed-by=/usr/share/keyrings/ros-archive-keyring.gpg` và codename là `focal`.

Nếu cài ROS bị dừng ở các package như `python3-catkin-pkg-modules`, `python3-rospkg-modules` hoặc `python3-rosdistro-modules`, apt đang bị kẹt ở trạng thái broken dependency. Thường nguyên nhân là package Python ROS từ Ubuntu `universe` và package mới hơn từ repo ROS đang bị cài xen kẽ. Sửa theo thứ tự sau:

```bash
sudo apt --fix-broken install
sudo dpkg --configure -a
sudo apt update
sudo apt install -y ros-foxy-desktop ros-foxy-rmw-cyclonedds-cpp ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-py
```

Nếu `apt --fix-broken install` vẫn fail, kiểm tra package đang xung đột:

```bash
dpkg -l | grep -E 'python3-(catkin-pkg|rospkg|rosdistro)'
sudo apt remove -y python3-catkin-pkg python3-rospkg python3-rosdistro
sudo apt --fix-broken install
sudo apt install -y ros-foxy-desktop ros-foxy-rmw-cyclonedds-cpp ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-py
```

Nếu vẫn còn lỗi `trying to overwrite`, lưu full log trước khi dùng biện pháp mạnh:

```bash
sudo apt --fix-broken install 2>&1 | tee /tmp/apt-fix-broken.log
```

Nếu log đúng là 3 dòng overwrite sau:

- `catkin_pkg/__init__.py` thuộc `python3-catkin-pkg`
- `rospkg/__init__.py` thuộc `python3-rospkg`
- `rosdistro/__init__.py` thuộc `python3-rosdistro`

thì cho phép dpkg thay thế 3 package cũ bằng 3 package `*-modules` từ repo ROS, rồi cấu hình lại các package còn dang dở:

```bash
sudo dpkg -i --force-overwrite \
  /var/cache/apt/archives/python3-catkin-pkg-modules_*.deb \
  /var/cache/apt/archives/python3-rospkg-modules_*.deb \
  /var/cache/apt/archives/python3-rosdistro-modules_*.deb

sudo apt --fix-broken install
sudo dpkg --configure -a
sudo apt install -y ros-foxy-desktop ros-foxy-rmw-cyclonedds-cpp ros-foxy-demo-nodes-cpp ros-foxy-demo-nodes-py
```

Sau khi apt hết broken, kiểm tra:

```bash
sudo apt-get check
```

Kiểm tra:

```bash
load_ros
ros2 --help
ros2 run demo_nodes_cpp talker
```

## 3. Tiêu chuẩn C++ build system

Ubuntu 20.04 dùng GCC 9.x làm compiler mặc định. Có thể dùng GCC mặc định để giảm rủi ro tương thích với Foxy. Nếu cần Clang cho phân tích tĩnh hoặc build nhanh hơn, cài thêm:

```bash
sudo apt update
sudo apt install -y clang clang-format clang-tidy lld
```

Build workspace:

```bash
cd ~/Projects/Happy-Baby-R1
load_ros
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.zsh
```

Build với Clang khi cần:

```bash
cd ~/Projects/Happy-Baby-R1
load_ros
CC=clang CXX=clang++ colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Nếu trước đó đã build bằng compiler khác hoặc CMake cache đang trỏ nhầm install prefix về `/usr/local`, build có thể fail với lỗi như:

```text
file cannot create directory: /usr/local/lib/<package_name>. Maybe need administrative privileges.
```

Không chạy `sudo colcon build`. Xóa cache rồi build lại:

```bash
cd ~/Projects/Happy-Baby-R1
rm -rf build install log
load_ros

CC=clang CXX=clang++ colcon build \
  --base-paths src \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_COMPILER=clang++ \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=lld" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 4. Terminal workflow với Zsh

### 4.1. Cài Zsh và plugin

```bash
sudo apt install -y zsh git curl
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
```

### 4.2. Thiết lập `~/.zshrc`

Mở `~/.zshrc`:

```bash
nano ~/.zshrc
```

Thêm hoặc cập nhật các dòng sau:

```zsh
plugins=(git zsh-autosuggestions zsh-syntax-highlighting)

if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
fi

_remove_ros_paths() {
  local var_name="$1"
  local value="${(P)var_name}"
  local filtered=""
  local entry

  for entry in ${(s.:.)value}; do
    [[ -z "$entry" ]] && continue
    [[ "$entry" == /opt/ros/* ]] && continue
    [[ "$entry" == "$HOME/Projects/Happy-Baby-R1/install"* ]] && continue
    filtered="${filtered:+$filtered:}$entry"
  done

  export "$var_name=$filtered"
}

load_ml() {
  _remove_ros_paths PATH
  _remove_ros_paths PYTHONPATH
  _remove_ros_paths LD_LIBRARY_PATH
  _remove_ros_paths AMENT_PREFIX_PATH
  _remove_ros_paths COLCON_PREFIX_PATH
  _remove_ros_paths CMAKE_PREFIX_PATH
  unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI
  conda activate r1_env
}

alias load_ros="source /opt/ros/foxy/setup.zsh && [ -f ~/Projects/Happy-Baby-R1/install/local_setup.zsh ] && source ~/Projects/Happy-Baby-R1/install/local_setup.zsh || true"

alias cb="colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
alias cb_clang="CC=clang CXX=clang++ colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_SHARED_LINKER_FLAGS='-fuse-ld=lld' -DCMAKE_EXE_LINKER_FLAGS='-fuse-ld=lld' -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
alias cb_pkg="cb --packages-select"

alias gcmsg="git commit -m"
alias gpr="git pull --rebase"
alias r1_ssh="ssh unitree@192.168.123.164"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/$USER/Projects/Happy-Baby-R1/config/cyclonedds_config.xml"
```

Nạp lại shell:

```bash
exec zsh
```

## 5. Thiết lập CycloneDDS

File cấu hình chuẩn nằm tại:

```text
config/cyclonedds_config.xml
```

Nội dung tối thiểu đang dùng trong repo:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
            <AllowMulticast>true</AllowMulticast>p nhậ
        </General>
        <Internal>
            <WatermarkPings>false</WatermarkPings>
        </Internal>
    </Domain>
</CycloneDDS>
```

Nếu CycloneDDS trên Foxy báo `unknown element` hoặc `deprecated element` với `WatermarkPings`, bỏ khối `<Internal>` để test local trước, sau đó cập nhật XML theo version CycloneDDS đang cài.

## 6. Kịch bản kiểm thử ban đầu

### Test 1: Kiểm tra tách biệt Python

Terminal mới, chưa load môi trường:

```bash
python3 -c "import mujoco"
# Kỳ vọng: lỗi ImportError nếu chưa load Conda.
```

Terminal AI:

```bash
exec zsh
load_ml
python -c "import mujoco; print(mujoco.__version__)"
python ~/Projects/Happy-Baby-R1/test/test_ai_env.py
conda deactivate
```

### Test 2: Kiểm tra ROS 2 Foxy

```bash
exec zsh
load_ros
ros2 doctor --report
```

### Test 3: Build workspace

```bash
exec zsh
load_ros
cd ~/Projects/Happy-Baby-R1
cb
```

### Test 4: Giao tiếp DDS nội bộ

```bash
exec zsh
load_ros
python3 ~/Projects/Happy-Baby-R1/test/test_dds_node.py
```

### Test 5: Demo pub/sub ROS 2

Terminal 1:

```bash
exec zsh
load_ros
unset CYCLONEDDS_URI
ros2 run demo_nodes_cpp talker
```

Terminal 2:

```bash
exec zsh
load_ros
unset CYCLONEDDS_URI
ros2 run demo_nodes_py listener
```

## 7. Tài liệu liên quan

* Hướng dẫn cài Ubuntu: [ubuntu_20_04_lts_setup_guide.md](ubuntu_20_04_lts_setup_guide.md)
* Golden Machine: [../hardware/golden_machine_spec.md](../hardware/golden_machine_spec.md)
* Thiết lập mạng/DDS: [network_setup_checklist.md](network_setup_checklist.md)
* Quy trình rosbag2: [rosbag2_operation.md](rosbag2_operation.md)
* Trang chỉ mục an toàn: [../safety/safety_rules.md](../safety/safety_rules.md)
