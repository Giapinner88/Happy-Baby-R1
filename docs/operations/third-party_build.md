# Third-Party Libraries Build & Integration Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-004
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này chuẩn hóa quy trình tải, biên dịch và tích hợp các thành phần chính thống từ nhà sản xuất hoặc dự án nguồn mở vào không gian làm việc của dự án. Quy trình đảm bảo tính cô lập tuyệt đối giữa thư viện điều khiển mức thấp (C++), lớp giao tiếp trung gian (ROS 2), môi trường nghiên cứu thuật toán (Python), và mã vận hành nội bộ của Happy-Baby-R1.

## 1. Bản chất Kiến trúc Third-Party

Hệ thống yêu cầu tích hợp các module độc lập với vai trò chuyên biệt:
1. **`unitree_sdk2` (Core C++ Library):** Thư viện liên kết động cấp hệ thống. Xử lý giao thức UDP nội bộ, đảm bảo tính tiền định cho hệ thống điều khiển Low-level ở tần số 1000Hz.
2. **`unitree_ros2` (ROS 2 Wrapper):** Bộ định nghĩa cấu trúc dữ liệu (Custom Messages). Ánh xạ các struct C++ thành tiêu chuẩn IDL của ROS 2 để vận hành Data Pipeline qua CycloneDDS.
3. **`unitree_sdk2_python` (Python Binding):** Lớp bọc Pybind11 kết nối môi trường Miniconda (`r1_env`) với thư viện C++ lõi, phục vụ quá trình huấn luyện Reinforcement Learning và High-level control ở 50Hz.
4. **`unitree_mujoco` (MuJoCo simulator/reference):** Simulator chính thống của Unitree để kiểm thử DDS/control loop.
5. **`unitree_rl_mjlab` (RL training/deploy/policies):** Nguồn policy ONNX mẫu, motion artifact, training/play scripts và C++ deploy controller của Unitree.

### 1.1. Manifest nguồn chính thống

Các nguồn chính thống từ GitHub trong `third_party`:

| Thư mục | Tổ chức | Remote chính thống | Vai trò | Trạng thái local | Cách dùng trong repo |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `unitree_sdk2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2.git` | Core SDK C++ | Có `.git` remote chính thống | Build/install riêng theo mục 3 |
| `unitree_ros2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_ros2.git` | ROS 2 wrapper/messages/examples | Có `.git` remote chính thống | Symlink vào `src/unitree_ros2` rồi build bằng colcon |
| `unitree_sdk2_python` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2_python.git` | Python binding cho high-level code | Có `.git` remote chính thống | Cài editable trong Conda `r1_env` |
| `unitree_mujoco` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_mujoco.git` | MuJoCo simulator/reference | Giữ sạch theo upstream | Vendor mô phỏng; script policy local nằm ngoài `third_party` |
| `unitree_rl_mjlab` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_rl_mjlab.git` | RL training/play/deploy, policy ONNX mẫu | Có `.git` remote chính thống | Vendor policy/training; symlink artifact vào `data/models/unitree_mujoco_policy` |
| `cyclonedds` | Eclipse Cyclone DDS | `https://github.com/eclipse-cyclonedds/cyclonedds.git` | DDS middleware source/reference | Local hiện không có `.git` để verify commit | Vendor/reference; ROS runtime ưu tiên package `ros-foxy-rmw-cyclonedds-cpp` |
| `cyclonedds-python` | Eclipse Cyclone DDS | `https://github.com/eclipse-cyclonedds/cyclonedds-python.git` | Python API/binding của CycloneDDS | Local hiện không có `.git` để verify commit | Chỉ dùng khi cần Python DDS API riêng; không build chung workspace ROS |

Các thư mục không có `.git` local cần được clone lại từ remote chính thống nếu muốn pin commit hoặc audit provenance.

**Quy tắc chuẩn:**

* Khi build ROS workspace từ root repo, luôn dùng `--base-paths src` để colcon không crawl toàn bộ `third_party`.
* `third_party` chỉ chứa mã vendor/upstream. Không đặt policy script, logger, replay helper, ONNX, CSV, wandb log, hoặc patch vận hành nội bộ trong `third_party`.
* Runtime MuJoCo của Happy-Baby-R1 đặt tại `sim/unitree_mujoco_policy/`; model/motion artifact đặt tại `data/models/unitree_mujoco_policy/`; CSV chạy thử đặt tại `data/sim_state_logs/`.
* Nếu cần sửa `unitree_mujoco` để chạy policy nội bộ, tạo wrapper hoặc bản glue trong `sim/unitree_mujoco_policy/`, không sửa trực tiếp file vendor.

---

## 2. Chuẩn bị Không gian và Dependencies

Tránh lỗi con trỏ từ Git Submodule bằng cách tải mã nguồn độc lập. Bổ sung các công cụ sinh mã IDL chuyên biệt cho CycloneDDS.

```bash
# Cài đặt dependency cho luồng sinh Message
sudo apt update
sudo apt install -y ros-foxy-rosidl-generator-dds-idl ros-foxy-rmw-cyclonedds-cpp libyaml-cpp-dev

# Tải mã nguồn nguyên bản. Chỉ xóa thư mục cũ nếu chắc chắn đó là bản clone lỗi.
cd ~/Projects/Happy-Baby-R1/third_party

git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone https://github.com/unitreerobotics/unitree_mujoco.git
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds-python.git
```

Nếu chỉ thiết lập máy vận hành Ubuntu 20.04 + ROS 2 Foxy, ba repo bắt buộc là `unitree_sdk2`, `unitree_ros2`, `unitree_sdk2_python`. `unitree_mujoco`, `unitree_rl_mjlab`, `cyclonedds`, và `cyclonedds-python` là nguồn tham chiếu/vendor phục vụ nghiên cứu, debug hoặc mô phỏng; không build chung bằng `colcon build` từ root repo.

Sau khi clone `unitree_mujoco`, không chép policy local vào `third_party/unitree_mujoco/simulate_python`. Dùng hướng dẫn chạy policy ở [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md).

Sau khi clone `unitree_rl_mjlab`, symlink policy artifact mẫu của Unitree sang thư mục model chuẩn của dự án:

```bash
mkdir -p ~/Projects/Happy-Baby-R1/data/models/unitree_mujoco_policy
cd ~/Projects/Happy-Baby-R1

ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy98.onnx
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy.onnx

ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/exported/policy.onnx \
  data/models/unitree_mujoco_policy/policy_dance.onnx
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/exported/policy.onnx.data \
  data/models/unitree_mujoco_policy/policy.onnx.data
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/dance1_subject2/params/dance1_subject2.npz \
  data/models/unitree_mujoco_policy/dance1_subject2.npz
```

---

## 3. Biên dịch Core SDK C++ (`unitree_sdk2`)

Quá trình này sinh ra thư viện tĩnh `libunitree_sdk2.a` cùng các thư viện DDS đi kèm, rồi khai báo chúng vào bộ nhớ đệm của Ubuntu 20.04.

```bash
cd ~/Projects/Happy-Baby-R1/third_party/unitree_sdk2
mkdir build && cd build

# Cấu hình biên dịch bằng Clang thay vì GCC mặc định
cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ ..

# Biên dịch và đẩy thư viện vào System-level
make -j$(nproc)
sudo make install
sudo ldconfig
```

---

## 4. Tích hợp ROS 2 Wrapper (`unitree_ros2`)

Sử dụng liên kết mềm (Symlink) tới từng package ROS thật để Colcon nhận diện được package mà không phá vỡ cấu trúc vật lý của dự án. Không symlink root `third_party/unitree_ros2` vào `src`, vì repo upstream chứa nhiều workspace con.

```bash
# 1. Kích hoạt môi trường ROS 2 bằng Python hệ thống, không dùng Conda.
# Nếu prompt còn hiển thị (r1_env), chạy conda deactivate trước.
conda deactivate 2>/dev/null || true
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 2. Tạo liên kết định tuyến từ các package Unitree sang src
cd ~/Projects/Happy-Baby-R1
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api src/unitree_api
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/unitree_go
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg src/unitree_hg
ln -sfn ~/Projects/Happy-Baby-R1/third_party/unitree_ros2/example/src src/unitree_ros2_example

# 3. Quét workspace ROS trong src và biên dịch.
# Ép Python về /usr/bin/python3 để tránh CMake dùng Python trong Conda.
colcon build --base-paths src --symlink-install \
  --packages-select unitree_api unitree_go unitree_hg unitree_ros2_example \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

# 4. Nạp lại nguồn để ghi nhận hệ thống tin nhắn mới
source install/setup.bash
```
> **Lưu ý:** Bỏ qua các cảnh báo (Warnings) dạng `-Wnon-c-typedef-for-linkage` hoặc `-Wunused-variable` từ các tệp `example` do bộ phân tích tĩnh của Clang tạo ra.

Nếu build báo `ModuleNotFoundError: No module named 'em'` và log đang chạy `/home/.../miniconda3/envs/r1_env/bin/python3`, nghĩa là terminal ROS vẫn đang nhiễm Conda. Thoát env và build lại với block trên.

---

## 5. Cài đặt Python Binding (`unitree_sdk2_python`)

Cài đặt lớp bọc AI vào không gian cô lập của Conda, tuyệt đối không can thiệp vào Python hệ thống.

```bash
# 1. Khởi động Zsh và kích hoạt môi trường Machine Learning
exec zsh
load_ml

# 2. Di chuyển vào thư mục binding
cd ~/Projects/Happy-Baby-R1/third_party/unitree_sdk2_python

# 3. Cài đặt theo dạng Editable Mode
pip install -e .

# 4. Thoát môi trường an toàn
conda deactivate
```

---

## 6. Kịch bản kiểm thử (Integration Tests)

Sau khi hoàn tất cài đặt, Integration Lead phải thực thi 3 kịch bản sau để xác thực tính toàn vẹn của chuỗi phụ thuộc.

### Test 1: Xác thực thư viện SDK (Hệ điều hành)
Kiểm tra xem hệ thống đã nhận diện được thư viện C++ tĩnh chưa.
```bash
ls -lh /usr/local/lib/libunitree_sdk2.a
# Output mong đợi: Hiển thị thông tin tệp tĩnh .a cùng dung lượng cấp phát.
```

### Test 2: Xác thực Message Pipeline (ROS 2)
Kiểm tra khả năng phân giải IDL của ROS 2. Bật terminal mới:
```bash
exec zsh
load_ros
ros2 interface show unitree_api/msg/Request
# Output mong đợi: Trả về cấu trúc chi tiết của bản tin Request (chứa header, parameter...). Nếu báo "Unknown interface", quá trình biên dịch IDL đã thất bại.
```

### Test 3: Xác thực Python Binding (AI Environment)
Kiểm tra khả năng triệu gọi thư viện C++ từ không gian thuật toán.
```bash
exec zsh
load_ml
python -c "import unitree_sdk2py; print('Unitree Python Binding: OK')"
conda deactivate
# Output mong đợi: "Unitree Python Binding: OK". Nếu báo ImportError, liên kết giữa r1_env và thư viện SDK/DDS đã đứt gãy.
```

## 7. Tài liệu liên quan

* Hướng dẫn môi trường dev: [development_environment_setup_guide.md](development_environment_setup_guide.md)
* Triển khai DDS: [dds_implementation.md](dds_implementation.md)
* Cấu hình mạng tĩnh: [network_configuration_static_ethernet.md](network_configuration_static_ethernet.md)
* Runtime policy MuJoCo: [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md)
* Thực hành cầu nối third-party: [practice/05_third_party_bridge_exercise.md](practice/05_third_party_bridge_exercise.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)
