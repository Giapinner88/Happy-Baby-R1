# Third-Party Libraries Build & Integration Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-004
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Approved / Final

Tài liệu này chuẩn hóa quy trình tải, biên dịch và tích hợp các thành phần chính thống từ nhà sản xuất hoặc dự án nguồn mở vào không gian làm việc của dự án. Quy trình đảm bảo tính cô lập tuyệt đối giữa thư viện điều khiển mức thấp (C++), lớp giao tiếp trung gian (ROS 2), môi trường nghiên cứu thuật toán (Python), và mã vận hành nội bộ của Happy-Baby-R1.

## 1. Bản chất Kiến trúc Third-Party

Hệ thống yêu cầu tích hợp ba module độc lập với vai trò chuyên biệt:
1. **`unitree_sdk2` (Core C++ Library):** Thư viện liên kết động cấp hệ thống. Xử lý giao thức UDP nội bộ, đảm bảo tính tiền định cho hệ thống điều khiển Low-level ở tần số 1000Hz.
2. **`unitree_ros2` (ROS 2 Wrapper):** Bộ định nghĩa cấu trúc dữ liệu (Custom Messages). Ánh xạ các struct C++ thành tiêu chuẩn IDL của ROS 2 để vận hành Data Pipeline qua CycloneDDS.
3. **`unitree_sdk2_python` (Python Binding):** Lớp bọc Pybind11 kết nối môi trường Miniconda (`r1_env`) với thư viện C++ lõi, phục vụ quá trình huấn luyện Reinforcement Learning và High-level control ở 50Hz.

### 1.1. Manifest nguồn chính thống

Các nguồn chính thống từ GitHub trong `third_party`:

| Thư mục | Tổ chức | Remote chính thống | Vai trò | Trạng thái local | Cách dùng trong repo |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `unitree_sdk2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2.git` | Core SDK C++ | Có `.git` remote chính thống | Build/install riêng theo mục 3 |
| `unitree_ros2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_ros2.git` | ROS 2 wrapper/messages/examples | Có `.git` remote chính thống | Symlink vào `src/unitree_ros2` rồi build bằng colcon |
| `unitree_sdk2_python` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2_python.git` | Python binding cho high-level code | Có `.git` remote chính thống | Cài editable trong Conda `r1_env` |
| `unitree_mujoco` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_mujoco.git` | MuJoCo simulator/reference | Giữ sạch theo upstream | Vendor mô phỏng; script policy local nằm ngoài `third_party` |
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
sudo apt install -y ros-foxy-rosidl-generator-dds-idl ros-foxy-rmw-cyclonedds-cpp

# Dọn dẹp thư mục lỗi (nếu có) và tải mã nguồn nguyên bản
cd ~/Projects/Happy-Baby-R1/third_party
rm -rf unitree_sdk2 unitree_ros2 unitree_sdk2_python unitree_mujoco cyclonedds cyclonedds-python

git clone https://github.com/unitreerobotics/unitree_sdk2.git
git clone https://github.com/unitreerobotics/unitree_ros2.git
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
git clone https://github.com/unitreerobotics/unitree_mujoco.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds-python.git
```

Nếu chỉ thiết lập máy vận hành Ubuntu 20.04 + ROS 2 Foxy, ba repo bắt buộc là `unitree_sdk2`, `unitree_ros2`, `unitree_sdk2_python`. `unitree_mujoco`, `cyclonedds`, và `cyclonedds-python` là nguồn tham chiếu/vendor phục vụ nghiên cứu, debug hoặc mô phỏng; không build chung bằng `colcon build` từ root repo.

Sau khi clone `unitree_mujoco`, không chép policy local vào `third_party/unitree_mujoco/simulate_python`. Dùng hướng dẫn chạy policy ở [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md).

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

Sử dụng liên kết mềm (Symlink) để Colcon nhận diện được package mà không phá vỡ cấu trúc vật lý của dự án.

```bash
# 1. Khởi động Zsh và kích hoạt môi trường ROS 2
exec zsh
load_ros

# 2. Tạo liên kết định tuyến từ third_party sang src
ln -s ~/Projects/Happy-Baby-R1/third_party/unitree_ros2 ~/Projects/Happy-Baby-R1/src/unitree_ros2

# 3. Quét workspace ROS trong src và biên dịch bằng alias cb
cd ~/Projects/Happy-Baby-R1
cb

# 4. Nạp lại nguồn để ghi nhận hệ thống tin nhắn mới
source install/setup.zsh
```
> **Lưu ý:** Bỏ qua các cảnh báo (Warnings) dạng `-Wnon-c-typedef-for-linkage` hoặc `-Wunused-variable` từ các tệp `example` do bộ phân tích tĩnh của Clang tạo ra.

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
