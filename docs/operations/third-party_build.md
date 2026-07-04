# Third-Party Libraries Build & Integration Guide
**Project:** Unitree - Happy Baby (R1 Humanoid Research)
**Document ID:** HB-SPEC-004
**Author:** Integration Lead (Nguyễn Trọng Giáp)
**Status:** Draft / Working

Tài liệu này chuẩn hóa quy trình tải, biên dịch và tích hợp các thành phần chính thống từ nhà sản xuất hoặc dự án nguồn mở vào không gian làm việc của dự án. `third_party` chỉ là nơi chứa source/vendor lấy từ bên ngoài; cài đặt từ đầu đến cuối vẫn phải đi qua pipeline chính của Happy-Baby-R1. Quy trình đảm bảo tính cô lập tuyệt đối giữa thư viện điều khiển mức thấp (C++), lớp giao tiếp trung gian (ROS 2), môi trường nghiên cứu thuật toán (Python), và mã vận hành nội bộ của Happy-Baby-R1.

## 1. Bản chất Kiến trúc Third-Party

Hệ thống yêu cầu tích hợp nhiều module độc lập với vai trò chuyên biệt. Các
module này được chia thành bốn nhóm:

1. **Core Unitree / DDS:** `cyclonedds`, `unitree_sdk2`, `unitree_ros2`,
   `unitree_sdk2_python`.
2. **MuJoCo runtime và RL:** `unitree_mujoco`, `mjlab`, `unitree_rl_mjlab`,
   `cyclonedds-python` nếu cần Python DDS riêng.
3. **Isaac Sim / Isaac Lab:** `IsaacLab`, `unitree_sim_isaaclab` và các nested
   tool `xr_teleoperate`, `teleimager`.
4. **Happy-Baby-R1 internal code:** wrapper, launcher, policy runner, docs,
   config, log và artifact nằm ngoài `third_party`.

Ba module core phải hiểu rõ nhất:

1. **`unitree_sdk2` (Core C++ Library):** Thư viện liên kết động cấp hệ thống. Xử lý giao thức UDP nội bộ, đảm bảo tính tiền định cho hệ thống điều khiển Low-level ở tần số 1000Hz.
2. **`unitree_ros2` (ROS 2 Wrapper):** Bộ định nghĩa cấu trúc dữ liệu (Custom Messages). Ánh xạ các struct C++ thành tiêu chuẩn IDL của ROS 2 để vận hành Data Pipeline qua CycloneDDS.
3. **`unitree_sdk2_python` (Python Binding):** Lớp bọc Pybind11 kết nối môi trường Miniconda (`r1_env`) với thư viện C++ lõi, phục vụ quá trình huấn luyện Reinforcement Learning và High-level control ở 50Hz.

`third_party` đang được ignore bởi `.gitignore`, nên parent repo chỉ quản lý
tài liệu/pipeline, không commit nguyên source vendor.

### 1.1. Manifest nguồn chính thống

Các nguồn chính thống từ GitHub trong `third_party`:

| Thư mục | Tổ chức | Remote chính thống | Vai trò | Trạng thái local | Cách dùng trong repo |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `unitree_sdk2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2.git` | Core SDK C++ | Có `.git` remote chính thống | Build/install riêng theo mục 3 |
| `unitree_ros2` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_ros2.git` | ROS 2 wrapper/messages/examples | Có `.git` remote chính thống | Symlink vào `src/unitree_ros2` rồi build bằng colcon |
| `unitree_sdk2_python` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sdk2_python.git` | Python binding cho high-level code | Có `.git` remote chính thống | Cài editable trong Conda `r1_env` |
| `cyclonedds` | Eclipse Cyclone DDS | `https://github.com/eclipse-cyclonedds/cyclonedds.git` | DDS middleware source/reference | Có local checkout | Vendor/reference; ROS runtime ưu tiên package `ros-foxy-rmw-cyclonedds-cpp` |
| `unitree_mujoco` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_mujoco.git` | MuJoCo simulator/reference | Cài lại khi cần MuJoCo runtime | Vendor mô phỏng; script policy local nằm ngoài `third_party` |
| `mjlab` | MuJoCo Lab | `https://github.com/mujocolab/mjlab.git` | RL framework trên MuJoCo/MuJoCo Warp | Cài lại khi cần MJLab core | Cài qua pipeline chính `mjlab_installation.md`; tách khỏi Isaac Sim stack |
| `unitree_rl_mjlab` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_rl_mjlab.git` | Unitree robot RL tasks/train/play/deploy trên MJLab | Có `.git` remote chính thống | Cài qua pipeline chính `mjlab_installation.md`; artifact train/deploy để ngoài vendor khi dùng nội bộ |
| `cyclonedds-python` | Eclipse Cyclone DDS | `https://github.com/eclipse-cyclonedds/cyclonedds-python.git` | Python API/binding của CycloneDDS | Cài lại khi cần Python DDS API riêng | Không build chung workspace ROS |
| `IsaacLab` | NVIDIA / Isaac Sim | `https://github.com/isaac-sim/IsaacLab.git` | Isaac Lab framework độc lập | Cài lại theo pipeline Isaac Lab | Cài qua pipeline chính `isaaclab_installation.md` hoặc build trong Docker |
| `unitree_sim_isaaclab` | Unitree Robotics | `https://github.com/unitreerobotics/unitree_sim_isaaclab.git` | Unitree task/robot/DDS integration trên Isaac Lab | Cài lại theo pipeline Isaac Lab | Vendor mô phỏng Isaac Lab + Quest teleop; không build chung ROS workspace |
| `xr_teleoperate` | Unitree Robotics | `https://github.com/unitreerobotics/xr_teleoperate.git` | Quest/Vuer teleoperation stack | Có `.git` remote chính thống | Host teleop repo; dùng cùng Unitree Sim, có submodule riêng |

Trạng thái sau lần dọn hiện tại:

| Nhóm | Đang giữ trong `third_party` | Sẽ clone/cài lại khi cần |
| :--- | :--- | :--- |
| Core DDS/Unitree | `cyclonedds`, `unitree_sdk2`, `unitree_sdk2_python`, `unitree_ros2` | Không cần nếu checkout còn nguyên |
| Unitree RL MJLab | `unitree_rl_mjlab` | `mjlab` nếu cần core checkout riêng |
| MuJoCo runtime | - | `unitree_mujoco`, `cyclonedds-python` nếu cần |
| Isaac Lab / Quest teleop | `IsaacLab`, `unitree_sim_isaaclab`, `xr_teleoperate` | Đã clone; cài/build theo pipeline Isaac Lab |

Các thư mục thiếu local checkout phải được clone lại từ remote chính thống nếu
muốn pin commit hoặc audit provenance.

### 1.2. Third-party Isaac Lab / Unitree Sim

Luồng mô phỏng pick-and-place G1/Dex3 bằng Meta Quest 3 dùng nhiều lớp
third-party, nhưng toàn bộ cài đặt đi qua pipeline chính của Happy-Baby-R1:

| Thành phần | Vai trò | Trạng thái trong Happy-Baby-R1 |
| :--- | :--- | :--- |
| `IsaacLab` | Framework robot learning độc lập trên Isaac Sim | Vendor/upstream trong `third_party/IsaacLab` |
| `unitree_sim_isaaclab` | Unitree task/robot/DDS integration dùng Isaac Lab | Vendor/upstream trong `third_party/unitree_sim_isaaclab` |
| `xr_teleoperate` | Teleoperation host app, Vuer server, Quest 3 input bridge | Vendor/upstream trong `third_party/xr_teleoperate` |
| `televuer` | Vuer/WebXR layer | Submodule trong `third_party/xr_teleoperate/teleop/televuer` |
| `dex-retargeting` | Dex hand retargeting dependency | Submodule trong `third_party/xr_teleoperate/teleop/robot_control/dex-retargeting` |
| `teleimager` | Camera config, ZMQ/WebRTC stream cho browser/Quest | Submodule trong `unitree_sim_isaaclab` và `xr_teleoperate` |

Doc cài đặt gốc của Isaac Lab theo luồng Unitree nằm trong
`third_party/unitree_sim_isaaclab/doc/`, nhưng chỉ dùng để tham khảo version,
phụ thuộc và lỗi thường gặp. Người vận hành không chạy một pipeline riêng bên
trong third-party và cũng không copy/paste nguyên xi nếu không phù hợp với môi
trường chính. Repo chính gom luồng đó thành pipeline Happy-Baby-R1: lấy/cài
third-party, build image/env, verify ở
[isaaclab_installation.md](isaaclab_installation.md), vận hành/task/Quest ở
[teleop_quest3_vi.md](teleop_quest3_vi.md), artifact nội bộ để ngoài vendor tree.

### 1.3. Third-party MJLab

MJLab là third-party độc lập với `unitree_sim_isaaclab`. Nó dùng MuJoCo/MuJoCo
Warp và mượn phong cách API của Isaac Lab, nhưng không phải một phần của Isaac
Sim/Quest teleop stack.

| Thành phần | Vai trò | Trạng thái trong Happy-Baby-R1 |
| :--- | :--- | :--- |
| `mjlab` | RL framework trên backend MuJoCo/MuJoCo Warp | Vendor/upstream trong `third_party/mjlab` |
| `unitree_rl_mjlab` | Unitree task, train/play script và deploy config dùng MJLab | Vendor/upstream trong `third_party/unitree_rl_mjlab` |
| `mujoco-warp` | GPU backend cho MuJoCo training | Dependency của MJLab |
| `GMR` / `GVHMR` | Pipeline retarget motion sang asset mimic nếu dùng | External research checkout; artifact phải ghi provenance |

Doc cài đặt gốc nằm ở `third_party/mjlab/docs/source/installation.rst` và
`third_party/unitree_rl_mjlab/doc/setup_en.md`; pipeline cài đặt của repo chính
nằm ở [mjlab_installation.md](mjlab_installation.md).

**Quy tắc chuẩn:**

* Khi build ROS workspace từ root repo, luôn dùng `--base-paths src` để colcon không crawl toàn bộ `third_party`.
* `third_party` chỉ chứa mã vendor/upstream. Không đặt policy script, logger, replay helper, ONNX, CSV, wandb log, hoặc patch vận hành nội bộ trong `third_party`.
* Runtime MuJoCo của Happy-Baby-R1 đặt tại `sim/unitree_mujoco_policy/`; model/motion artifact đặt tại `data/models/unitree_mujoco_policy/`; CSV chạy thử đặt tại `data/sim_state_logs/`.
* Nếu cần sửa `unitree_mujoco` để chạy policy nội bộ, tạo wrapper hoặc bản glue trong `sim/unitree_mujoco_policy/`, không sửa trực tiếp file vendor.
* Nếu cần sửa `IsaacLab`, `unitree_sim_isaaclab`, `xr_teleoperate`, `teleimager`, `mjlab` hoặc `unitree_rl_mjlab`, ghi lại patch/provenance trong tài liệu vận hành trước khi coi đó là baseline lab.
* Nếu cần sửa asset motion train/play, ghi rõ tool sinh asset, task ID, motion file và checkpoint trong tài liệu/log thí nghiệm.

---

## 2. Chuẩn bị Không gian và Dependencies

Tránh lỗi con trỏ từ Git Submodule bằng cách tải mã nguồn độc lập. Tất cả thao
tác clone chạy từ root repo chính hoặc từ `third_party`, không clone lẫn vào
`sim/`, `src/` hoặc `docs/`.

### 2.1. Host dependencies tối thiểu cho DDS/ROS

```bash
# Cài đặt dependency cho luồng sinh Message
sudo apt update
sudo apt install -y ros-foxy-rosidl-generator-dds-idl ros-foxy-rmw-cyclonedds-cpp
```

### 2.2. Clone nhóm core giữ lại

Chỉ clone repo chưa tồn tại. Nếu muốn reset sạch một repo, xóa riêng repo đó
rồi chạy lại lệnh tương ứng.

```bash
cd ~/Projects/Happy-Baby-R1
mkdir -p third_party

test -d third_party/cyclonedds || \
  git clone https://github.com/eclipse-cyclonedds/cyclonedds.git third_party/cyclonedds

test -d third_party/unitree_sdk2 || \
  git clone https://github.com/unitreerobotics/unitree_sdk2.git third_party/unitree_sdk2

test -d third_party/unitree_ros2 || \
  git clone https://github.com/unitreerobotics/unitree_ros2.git third_party/unitree_ros2

test -d third_party/unitree_sdk2_python || \
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git third_party/unitree_sdk2_python
```

Nếu chỉ thiết lập máy vận hành Ubuntu 20.04 + ROS 2 Foxy, ba repo bắt buộc là
`unitree_sdk2`, `unitree_ros2`, `unitree_sdk2_python`. `cyclonedds` là
source/reference DDS; ROS runtime vẫn ưu tiên package apt
`ros-foxy-rmw-cyclonedds-cpp`.

### 2.3. Clone nhóm MuJoCo / MJLab khi cần

```bash
cd ~/Projects/Happy-Baby-R1
mkdir -p third_party

test -d third_party/unitree_mujoco || \
  git clone https://github.com/unitreerobotics/unitree_mujoco.git third_party/unitree_mujoco

test -d third_party/mjlab || \
  git clone https://github.com/mujocolab/mjlab.git third_party/mjlab

test -d third_party/unitree_rl_mjlab || \
  git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git third_party/unitree_rl_mjlab

test -d third_party/cyclonedds-python || \
  git clone https://github.com/eclipse-cyclonedds/cyclonedds-python.git third_party/cyclonedds-python
```

Sau khi clone `unitree_mujoco`, không chép policy local vào `third_party/unitree_mujoco/simulate_python`. Dùng hướng dẫn chạy policy ở [unitree_mujoco_policy_runtime.md](unitree_mujoco_policy_runtime.md).

MJLab core và Unitree RL MJLab dùng pipeline riêng ở
[mjlab_installation.md](mjlab_installation.md). Không đưa checkpoint
`logs/rsl_rl`, ONNX export, `.npz`, CSV motion hoặc wandb log vào baseline
vendor tree nếu đó là artifact thí nghiệm nội bộ.

### 2.4. Clone nhóm Isaac Lab / Unitree Sim khi cần

```bash
cd ~/Projects/Happy-Baby-R1
mkdir -p third_party

test -d third_party/IsaacLab || \
  git clone https://github.com/isaac-sim/IsaacLab.git third_party/IsaacLab

test -d third_party/unitree_sim_isaaclab || \
  git clone https://github.com/unitreerobotics/unitree_sim_isaaclab.git third_party/unitree_sim_isaaclab

test -d third_party/xr_teleoperate || \
  git clone https://github.com/unitreerobotics/xr_teleoperate.git third_party/xr_teleoperate

cd ~/Projects/Happy-Baby-R1/third_party/unitree_sim_isaaclab
git submodule update --init --depth 1

cd ~/Projects/Happy-Baby-R1/third_party/xr_teleoperate
git submodule update --init --depth 1
```

Isaac Lab / Unitree Sim không cài theo upstream verbatim. Dùng pipeline chính ở
[isaaclab_installation.md](isaaclab_installation.md), sau đó vận hành Quest 3 ở
[teleop_quest3_vi.md](teleop_quest3_vi.md).

### 2.5. Audit provenance của third-party

Sau khi clone hoặc pull, ghi lại remote và commit để tái lập môi trường:

```bash
cd ~/Projects/Happy-Baby-R1

for repo in third_party/*; do
  test -d "$repo/.git" || continue
  name="$(basename "$repo")"
  remote="$(git -C "$repo" remote get-url origin)"
  commit="$(git -C "$repo" rev-parse --short HEAD)"
  printf "%-24s %s %s\n" "$name" "$commit" "$remote"
done
```

Nếu repo third-party bị sửa local, không coi đó là baseline thầm lặng. Ghi lại
patch/provenance trong tài liệu vận hành hoặc tạo branch riêng trong repo
third-party đó.

---

## 3. Biên dịch Core SDK C++ (`unitree_sdk2`)

Quá trình này sinh ra thư viện tĩnh `libunitree_sdk2.a` cùng các thư viện DDS đi kèm, rồi khai báo chúng vào bộ nhớ đệm của Ubuntu 20.04.

```bash
cd ~/Projects/Happy-Baby-R1/third_party/unitree_sdk2
mkdir -p build
cd build

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
cd ~/Projects/Happy-Baby-R1
test -e src/unitree_ros2 || test -L src/unitree_ros2 || \
  ln -s ../third_party/unitree_ros2 src/unitree_ros2

# 3. Quét workspace ROS trong src và biên dịch bằng alias cb
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
python -m pip install -e .

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
* Cài Isaac Lab/Unitree Sim: [isaaclab_installation.md](isaaclab_installation.md)
* Cài MJLab: [mjlab_installation.md](mjlab_installation.md)
* Thực hành cầu nối third-party: [practice/05_third_party_bridge_exercise.md](practice/05_third_party_bridge_exercise.md)
* Quy trình vận hành: [SOP_v0.md](SOP_v0.md)
