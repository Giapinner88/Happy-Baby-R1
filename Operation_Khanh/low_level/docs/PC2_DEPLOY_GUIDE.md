# Hướng dẫn Deploy R1 Joint Tuner sang PC2

**PC1**: khanh248 — máy phát triển (có internet, chạy PySide6)
**PC2**: unitree@ubuntu (192.168.123.164) — Jetson aarch64, kết nối trực tiếp robot

---

## Tổng quan kiến trúc

Kiến trúc giống hệt PC1, chỉ chạy trên máy khác:

```
PC2 (Jetson aarch64)  ──── DDS ────► ROBOT
    r1_dds_bridge (C++)
         │ UDP 127.0.0.1
    Python GUI (PyQt5)
```

---

## PC2 không có internet — ảnh hưởng gì?

| Thành phần | Tình trạng       | Ghi chú                               |
| ------------ | ------------------ | -------------------------------------- |
| PyQt5        | ✅ Đã cài (apt) | Không cần làm gì                   |
| unitree_sdk2 | ✅ ~/unitree_sdk2  | lib/aarch64/libunitree_sdk2.a có sẵn |
| cmake, g++   | ✅ Có sẵn        | cmake 3.16, g++ 9.4                    |
| numpy        | ❓ Cần kiểm tra  | Xem Bước 0                           |

**Kết luận: Không ảnh hưởng nhiều.** Chỉ cần kiểm tra numpy.

---

## Bước 0 — Kiểm tra numpy trên PC2

```bash
python3 -c "import numpy; print('numpy OK:', numpy.__version__)"
```

Nếu thiếu:

```bash
# Cách 1 - thử apt:
sudo apt install -y python3-numpy

# Cách 2 - copy wheel từ PC1 qua LAN:
# Trên PC1:
pip3 download "numpy<2.0" --dest /tmp/npy_whl \
    --platform manylinux2014_aarch64 --python-version 38 --only-binary=:all:
rsync -avz /tmp/npy_whl/ unitree@192.168.123.164:/tmp/npy_whl/
# Trên PC2:
pip3 install /tmp/npy_whl/numpy*.whl
```

---

## Bước 1 — Thay đổi code trên PC1

> Tất cả thay đổi code thực hiện trên **PC1**, sau đó sync lên PC2 bằng rsync.

### 1.1 Tạo gui/qt_compat.py

```python
"""Qt compatibility shim — PySide6 (PC1) / PyQt5 (PC2 Jetson)"""
try:
    from PySide6.QtWidgets import *
    from PySide6.QtCore import Qt, Signal, Slot, QTimer, QEvent, QCoreApplication, QSize
    from PySide6.QtGui import QAction, QColor, QFont, QIcon, QKeySequence, QPainter, QPen, QPalette
    _QT = "PySide6"
except ImportError:
    from PyQt5.QtWidgets import *
    from PyQt5.QtWidgets import QAction
    from PyQt5.QtCore import Qt, pyqtSignal as Signal, pyqtSlot as Slot
    from PyQt5.QtCore import QTimer, QEvent, QCoreApplication, QSize
    from PyQt5.QtGui import QColor, QFont, QIcon, QKeySequence, QPainter, QPen, QPalette
    _QT = "PyQt5"
```

### 1.2 Sửa imports trong 7 file GUI + entry point

| File                   | Import thay thế                                                                                                                                                                                                                                |
| ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| theme.py               | `from .qt_compat import Qt, QColor, QPalette`                                                                                                                                                                                                 |
| imu_panel.py           | `from .qt_compat import Qt, QWidget, QVBoxLayout, QGridLayout, QLabel, QFrame, QFont`                                                                                                                                                         |
| telemetry_graph.py     | `from .qt_compat import Qt, QTimer, QWidget, QVBoxLayout, QLabel, QGridLayout, QFrame, QFont, QPainter, QColor, QPen`                                                                                                                         |
| robot_3d_view.py       | `from .qt_compat import Qt, QTimer, QWidget, QVBoxLayout, QLabel, QPainter, QPen, QFont`                                                                                                                                                      |
| joint_panel.py         | `from .qt_compat import (Qt, Signal, QSize, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QFrame, QScrollArea, QSizePolicy, QFont, QPainter, QColor, QPen)`                                                                         |
| control_panel.py       | `from .qt_compat import (Qt, Signal, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QPushButton, QLabel, QFrame, QSlider, QFont, QIcon)`                                                                                                     |
| main_window.py         | `from .qt_compat import (Qt, QTimer, Signal, QEvent, QCoreApplication, QAction, QKeySequence, QFont, QIcon, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar, QMenuBar, QMenu, QToolBar, QLabel, QFrame, QMessageBox)` |
| r1_professional_gui.py | `from gui.qt_compat import QApplication, QMessageBox, Qt`                                                                                                                                                                                     |

### 1.3 Tạo CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.10)
project(r1_joint_tuner)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(UNITREE_SDK2_DIR $ENV{HOME}/unitree_sdk2)

include_directories(
    ${UNITREE_SDK2_DIR}/include
    ${UNITREE_SDK2_DIR}/thirdparty/include
)

add_executable(r1_dds_bridge r1_dds_bridge.cpp)

target_link_libraries(r1_dds_bridge
    ${UNITREE_SDK2_DIR}/lib/aarch64/libunitree_sdk2.a
    pthread
)
```

### 1.4 Cập nhật start.sh (universal — chạy được cả PC1 lẫn PC2)

```bash
#!/bin/bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTERFACE=${1:-eno1}

python3 -c "import PySide6" 2>/dev/null || python3 -c "import PyQt5" 2>/dev/null || {
    echo "ERROR: Chua co PySide6 hay PyQt5!"
    echo "  sudo apt install python3-pyqt5"
    exit 1
}

BRIDGE_BIN="$DIR/build/r1_dds_bridge"
[ ! -f "$BRIDGE_BIN" ] && {
    echo "ERROR: Chua build bridge!"
    echo "  cd $DIR && mkdir -p build && cd build && cmake .. && make -j4"
    exit 1
}

echo "Starting DDS Bridge (interface: $INTERFACE)..."
"$BRIDGE_BIN" "$INTERFACE" &
BRIDGE_PID=$!
sleep 1
kill -0 $BRIDGE_PID 2>/dev/null || { echo "Bridge failed!"; exit 1; }

trap "echo Stopping...; kill $BRIDGE_PID 2>/dev/null" EXIT SIGINT SIGTERM

cd "$DIR"
python3 r1_professional_gui.py "$INTERFACE"
```

### 1.5 Tạo deploy_to_pc2.sh

```bash
#!/bin/bash
PC2="unitree@192.168.123.164"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Syncing r1_joint_tuner to PC2..."
rsync -avz --progress \
    --exclude='__pycache__' --exclude='*.pyc' \
    --exclude='build/' --exclude='.git/' \
    "$SRC/" "$PC2:~/HappyBaby/r1_joint_tuner/"

echo ""
echo "Done! Next on PC2:"
echo "  cd ~/HappyBaby/r1_joint_tuner && mkdir -p build && cd build"
echo "  cmake .. && make -j4"
echo "  cd .. && ./start.sh eno1"

# Phase 2 (uncomment khi can):
# rsync -avz /path/to/policies/ $PC2:~/HappyBaby/policies/
# rsync -avz /path/to/run_scripts/ $PC2:~/HappyBaby/run_scripts/
```

---

## Bước 2 — Tạo thư mục trên PC2 (SSH)

```bash
mkdir -p ~/HappyBaby/r1_joint_tuner
mkdir -p ~/HappyBaby/policies
mkdir -p ~/HappyBaby/run_scripts
```

---

## Bước 3 — Sync code từ PC1

```bash
# Trên PC1:
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp/src/r1_test_joints
chmod +x deploy_to_pc2.sh
bash deploy_to_pc2.sh
```

---

## Bước 4 — Build C++ bridge trên PC2

```bash
# Trên PC2 (SSH):
cd ~/HappyBaby/r1_joint_tuner
mkdir -p build && cd build
cmake ..
make -j4
```

Kiểm tra:

```bash
file ~/HappyBaby/r1_joint_tuner/build/r1_dds_bridge
# Phải ra: ELF 64-bit LSB executable, ARM aarch64
```

---

## Bước 5 — Chạy GUI (terminal vật lý hoặc NoMachine)

```bash
cd ~/HappyBaby/r1_joint_tuner
chmod +x start.sh
./start.sh eno1
```

Kiểm tra hoạt động:

1. GUI mở với 26 ô khớp
2. ENABLE → data robot hiện
3. W/S → robot di chuyển, nhả phím → dừng
4. Space → Emergency Stop từ bất cứ đâu

---

## Cấu trúc thư mục cuối cùng trên PC2

```
~/HappyBaby/
│
├── r1_joint_tuner/             Phase 1 - Joint Tuner GUI
│   ├── gui/
│   │   ├── qt_compat.py        [NEW] shim PySide6/PyQt5
│   │   ├── theme.py
│   │   ├── joint_panel.py
│   │   ├── control_panel.py
│   │   ├── imu_panel.py
│   │   ├── telemetry_graph.py
│   │   ├── robot_3d_view.py
│   │   ├── main_window.py
│   │   └── __init__.py
│   ├── bridge/
│   │   ├── udp_client.py
│   │   ├── robot_state.py
│   │   └── __init__.py
│   ├── utils/
│   │   ├── joint_names.py
│   │   ├── safe_limits.py
│   │   └── __init__.py
│   ├── r1_dds_bridge.cpp
│   ├── CMakeLists.txt          [NEW] build standalone
│   ├── r1_professional_gui.py
│   ├── start.sh                universal PC1+PC2
│   ├── deploy_to_pc2.sh        [NEW] sync script
│   └── build/
│       └── r1_dds_bridge       ARM aarch64 binary
│
├── policies/                   Phase 2 - .pt files
└── run_scripts/                Phase 2 - run_*.py
```

---

## Xử lý sự cố

| Vấn đề                      | Nguyên nhân      | Giải pháp                                              |
| ------------------------------ | ------------------ | -------------------------------------------------------- |
| `No module named numpy`      | numpy chưa cài   | `sudo apt install python3-numpy`                       |
| `Bridge failed`              | Sai interface      | Thử`eth0`, `enp3s0`, xem `ip link show`           |
| `Undefined symbol` khi cmake | Include path sai   | Kiểm tra UNITREE_SDK2_DIR                               |
| GUI không mở                 | Không có DISPLAY | Dùng terminal vật lý hoặc NoMachine                  |
| `Port in use`                | Tiến trình cũ   | `pkill -f r1_dds_bridge; pkill -f r1_professional_gui` |

---

## Phase 2: Deploy Policy & Run Scripts

```bash
# Thêm vào deploy_to_pc2.sh hoặc chạy thủ công:
rsync -avz /path/to/policies/ unitree@192.168.123.164:~/HappyBaby/policies/
rsync -avz /path/to/run_scripts/ unitree@192.168.123.164:~/HappyBaby/run_scripts/
```

Kiểm tra PyTorch trên PC2 (cần cho policy):

```bash
python3 -c "import torch; print(torch.__version__)"
# Jetson thường đã có JetPack PyTorch
```
