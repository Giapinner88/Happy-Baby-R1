# Hướng dẫn start nhanh (Unitree MuJoCo – Python)

Mục tiêu: chạy mô phỏng + chạy script điều khiển/policy trong 2 terminal.

## 0) Điều kiện tối thiểu
- OS: Linux (Ubuntu/Debian ok)
- Có Python 3 và môi trường ảo trong repo: `.venv/`
- Có hiển thị GUI (MuJoCo viewer cần OpenGL)

## 1) Kích hoạt môi trường và cài dependencies (1 lần)
Tại thư mục gốc repo (thư mục có `.venv/`):

```bash
source .venv/bin/activate
python -m pip install -U pip

# Cài unitree sdk2 python từ source trong repo (editable)
pip install -e ./unitree_sdk2_python

# Các package thường dùng cho simulate_python + policy
pip install mujoco onnxruntime pygame numpy
```

Nếu bạn gặp lỗi kiểu:
`Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH`
- Repo này đã có `cyclonedds/` (và thường đã build sẵn ở `cyclonedds/build/`). Thử export:

```bash
export CYCLONEDDS_HOME="$PWD/cyclonedds/build"
export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME:${CMAKE_PREFIX_PATH:-}"
```

Sau đó chạy lại `pip install -e ./unitree_sdk2_python`.

## 2) Start mô phỏng (Terminal 1)
```bash
source .venv/bin/activate
cd unitree_mujoco/simulate_python

# Start simulator (viewer + DDS bridge)
python unitree_mujoco2.py
```

Gợi ý cấu hình robot/domain/interface ở `unitree_mujoco/simulate_python/config.py`:
- `ROBOT = "g1"` (hoặc `go2`, ...)
- `DOMAIN_ID = 1`
- `INTERFACE = "lo"` (loopback cho simulation)

## 3) Start điều khiển/policy (Terminal 2)
Mở terminal mới:

```bash
source .venv/bin/activate
cd unitree_mujoco/simulate_python

# Ví dụ: policy 98
python run98.py
```

Một số script khác cùng thư mục (tuỳ nhu cầu):
- `run480.py`, `run480_2.py`
- `run98_2.py`
- `run_dance.py`, `run_policy_dance.py`

Lưu ý:
- Các script policy thường load file `.onnx` ngay trong thư mục `simulate_python/` (ví dụ `policy98.onnx`).
- `run98.py` có hỗ trợ gamepad (pygame). Nếu không có gamepad, script sẽ chuyển sang keyboard.

## 4) Dừng chương trình
- Dừng simulator hoặc controller: `Ctrl+C` tại terminal tương ứng.

## 5) Troubleshooting nhanh
- Không mở được viewer / lỗi OpenGL: đảm bảo đang chạy trong môi trường có GUI. Có thể cần cài thêm:
  ```bash
  sudo apt-get update
  sudo apt-get install -y libgl1-mesa-glx libglfw3
  ```
- Không nhận DDS message: đảm bảo cả simulator và controller dùng cùng `DOMAIN_ID` và `INTERFACE` (mặc định repo dùng `1` và `lo`).
