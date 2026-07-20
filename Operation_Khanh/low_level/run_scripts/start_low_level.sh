#!/bin/bash
# start_low_level.sh - Khởi chạy đồng thời C++ DDS Bridge và Python GUI

set -e

# Bắt tín hiệu Ctrl+C và ngắt sạch toàn bộ
trap 'echo "\n🛑 Đã nhận lệnh thoát (Ctrl+C). Đang dọn dẹp..."; kill -9 $BRIDGE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTERFACE=${1:-eth10}
BRIDGE_BIN="$DIR/build/r1_dds_bridge"
GUI_PY="$DIR/python_gui/r1_professional_gui.py"

# Kiểm tra build
if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ Không tìm thấy C++ Bridge."
    echo "Vui lòng build trước: cd $DIR && mkdir -p build && cd build && cmake .. && make -j4"
    exit 1
fi

echo "========================================="
echo " KHỞI CHẠY LOW-LEVEL CONTROL (PC2 -> X11)"
echo " Interface DDS : $INTERFACE"
echo "========================================="

# 1. Setup DDS
export CYCLONEDDS_URI="file://$HOME/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml"

# 2. Chạy C++ Bridge dưới nền (background)
echo "📡 Đang khởi động C++ DDS Bridge..."
cd "$DIR"
"$BRIDGE_BIN" "$INTERFACE" &
BRIDGE_PID=$!

# Đợi một chút để Bridge sẵn sàng
sleep 1.5

# 3. Chạy Python GUI trên foreground
echo "🖥️ Đang khởi động Python GUI (X11 Forwarding)..."
python3 "$GUI_PY"

# 4. Dọn dẹp (nếu GUI tự đóng)
echo "🛑 GUI đã đóng. Script sẽ kết thúc và tự động gọi trap dọn dẹp..."

