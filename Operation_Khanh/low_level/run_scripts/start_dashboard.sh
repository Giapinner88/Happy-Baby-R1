#!/bin/bash
# start_dashboard.sh - Khởi chạy SENSOR DASHBOARD ở chế độ CHỈ ĐỌC (READ-ONLY)
# Cho phép mở song song với file start_combined.sh mà không gây xung đột.

set -e

# Bắt tín hiệu Ctrl+C và ngắt sạch toàn bộ
trap 'echo "\n🛑 Đã nhận lệnh thoát (Ctrl+C). Đang dọn dẹp..."; kill -9 $BRIDGE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTERFACE=${1:-eth10}
BRIDGE_BIN="$DIR/build/r1_dds_bridge"
GUI_PY="$DIR/python_gui/r1_sensor_dashboard.py"

# Kiểm tra build
if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ Không tìm thấy C++ Bridge."
    echo "Vui lòng build trước: cd $DIR && mkdir -p build && cd build && cmake .. && make -j4"
    exit 1
fi

echo "========================================="
echo " KHỞI CHẠY SENSOR DASHBOARD (READ-ONLY)"
echo " Giao diện này CHỈ ĐỌC thông số cảm biến,"
echo " hoàn toàn không điều khiển hay can thiệp"
echo " vào tiến trình chạy của robot."
echo " Interface DDS : $INTERFACE"
echo "========================================="

# 1. Setup DDS
export CYCLONEDDS_URI="file://$HOME/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml"

# 2. Chạy C++ Bridge dưới nền (background)
# Chú ý: BẮT BUỘC PHẢI CÓ CỜ --monitor ĐỂ KHÔNG PHÁT LỆNH DDS
echo "📡 Đang khởi động C++ DDS Bridge ở chế độ Chỉ Nghe (--monitor)..."
cd "$DIR"
"$BRIDGE_BIN" "$INTERFACE" --monitor &
BRIDGE_PID=$!

# Đợi một chút để Bridge kết nối DDS
sleep 1.5

# 3. Chạy Python GUI trên foreground
echo "🖥️ Đang hiển thị Giao diện Dashboard (X11)..."
python3 "$GUI_PY"

# 4. Dọn dẹp (nếu GUI tự đóng)
echo "🛑 GUI đã đóng. Script sẽ kết thúc và tự động dọn dẹp tiến trình ngầm..."
