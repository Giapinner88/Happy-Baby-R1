#!/bin/bash
# start_tuning_gui.sh - Khởi chạy PD Tuning GUI

set -e
trap 'echo "\n🛑 Đang dọn dẹp..."; kill -9 $BRIDGE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTERFACE=${1:-eth10}
BRIDGE_BIN="$DIR/build/r1_dds_bridge"
GUI_PY="$DIR/python_gui/r1_tuning_gui.py"

if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ Không tìm thấy C++ Bridge."
    echo "Vui lòng build trước: cd $DIR && mkdir -p build && cd build && cmake .. && make -j4"
    exit 1
fi

export CYCLONEDDS_URI="file://$HOME/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml"

echo "📡 Đang khởi động C++ DDS Bridge..."
cd "$DIR"
"$BRIDGE_BIN" "$INTERFACE" &
BRIDGE_PID=$!

sleep 1.5

echo "🖥️ Đang khởi động Python GUI (Tuning)..."
cd "$DIR/python_gui"
python3 "$GUI_PY" "$INTERFACE"

echo "🛑 GUI đã đóng. Script sẽ kết thúc và tự động dọn dẹp tiến trình ngầm..."
