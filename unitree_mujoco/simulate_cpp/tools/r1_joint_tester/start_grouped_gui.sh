#!/bin/bash
INTERFACE=${1:-auto}

# Kill existing
pkill -f "r1_grouped_gui.py" 2>/dev/null
pkill -f "r1_dds_bridge" 2>/dev/null

echo "Khởi động R1 DDS Bridge C++ (Grouped)..."
./build/r1_dds_bridge $INTERFACE &
BRIDGE_PID=$!

sleep 1

echo "Khởi động Python GUI (Grouped)..."
python3 r1_grouped_gui.py $INTERFACE

echo "Đang dọn dẹp..."
kill $BRIDGE_PID 2>/dev/null
