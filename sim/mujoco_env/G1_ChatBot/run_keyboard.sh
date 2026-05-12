#!/bin/bash
# run_keyboard.sh – Khởi động Bàn Phím (keyboard_publisher)
#
# Dùng trong terminal 2:
#   ./run_keyboard.sh         (local, loopback)
#   ./run_keyboard.sh eth0    (robot thật)

cd "$(dirname "$0")"
source /home/khanh248/Documents/HB/Mujoco/.venv/bin/activate

echo "========================================"
echo " Bàn Phím (Keyboard Publisher) đang bật"
echo "========================================"

IFACE="${1:-lo}"
echo "Card mạng: $IFACE"
echo ""

python3 keyboard_publisher.py "$IFACE"
