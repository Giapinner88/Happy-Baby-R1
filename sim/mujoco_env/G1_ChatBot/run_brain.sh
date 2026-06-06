#!/bin/bash
# run_brain.sh – Khởi động Não Bộ (chatbot_subscriber)
#
# Dùng trong terminal 1:
#   ./run_brain.sh          (local, loopback)
#   ./run_brain.sh eth0     (robot thật)

cd "$(dirname "$0")"
source /home/khanh248/Documents/HB/Mujoco/.venv/bin/activate

echo "========================================"
echo " Não Bộ (Chatbot Subscriber) đang bật"
echo "========================================"

IFACE="${1:-lo}"
echo "Card mạng: $IFACE"
echo ""

python3 chatbot_subscriber.py "$IFACE"
