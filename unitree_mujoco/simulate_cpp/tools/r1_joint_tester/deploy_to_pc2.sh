#!/bin/bash
PC2="unitree@192.168.123.164"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DST="$PC2:~/HappyBaby/low_level"

echo "========================================"
echo "  Syncing r1_joint_tuner to PC2..."
echo "  Source: $SRC"
echo "  Dest:   $DST"
echo "========================================"

rsync -avz --progress \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='build/' \
    --exclude='.git/' \
    "$SRC/" "$DST/"

echo ""
echo "========================================"
echo "  Sync complete!"
echo ""
echo "  Next steps on PC2 (via SSH):"
echo "  cd ~/HappyBaby/low_level"
echo "  mkdir -p build && cd build"
echo "  cmake .. && make -j4"
echo "  cd .. && ./start.sh eno1"
echo "========================================"
