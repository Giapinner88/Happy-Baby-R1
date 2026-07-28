#!/usr/bin/env bash
# Script build và chạy dds_probe trên robot, sau đó kéo dữ liệu CSV về máy cục bộ.
set -e

cd "$(dirname "$0")/.."
source scripts/_find_robot.sh
find_robot

DEST="${DEST:-~/HB/high_level_2}"
# Lấy card mạng từ tuning.yaml.
IFACE="${IFACE:-$(awk '/^network_interface:/ {print $2}' config/tuning.yaml)}"
IFACE="${IFACE:-eth10}"

STAMP="$(date +%Y%m%d_%H%M%S)"
REMOTE_CSV="/tmp/r1_probe_${STAMP}.csv"
LOCAL_DIR="logs/probe"

echo ">>> Build dds_probe trên robot ..."
ssh "$ROBOT" "cd $DEST/build 2>/dev/null || (mkdir -p $DEST/build && cd $DEST/build && cmake .. -DCMAKE_BUILD_TYPE=Release); cd $DEST/build && cmake --build . --target dds_probe -j\$(nproc)"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo " Card mạng: $IFACE"
echo ""
echo " BÂY GIỜ: sang terminal 1, cho robot NHẢY TRỌN 1 ĐIỆU."
echo " Nhảy xong thì quay lại đây bấm Ctrl-C để lấy báo cáo."
echo "═══════════════════════════════════════════════════════════"
echo ""

# Chạy dds_probe trên robot qua ssh.
ssh -t "$ROBOT" "cd $DEST/build && ./dds_probe $IFACE $REMOTE_CSV" || true

echo ""
echo ">>> Kéo CSV về máy này ..."
mkdir -p "$LOCAL_DIR"
if scp "$ROBOT:$REMOTE_CSV" "$LOCAL_DIR/" 2>/dev/null; then
    echo "✓ $LOCAL_DIR/$(basename $REMOTE_CSV)"
    echo ""
    echo "  Cột: qd_<khớp> (lệnh) / q_<khớp> (thực) / dq_<khớp> / tau_<khớp>"
    echo "  Vẽ qd vs q của L_ank_pitch là thấy ngay ankle có theo kịp lệnh không."
else
    echo "⚠ Không lấy được CSV (probe có chạy đủ lâu không?)"
fi
