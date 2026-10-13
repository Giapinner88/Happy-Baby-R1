#!/usr/bin/env bash
# Cài unit file. CỐ Ý không enable và không start.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
HB_ROOT="$(cd "$TELEOP_DIR/.." && pwd)"
RUN_USER="${RUN_USER:-unitree}"
INTERFACE="${UNITREE_NETWORK_INTERFACE:-eth10}"

sed -e "s|@HB_ROOT@|$HB_ROOT|g" -e "s|@RUN_USER@|$RUN_USER|g" -e "s|@INTERFACE@|$INTERFACE|g" \
    "$TELEOP_DIR/systemd/hb_teleop.service.in" | sudo tee /etc/systemd/system/hb_teleop.service >/dev/null
sudo systemctl daemon-reload

cat <<'MSG'
[OK] Đã cài hb_teleop.service.

Service CHƯA được enable và CHƯA chạy — đúng như thiết kế.
Teleop xung đột với hb_high_level.service (cùng ghi một nhóm khớp).

Trước khi chạy lần đầu, đọc docs/hardware_gate.md. Khi đã đủ điều kiện và CÓ
người giữ E-stop:

    sudo systemctl stop hb_high_level.service
    sudo systemctl start hb_teleop.service     # chỉ start, không enable
MSG
