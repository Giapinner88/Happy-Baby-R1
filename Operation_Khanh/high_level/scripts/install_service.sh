#!/usr/bin/env bash
# install_service.sh — Cài systemd service cho HB high_level trên robot để
# TỰ KHỞI ĐỘNG lúc bật nguồn, chạy headless (điều khiển thuần gamepad R3-1).
#
# Chạy TRÊN ROBOT:  cd ~/HB/high_level_2 && bash scripts/install_service.sh
set -e

DIR="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$DIR/build/run_r1"
USER_NAME="$(id -un)"
UNIT=/etc/systemd/system/hb_high_level.service

if [ ! -x "$BIN" ]; then
    echo "❌ Chưa thấy binary: $BIN"
    echo "   Build trước: bash scripts/build.sh"
    exit 1
fi

echo ">>> Tạo $UNIT (User=$USER_NAME, WorkingDirectory=$DIR/build)"
sudo tee "$UNIT" >/dev/null <<EOF
[Unit]
Description=HB R1 High-Level Runner (headless, gamepad R3-1)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
WorkingDirectory=$DIR/build
ExecStart=$BIN
Environment=DISPLAY=
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable hb_high_level
echo ""
echo "✓ Đã cài. Lệnh dùng:"
echo "    sudo systemctl start hb_high_level     # chạy ngay"
echo "    sudo systemctl status hb_high_level    # trạng thái"
echo "    journalctl -u hb_high_level -f         # xem log realtime"
echo "    sudo systemctl disable --now hb_high_level  # tắt tự khởi động"
echo ""
echo "⚠ Đảm bảo dev_no_keyboard: true trong config/tuning.yaml (headless)."
