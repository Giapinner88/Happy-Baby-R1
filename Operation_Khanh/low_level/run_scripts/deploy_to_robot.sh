#!/usr/bin/env bash
# Đẩy code sang máy tính nhúng của robot (PC2) rồi build bên đó.
#
#   ./run_scripts/deploy_to_robot.sh          # rsync + build từ xa
#   ./run_scripts/deploy_to_robot.sh --no-build  # chỉ rsync
#
# Yêu cầu: robot đã bật, PC nối được với robot qua LAN hoặc Tailscale.
set -e

# Cho phép override qua biến môi trường, nếu không thì tự động dò
if [ -z "$ROBOT" ]; then
    echo "🔍 Đang tìm robot..."
    for ip in "192.168.12.2" "100.82.165.36" "unitree-r1"; do
        if ping -c 1 -W 1 "$ip" &> /dev/null; then
            ROBOT="unitree@$ip"
            echo "✅ Đã thấy robot tại: $ip"
            break
        fi
    done
fi

if [ -z "$ROBOT" ]; then
    echo "❌ Lỗi: Không thể ping tới bất kỳ địa chỉ nào của robot (192.168.12.2, 100.82.165.36, unitree-r1)."
    echo "Vui lòng kiểm tra lại mạng LAN hoặc Tailscale."
    exit 1
fi

DEST="${DEST:-~/HB/low_level}"

cd "$(dirname "$0")/.."

echo ">>> Rsync sang $ROBOT:$DEST ..."
# Tự động tạo thư mục đích trên robot nếu chưa có
ssh "$ROBOT" "mkdir -p $DEST"

# Bỏ qua thư mục build cục bộ và các file cache
rsync -avz --progress \
    --exclude 'build/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    ./ "$ROBOT:$DEST/"

if [[ "$1" != "--no-build" ]]; then
    echo ">>> Build trên robot ..."
    ssh "$ROBOT" "cd $DEST && mkdir -p build && cd build && cmake .. && make -j4"
fi

echo ""
echo "✓ Xong. Chạy trên robot để bật Giao diện Test:"
echo "    ssh -Y $ROBOT"
echo "    cd $DEST"
echo "    ./run_scripts/start_grouped_gui.sh auto   # Hoặc start_tuning_gui.sh auto"
