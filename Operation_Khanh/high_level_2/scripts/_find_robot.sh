#!/usr/bin/env bash
# Dò địa chỉ robot. Source từ các script khác; đặt sẵn biến ROBOT để bỏ qua bước dò.
#   ROBOT=unitree@192.168.12.2 ./scripts/probe_robot.sh

find_robot() {
    if [ -n "$ROBOT" ]; then
        return 0
    fi
    echo "🔍 Đang tìm robot..."
    for ip in "192.168.12.2" "192.168.1.33" "100.82.165.36" "unitree-r1"; do
        if ping -c 1 -W 1 "$ip" &> /dev/null; then
            ROBOT="unitree@$ip"
            echo "✅ Thấy robot ở: $ip"
            return 0
        fi
    done
    echo "❌ Không liên lạc được với robot."
    exit 1
}
