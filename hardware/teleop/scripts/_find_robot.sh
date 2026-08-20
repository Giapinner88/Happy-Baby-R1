#!/usr/bin/env bash
# Dò địa chỉ robot. Source từ script khác; đặt sẵn ROBOT để bỏ qua bước dò.
#
# Thứ tự ưu tiên (cao -> thấp):
#   1) Biến môi trường ROBOT=unitree@<ip-or-hostname>
#   2) File RIÊNG của từng máy, đặt NGOÀI repo nên không bị commit hay ghi đè:
#        ${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}
#      Mẫu: hardware/teleop/config/robot.env.example
#   3) Ping lần lượt danh sách ứng viên.
#
# Dùng chung file cấu hình với stack HB để chỉ phải sửa địa chỉ ở một nơi.

find_robot() {
    if [ -n "${ROBOT:-}" ]; then
        echo "✅ Robot từ biến môi trường: $ROBOT"
        return 0
    fi

    local conf="${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}"
    if [ -f "$conf" ]; then
        # shellcheck disable=SC1090
        source "$conf"
        if [ -n "${ROBOT:-}" ]; then
            echo "✅ Robot từ $conf: $ROBOT"
            return 0
        fi
    fi

    echo "🔍 Đang tìm robot..."
    # HB-Hotspot (link trực tiếp, cùng mạng với Quest) trước; Tailscale để cuối
    # vì reachable ở mọi nơi nên không được che link cục bộ.
    for ip in "10.42.0.33" "192.168.12.2" "192.168.145.209" "192.168.1.33" "unitree-r1" "100.82.165.36"; do
        if ping -c 1 -W 1 "$ip" &> /dev/null; then
            ROBOT="unitree@$ip"
            echo "✅ Thấy robot ở: $ip"
            return 0
        fi
    done

    echo "❌ Không liên lạc được với robot." >&2
    echo "   Robot phải vào cùng mạng HB-Hotspot (host 10.42.0.1) hoặc LAN." >&2
    echo "   Đặt ROBOT=unitree@<ip> hoặc tạo $conf với dòng ROBOT=unitree@<ip>." >&2
    return 1
}
