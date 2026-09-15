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

# Xác minh một địa chỉ đúng là robot HB chứ không phải máy lạ đang giữ IP đó.
# Im lặng; trả 0 nếu đúng.
verify_robot() {
    local target="${1:?verify_robot cần unitree@<ip>}"
    ssh -o BatchMode=yes -o ConnectTimeout=6 -o StrictHostKeyChecking=accept-new \
        "$target" 'test -d "$HOME/HB/teleop/src/teleop"' 2>/dev/null
}

# Dừng ngay nếu địa chỉ đang dùng không phải robot. Gọi TRƯỚC mọi thao tác ghi.
assert_robot() {
    local target="${1:?assert_robot cần unitree@<ip>}"
    if ! verify_robot "$target"; then
        echo "❌ $target không phải robot HB (không thấy ~/HB/teleop/src/teleop)." >&2
        echo "   Robot có thể đang tắt và IP đã bị máy khác lấy mất." >&2
        echo "   Tìm lại theo MAC c0:3a:55:f1:41:84 rồi sửa ROBOT trong" >&2
        echo "   ${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}" >&2
        return 1
    fi
}

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
    for ip in "10.42.0.33" "192.168.1.108" "192.168.12.2" "192.168.145.209" "192.168.1.33" "unitree-r1" "100.82.165.36"; do
        ping -c 1 -W 1 "$ip" &> /dev/null || continue
        # Ping thôi thì chưa đủ. wlan0 của robot lấy IP động, nên khi robot tắt
        # thì DHCP cấp IP đó cho máy khác — máy lạ vẫn trả lời ping y hệt. Đã
        # xảy ra thật ngày 2026-08-29 ở 192.168.1.104. Chỉ nhận khi đúng là máy
        # có package teleop.
        if verify_robot "unitree@$ip"; then
            ROBOT="unitree@$ip"
            echo "✅ Thấy robot ở: $ip"
            return 0
        fi
        echo "   $ip trả lời ping nhưng không phải robot — bỏ qua." >&2
    done

    echo "❌ Không liên lạc được với robot." >&2
    echo "   Robot phải vào cùng mạng HB-Hotspot (host 10.42.0.1) hoặc LAN." >&2
    echo "   Đặt ROBOT=unitree@<ip> hoặc tạo $conf với dòng ROBOT=unitree@<ip>." >&2
    return 1
}
