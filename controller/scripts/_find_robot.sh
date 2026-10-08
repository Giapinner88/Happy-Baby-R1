#!/usr/bin/env bash
# Dò địa chỉ robot. Source từ các script khác; đặt sẵn biến ROBOT để ưu tiên nó.
#
# Thứ tự ưu tiên (cao -> thấp):
#   1) Biến môi trường ROBOT=unitree@<ip-or-hostname>
#   2) File RIÊNG của từng máy, đặt NGOÀI thư mục HB nên không bị ghi đè khi copy
#      folder giữa máy dev và máy trạm:
#        ${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}   (chứa dòng ROBOT=unitree@<ip>)
#      Mẫu: integration/config/robot.env.example
#   3) Thử lần lượt danh sách ứng viên (phủ cả mạng máy dev lẫn máy trạm).
#
# QUAN TRỌNG: địa chỉ ở mức 1 và 2 vẫn được KIỂM TRA. Robot hay nhảy wifi (đã xảy
# ra 2026-08-05: rời hotspot 10.42.0.x sang LAN 192.168.1.x), nên một địa chỉ ghim
# sẵn mà chết thì phải tự dò tiếp chứ không được dừng cả lượt deploy. Tailscale
# nằm cuối danh sách và là lưới cuối cùng: nó tới được robot ở bất kỳ mạng nào.

# Ứng viên theo thứ tự: link trực tiếp / LAN trước (nhanh), Tailscale sau cùng
# (reachable ở mọi nơi nên không được che link cục bộ).
HB_ROBOT_CANDIDATES=(
    "10.42.0.33"        # hotspot máy dev
    "192.168.145.209"
    "192.168.12.2"
    "192.168.1.33"      # LAN sự kiện
    "unitree-r1"
    "192.168.123.164"
    "192.168.1.104"
    "192.168.1.108"
    "10.233.177.5"
    "${HB_ROBOT_TAILSCALE:-100.82.165.36}"   # Tailscale: lưới cuối
)

# Thật sự dùng được hay không = SSH vào được, không phải ping được. sshd chết mà
# máy vẫn trả lời ping là chuyện có thật, và deploy thì cần SSH chứ không cần ICMP.
_robot_ssh_ok() {
    local probe_timeout="${HB_ROBOT_PROBE_TIMEOUT:-2}"
    [[ "$probe_timeout" =~ ^[0-9]+([.][0-9]+)?$ ]] || probe_timeout=2

    # ConnectTimeout only covers TCP connect. Tailscale SSH can connect and
    # then wait at authentication, so enforce a total wall-clock timeout too.
    timeout --signal=TERM --kill-after=1s "${probe_timeout}s" \
        ssh -o BatchMode=yes \
        -o ControlMaster=no \
        -o ControlPath=none \
        -o ConnectionAttempts=1 \
        -o ConnectTimeout="$probe_timeout" \
        -o StrictHostKeyChecking=no \
        "$1" true 2>/dev/null
}

_robot_direct_link() {
    local ip="$1" route=""
    command -v ip >/dev/null 2>&1 || return 1
    route="$(ip -4 route get "$ip" 2>/dev/null || true)"
    # A direct connected route has `dev ...` without an intermediate `via`.
    # This makes the active Ethernet/hotspot subnet win over stale candidates.
    [[ -n "$route" && "$route" == *" dev "* && "$route" != *" via "* ]]
}

_robot_probe_candidates() {
    local ip candidate probe_dir idx pid
    local -a direct_candidates=()
    local -a other_candidates=()
    local -a ordered_candidates=()
    local -a probe_pids=()
    declare -A seen_candidates=()

    # Probe candidates on currently connected subnets first. If `ip` is not
    # available, retain the original fixed candidate order as a fallback.
    for ip in "${HB_ROBOT_CANDIDATES[@]}"; do
        [[ -n "${seen_candidates[$ip]+x}" ]] && continue
        seen_candidates[$ip]=1
        if [[ "$ip" == *.*.*.* ]] && _robot_direct_link "$ip"; then
            direct_candidates+=("$ip")
        else
            other_candidates+=("$ip")
        fi
    done

    ordered_candidates=("${direct_candidates[@]}" "${other_candidates[@]}")
    probe_dir="$(mktemp -d "${TMPDIR:-/tmp}/hb-robot-scan.XXXXXX")" || return 1

    # Probe every candidate in parallel. Sequential SSH timeouts made a newly
    # plugged LAN invisible for tens of seconds while old addresses were tried.
    idx=0
    for ip in "${ordered_candidates[@]}"; do
        candidate="unitree@$ip"
        (
            if _robot_ssh_ok "$candidate"; then
                : > "$probe_dir/$idx.ok"
            fi
        ) &
        probe_pids+=("$!")
        idx=$((idx + 1))
    done

    for pid in "${probe_pids[@]}"; do
        wait "$pid" || true
    done

    idx=0
    for ip in "${ordered_candidates[@]}"; do
        candidate="unitree@$ip"
        if [[ -f "$probe_dir/$idx.ok" ]]; then
            rm -rf "$probe_dir"
            ROBOT="$candidate"
            case "$ip" in
                100.*) echo "✅ Thấy robot qua Tailscale: $ip (mạng cục bộ không tới được)" ;;
                *)     echo "✅ Thấy robot ở: $ip" ;;
            esac
            return 0
        fi
        idx=$((idx + 1))
    done
    rm -rf "$probe_dir"
    return 1
}

find_robot() {
    local source_label=""
    local preferred_robot="${ROBOT:-}"
    local scan_round=0
    local scan_interval="${HB_ROBOT_SCAN_INTERVAL:-1}"
    local scan_max_rounds="${HB_ROBOT_SCAN_MAX_ROUNDS:-0}"
    local scan_once="${HB_ROBOT_SCAN_ONCE:-0}"

    # Default is persistent discovery: deployment may start before the robot's
    # Ethernet/Wi-Fi link is up. Set HB_ROBOT_SCAN_ONCE=1 or a positive
    # HB_ROBOT_SCAN_MAX_ROUNDS for a bounded diagnostic probe.
    [[ "$scan_interval" =~ ^[0-9]+$ ]] || scan_interval=3
    [[ "$scan_max_rounds" =~ ^[0-9]+$ ]] || scan_max_rounds=0

    if [ -n "${ROBOT:-}" ]; then
        source_label="biến môi trường"
    else
        local conf="${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}"
        if [ -f "$conf" ]; then
            # shellcheck disable=SC1090
            source "$conf"
            [ -n "${ROBOT:-}" ] && {
                preferred_robot="$ROBOT"
                source_label="$conf"
            }
        fi
    fi

    while :; do
        scan_round=$((scan_round + 1))

        if [ -n "$preferred_robot" ]; then
            if _robot_ssh_ok "$preferred_robot"; then
                ROBOT="$preferred_robot"
                echo "✅ Robot từ $source_label: $ROBOT"
                return 0
            fi
            echo "⚠️  $preferred_robot ($source_label) không SSH được; vòng $scan_round sẽ quét toàn bộ danh sách..."
        elif [ "$scan_round" -eq 1 ]; then
            echo "🔍 Đang quét toàn bộ danh sách robot..."
        fi

        if _robot_probe_candidates; then
            return 0
        fi

        if [ "$scan_once" = "1" ] || {
            [ "$scan_max_rounds" -gt 0 ] && [ "$scan_round" -ge "$scan_max_rounds" ];
        }; then
            break
        fi

        echo "⏳ Chưa thấy robot sau vòng $scan_round; quét lại sau ${scan_interval}s (Ctrl+C để dừng)."
        sleep "$scan_interval"
    done

    echo "❌ Không liên lạc được với robot ở bất kỳ địa chỉ nào, kể cả Tailscale."
    echo "   Kiểm tra robot đã bật chưa, rồi xem: tailscale status"
    echo "   Hoặc đặt ROBOT=unitree@<ip> / sửa ${HB_ROBOT_CONF:-$HOME/.config/hb/robot.env}."
    exit 1
}
