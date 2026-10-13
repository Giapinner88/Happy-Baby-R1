#!/usr/bin/env bash
# Đồng bộ teleop lên robot. CHỈ copy file: không enable service, không arm motor.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEST="${DEST:-/home/unitree/HB}"
COMMAND="${1:-diff}"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    GREEN=$'\033[32m'; RED=$'\033[31m'; YELLOW=$'\033[33m'; RESET=$'\033[0m'
else
    GREEN=; RED=; YELLOW=; RESET=
fi
ok() { printf '%s[OK]%s %s\n' "$GREEN" "$RESET" "$*"; }
fail() { printf '%s[FAIL]%s %s\n' "$RED" "$RESET" "$*" >&2; }
wait_msg() { printf '%s[WAIT]%s %s\n' "$YELLOW" "$RESET" "$*"; }

# Catch local source/test errors before connecting or asking for sudo.
case "$COMMAND" in
    deploy|runtime) "$SCRIPT_DIR/preflight.sh" ;;
esac

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_find_robot.sh"
find_robot

# shellcheck disable=SC1091
source "$HB_ROOT/integration/scripts/deploy_auth.sh"
hb_ssh_setup 900
trap 'hb_deploy_auth_cleanup "$SSH_CONTROL_PATH" "$ROBOT"' EXIT
hb_deploy_auth_open_ssh "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT"

verify_robot_ssh() {
    if "${SSH[@]}" -o BatchMode=yes "$ROBOT" true; then
        ok "SSH tới R1 sẵn sàng: $ROBOT"
        return 0
    fi
    local host="${ROBOT#*@}"
    fail "Không tới được SSH R1 tại $ROBOT; chưa copy hay restart service nào."
    echo "       Kiểm tra R1 đang cùng mạng, IP SSH trong config/teleop.yaml, hoặc dùng" >&2
    echo "       HB_TELEOP_ROBOT_OVERRIDE=unitree@<IP-da-xac-minh> $0 $COMMAND" >&2
    ip route get "$host" 2>&1 || true
    return 1
}
verify_robot_ssh

if [[ "$COMMAND" == "runtime" ]]; then
    hb_deploy_auth_enable_sudo "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT"
fi

RSYNC_BASE+=(--itemize-changes)
EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/' --exclude '.pytest_cache/'
          --exclude 'logs/' --exclude 'docs/' --exclude 'results/' --exclude '*.env'
          --exclude 'third_party/xr_teleoperate_r1_a5_845b25b/**/__pycache__/')

sync_teleop() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${EXCLUDES[@]}" \
        "$TELEOP_DIR/" "$ROBOT:$DEST/teleop/"
}

verify_remote_runtime() {
    local local_sidecar local_sha remote_sha remote_help remote_sidecar local_launcher remote_launcher
    local relative
    # These files carry the live control pipeline too. Checking just the
    # launcher/sidecar can leave a missing or old queue/logger helper unnoticed.
    for relative in src/teleop/hardware/stream_io.py \
                    scripts/teleop/run_r1_vendor_targets.py \
                    scripts/teleop/run_r1_vendor_ik_stream.py \
                    scripts/teleop/quest_bridge.py \
                    scripts/teleop/quest_tracking.js; do
        local_sha="$(sha256sum "$TELEOP_DIR/$relative" | awk '{print $1}')"
        remote_sha="$("${SSH[@]}" "$ROBOT" "sha256sum '$DEST/teleop/$relative'" 2>/dev/null | awk '{print $1}')"
        if [[ -z "$remote_sha" || "$local_sha" != "$remote_sha" ]]; then
            fail "runtime file trên robot không khớp local: $relative"
            return 1
        fi
    done
    local_sidecar="$TELEOP_DIR/src/teleop/hardware/high_level_sidecar.py"
    remote_sidecar="$DEST/teleop/src/teleop/hardware/high_level_sidecar.py"
    local_sha="$(sha256sum "$local_sidecar" | awk '{print $1}')"
    remote_sha="$("${SSH[@]}" "$ROBOT" "sha256sum '$remote_sidecar'" 2>/dev/null | awk '{print $1}')"
    if [[ -z "$remote_sha" || "$local_sha" != "$remote_sha" ]]; then
        fail "sidecar trên robot không khớp local (local=$local_sha remote=${remote_sha:-missing})."
        return 1
    fi

    local_launcher="$TELEOP_DIR/scripts/run_robot_teleop.sh"
    remote_launcher="$DEST/teleop/scripts/run_robot_teleop.sh"
    local_sha="$(sha256sum "$local_launcher" | awk '{print $1}')"
    remote_sha="$("${SSH[@]}" "$ROBOT" "sha256sum '$remote_launcher'" 2>/dev/null | awk '{print $1}')"
    if [[ -z "$remote_sha" || "$local_sha" != "$remote_sha" ]]; then
        fail "launcher robot-local trên robot không khớp local (local=$local_sha remote=${remote_sha:-missing})."
        return 1
    fi

    if ! remote_help="$("${SSH[@]}" "$ROBOT" \
        "PYTHONPATH='$DEST/teleop/src' python3 -m teleop.hardware.high_level_sidecar --help 2>&1")"; then
        fail "không chạy được sidecar --help trên robot."
        return 1
    fi
    grep -q -- '--stream-contract' <<<"$remote_help" || {
        fail "sidecar trên robot thiếu --stream-contract."
        return 1
    }
    grep -q -- '--joint-limits-only' <<<"$remote_help" || {
        fail "sidecar trên robot thiếu --joint-limits-only."
        return 1
    }
    ok "sidecar và launcher robot-local khớp local; CLI H4 sẵn sàng"
}

verify_remote_quest_endpoint() {
    local report
    if ! report="$("${SSH[@]}" "$ROBOT" '
set -euo pipefail
source /etc/hb/teleop/runtime.env
runtime_ip="$(ip -4 -o addr show dev "$HB_TELEOP_RUNTIME_HOST_INTERFACE" scope global | awk '\''NR == 1 {split($4, a, "/"); print a[1]}'\'')"
[[ -n "$runtime_ip" ]]
systemctl is-active --quiet avahi-daemon.service
avahi-resolve-host-name -4 "$HB_TELEOP_RUNTIME_HOSTNAME" | awk -v ip="$runtime_ip" '\''$2 == ip { found=1 } END { exit !found }'\''
openssl x509 -in "$HB_TELEOP_RUNTIME_CERT_FILE" -noout -ext subjectAltName \
    | grep -Fq "DNS:$HB_TELEOP_RUNTIME_HOSTNAME"
openssl x509 -in "$HB_TELEOP_RUNTIME_CERT_FILE" -noout -ext subjectAltName \
    | grep -Fq "IP Address:$runtime_ip"
printf "hostname=%s ip=%s port=%s\n" "$HB_TELEOP_RUNTIME_HOSTNAME" "$runtime_ip" "$HB_TELEOP_RUNTIME_PORT"
')"; then
        fail "Quest endpoint không đạt mDNS/certificate preflight trên robot."
        return 1
    fi
    ok "Quest endpoint đã xác nhận trên robot: $report"
}

case "$COMMAND" in
    diff)
        echo "== dry-run: sẽ thay đổi những gì =="
        sync_teleop --dry-run
        ;;
    deploy)
        sync_teleop
        verify_remote_runtime
        echo
        ok "Đã copy file lên $ROBOT:$DEST/teleop/"
        echo "     KHÔNG có service nào được bật và KHÔNG có motor nào được ghi."
        echo "     Bật service là thao tác thủ công; đọc docs/hardware_gate.md trước."
        ;;
    runtime)
        # Sync YAML/scripts and regenerate the robot-owned runtime.env,
        # certificate and systemd service. This is the short path after an IP,
        # hostname or robot-runtime setting changes; it does not rebuild C++.
        sync_teleop
        verify_remote_runtime
        "${SSH[@]}" -tt "$ROBOT" \
            "HB_ROOT='$DEST' bash '$DEST/teleop/scripts/bootstrap_robot_runtime.sh'"
        runtime_install_args="--start"
        [[ "${HB_TELEOP_APPLY_RATE:-0}" == "1" ]] && runtime_install_args+=" --apply-rate"
        if ! "${SSH[@]}" -tt "$ROBOT" \
            "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' $runtime_install_args"; then
            fail "Cài runtime/upgrade bị từ chối hoặc thất bại; xem lỗi ở trên. Nếu refuse restart: đưa high-level về DISARMED, idle và có status mới rồi chạy lại."
            exit 1
        fi
        runtime_settings="$("${SSH[@]}" "$ROBOT" 'source /etc/hb/teleop/runtime.env; printf "%s %s\n" "$HB_TELEOP_RUNTIME_PORT" "$HB_TELEOP_RUNTIME_CAMERA_ENABLED"')"
        read -r runtime_port runtime_camera_enabled <<<"$runtime_settings"
        if [[ ! "$runtime_port" =~ ^[0-9]+$ || ! "$runtime_camera_enabled" =~ ^[01]$ ]]; then
            fail "Không đọc được port/camera_enabled từ runtime.env trên robot."
            exit 1
        fi
        runtime_status=""
        sidecar_process=""
        sidecar_ready=""
        listening=""
        camera_health=""
        tracking_health=""
        ready=0
        wait_msg "Đợi WebXR, H4 sidecar và cổng $runtime_port sẵn sàng (tối đa 30s)..."
        for _ in $(seq 1 30); do
            runtime_status="$("${SSH[@]}" "$ROBOT" 'systemctl is-active hb_teleop_cert.service hb_teleop_runtime.service' 2>/dev/null || true)"
            listening="$("${SSH[@]}" "$ROBOT" "ss -H -ltn 'sport = :$runtime_port' 2>/dev/null" || true)"
            sidecar_process="$("${SSH[@]}" "$ROBOT" "pgrep -af '[t]eleop.hardware.high_level_sidecar.*--stream-contract h4' 2>/dev/null || true" 2>/dev/null || true)"
            sidecar_ready="$("${SSH[@]}" "$ROBOT" "latest=\$(find '$DEST/teleop/logs/robot_runtime' -name sidecar.log -printf '%T@ %p\\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-); test -n \"\$latest\" && grep -F '[SIDECAR] initial rt/lowstate ready; H4 STOP preamble armed' \"\$latest\" 2>/dev/null" 2>/dev/null || true)"
            tracking_health="$("${SSH[@]}" "$ROBOT" "curl -fkSs --max-time 2 'https://127.0.0.1:$runtime_port/tracking/status'" 2>/dev/null || true)"
            if [[ "$runtime_camera_enabled" == "1" ]]; then
                camera_health="$("${SSH[@]}" "$ROBOT" "curl -fkSs --max-time 2 'https://127.0.0.1:$runtime_port/camera/health'" 2>/dev/null || true)"
            else
                camera_health='{"transport": "webrtc"}'
            fi
            if grep -qx 'active' <<<"$runtime_status" &&
               [[ "$(grep -c '^active$' <<<"$runtime_status")" -eq 2 ]] &&
               [[ -n "$listening" ]] && [[ -n "$sidecar_process" ]] && [[ -n "$sidecar_ready" ]] && [[ "$camera_health" == *'"transport": "webrtc"'* ]] && [[ "$tracking_health" == *'"tracking_protocol": "xr-frame-v3"'* ]]; then
                ready=1
                break
            fi
            sleep 1
        done
        camera_deploy_error=0
        if (( ready )); then
            ok "Robot-local runtime, WebXR port $runtime_port và H4 sidecar ACTIVE"
            ok "Kênh tracking xr-frame-v3 sẵn sàng trên robot; cần xác nhận mẫu XR thật sau khi ENTER VR trên Quest"
            if [[ "$runtime_camera_enabled" == "1" ]]; then
                ok "Camera WebRTC signaling sẵn sàng; kết quả kiểm tra camera nguồn ở bên dưới, media tới Quest cần kiểm tra trên kính"
            fi
            verify_remote_quest_endpoint
            # Read only this invocation: old warnings must not describe a new run.
            invocation="$("${SSH[@]}" "$ROBOT" 'systemctl show hb_teleop_runtime.service -p InvocationID --value')"
            runtime_report="$("${SSH[@]}" "$ROBOT" "journalctl _SYSTEMD_INVOCATION_ID=$invocation --no-pager -o cat 2>/dev/null" || true)"
            grep -E '^\[RUNTIME\] (Quest URL|Target limit mode):' <<<"$runtime_report" | tail -2 || true
            if grep -qE '^\[RUNTIME PREFLIGHT\] (DEGRADED|WARN)' <<<"$runtime_report"; then
                camera_errors="$(grep -E '^\[RUNTIME PREFLIGHT\] (DEGRADED|WARN)' <<<"$runtime_report")"
                camera_deploy_error=1
                fail "DEPLOY DEGRADED: camera robot chưa sẵn sàng; nút A có thể không hiển thị camera robot. Teleop tay/đầu vẫn hoạt động trong passthrough."
                printf '%s%s%s\n' "$RED" "$camera_errors" "$RESET" >&2
            elif [[ "$runtime_camera_enabled" == "1" ]]; then
                if grep -q '^\[RUNTIME PREFLIGHT\] OK R1 videohub camera read' <<<"$runtime_report"; then
                    ok "Camera nguồn đã đọc và giải mã ảnh; còn cần kiểm tra hình WebRTC trên Quest"
                else
                    camera_deploy_error=1
                    fail "Không xác nhận được camera nguồn trong lần chạy hiện tại; kiểm tra journal, không coi signaling là camera đã chạy."
                fi
            fi
            quest_hostname="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
            echo "     Quest bookmark: https://$quest_hostname:$runtime_port/?ws=wss://$quest_hostname:$runtime_port"
            echo "     Trên Quest: vào cùng LAN/VLAN, mở bookmark, chấp nhận cert lần đầu, ENTER VR."
            # Nonzero reports incomplete camera deployment while preserving teleop.
            (( camera_deploy_error == 0 )) || exit 2
        else
            fail "Runtime chưa sẵn sàng (service/WebXR/H4 sidecar/rt-lowstate, cổng $runtime_port)."
            printf '%s\n' "$runtime_status" >&2
            printf '[H4 sidecar process]\n%s\n' "$sidecar_process" >&2
            printf '[H4 initial rt/lowstate]\n%s\n' "${sidecar_ready:-chưa sẵn sàng}" >&2
            "${SSH[@]}" "$ROBOT" "systemctl --no-pager -l status hb_teleop_runtime.service; journalctl -u hb_teleop_runtime.service -b -n 60 --no-pager -o short-iso" >&2 || true
            exit 1
        fi
        ;;
    status)
        # shellcheck disable=SC2029
        "${SSH[@]}" "$ROBOT" "cat $DEST/teleop/src/SOURCE.txt 2>/dev/null || echo 'chưa deploy'"
        ;;
    *)
        echo "Dùng: $0 {diff|deploy|runtime|status}" >&2
        exit 2
        ;;
esac
