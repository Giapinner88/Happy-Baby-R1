#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then HB_COLOR=1; else HB_COLOR="${HB_COLOR:-0}"; fi
source "$SCRIPT_DIR/status_log.sh"
ROBOT="${ROBOT:-}"
DEST="${DEST:-/home/unitree/HB}"
COMMAND="${1:-diff}"
OPTION="${2:-}"

# Dò robot (thư viện dùng chung của integration). Có thể đặt sẵn
# ROBOT=unitree@<ip-or-hostname> để bỏ qua bước dò.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/find_robot.sh"
find_robot

# shellcheck disable=SC1091
source "$SCRIPT_DIR/deploy_auth.sh"
# Use a per-run socket by default. A fixed ControlPath can refer to a dead
# master from a previous deploy and produce `mux_client... Broken pipe`.
hb_ssh_setup 300
COMMON_EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/' --exclude 'logs/')
# Training checkpoints are never loaded on the robot (runtime uses ONNX only).
TRAINING_EXCLUDES=(--exclude '*.pt' --exclude '*.pth')
PRESETS_DIR="$HB_ROOT/voice_presets"
teleop_ok() { hb_ok "$*"; }
teleop_fail() { hb_fail "$*"; }

# Open the actual SSH transport before any long local/remote work.  Commands
# that can reach sudo also validate it here, then use sudo -n below so a
# deploy never pauses for a password after files or a build were started.
hb_deploy_auth_open_ssh "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT"
case "$COMMAND" in
    diff|status|pull|logs-*) ;;
    *) hb_deploy_auth_enable_sudo "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT" ;;
esac
trap 'hb_deploy_auth_cleanup "$SSH_CONTROL_PATH" "$ROBOT"' EXIT

sync_high() {
    rsync "${RSYNC_BASE[@]}" "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'build/' --exclude 'build_*/' --exclude 'thirdparty/onnxruntime/' --exclude 'docs/' \
        "${TRAINING_EXCLUDES[@]}" \
        "$HB_ROOT/controller/" "$ROBOT:$DEST/controller/"
}

# Lệnh `./hb` và bí danh `make` ở gốc repo, để robot dùng cùng lệnh với máy dev.
sync_root() {
    rsync "${RSYNC_BASE[@]}" "$@" "$HB_ROOT/hb" "$HB_ROOT/Makefile" "$ROBOT:$DEST/"
}

sync_voice() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
        --exclude 'unitree_bridge/build/' \
        "$HB_ROOT/voice/" "$ROBOT:$DEST/voice/"
}

sync_integration() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" --exclude 'build/' \
        "$HB_ROOT/integration/" "$ROBOT:$DEST/integration/"
}

sync_presets() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" \
        "$PRESETS_DIR/" "$ROBOT:$DEST/voice_presets/"
}

# Mirror CHỈ policies/dance/ với --delete: khi bạn xóa policy/npz cũ bên dev thì
# robot cũng xóa theo, để mỗi folder dance chỉ còn đúng 1 .onnx + 1 .npz và
# ScanDanceFolder không vớ nhầm file cũ (khỏi phải lên NoMachine xóa tay).
# Chừa 'backup/' để KHÔNG bao giờ lỡ xóa bản lưu trên robot.
sync_dance() {
    rsync "${RSYNC_BASE[@]}" --delete "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'backup/' "${TRAINING_EXCLUDES[@]}" \
        "$HB_ROOT/controller/policies/dance/" "$ROBOT:$DEST/controller/policies/dance/"
}

# Copy the robot-local WebXR/vendor IK runtime. Real env files live in /etc/hb
# and are never synced; local results and robot logs stay out of the sync.
sync_teleop() {
    rsync "${RSYNC_BASE[@]}" "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude '*.env' \
        --exclude 'results/' --exclude 'logs/' --exclude '__pycache__/' \
        --exclude '.pytest_cache/' --exclude '*.pyc' \
        "$HB_ROOT/teleop/" "$ROBOT:$DEST/teleop/"
}

load_robot_teleop_settings() {
    local settings
    settings="$(python3 "$HB_ROOT/teleop/scripts/load_teleop_config.py" \
        --config "$HB_ROOT/teleop/config/teleop.yaml")" || return 1
    ROBOT_TELEOP_ENABLED="$(awk -F= '$1 == "HB_TELEOP_RUNTIME_ENABLED" {print $2}' <<<"$settings")"
    ROBOT_TELEOP_PORT="$(awk -F= '$1 == "HB_TELEOP_RUNTIME_PORT" {print $2}' <<<"$settings")"
    [[ "$ROBOT_TELEOP_ENABLED" == "0" || "$ROBOT_TELEOP_ENABLED" == "1" ]] || {
        teleop_fail "robot_runtime.enabled invalid in teleop.yaml"
        return 2
    }
    [[ "$ROBOT_TELEOP_PORT" =~ ^[0-9]+$ ]] &&
        (( ROBOT_TELEOP_PORT >= 1024 && ROBOT_TELEOP_PORT <= 65535 )) || {
        teleop_fail "robot_runtime.port invalid in teleop.yaml"
        return 2
    }
}

# mDNS name and certificate SANs must match what the Quest will open.
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
        teleop_fail "Quest endpoint failed the mDNS/certificate check on the robot."
        return 1
    fi
    teleop_ok "Quest endpoint confirmed on robot: $report"
}

activate_robot_teleop_runtime() {
    local runtime_status listening local_sha remote_sha ready=0
    local runtime_settings runtime_port runtime_camera_enabled sidecar_process sidecar_ready
    local tracking_health camera_health invocation runtime_report camera_errors quest_hostname
    local camera_deploy_error=0 RED="$HB_STATUS_RED" RESET="$HB_STATUS_RESET"
    local_sha="$(sha256sum "$HB_ROOT/teleop/scripts/run_robot_teleop.sh" | awk '{print $1}')"
    remote_sha="$("${SSH[@]}" "$ROBOT" \
        "sha256sum '$DEST/teleop/scripts/run_robot_teleop.sh'" 2>/dev/null | awk '{print $1}')"
    if [[ -z "$remote_sha" || "$local_sha" != "$remote_sha" ]]; then
        teleop_fail "robot-local launcher checksum differs from local source"
        return 1
    fi
    case "$ROBOT_TELEOP_ENABLED" in
        0)
            "${SSH[@]}" -t "$ROBOT" \
                "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start"
            teleop_ok "robot_runtime.enabled=false; WebXR/vendor IK stopped and disabled."
            return 0
            ;;
        1) ;;
    esac

    "${SSH[@]}" "$ROBOT" "HB_ROOT='$DEST' bash '$DEST/teleop/scripts/bootstrap_robot_runtime.sh'"
    "${SSH[@]}" -t "$ROBOT" \
        "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start"
    runtime_settings="$("${SSH[@]}" "$ROBOT" 'source /etc/hb/teleop/runtime.env; printf "%s %s\n" "$HB_TELEOP_RUNTIME_PORT" "$HB_TELEOP_RUNTIME_CAMERA_ENABLED"')"
    read -r runtime_port runtime_camera_enabled <<<"$runtime_settings"
    if [[ ! "$runtime_port" =~ ^[0-9]+$ || ! "$runtime_camera_enabled" =~ ^[01]$ ]]; then
        teleop_fail "cannot read port/camera_enabled from /etc/hb/teleop/runtime.env on the robot"
        return 1
    fi
    # Ready means: both units active, WebXR listening, the H4 sidecar has its
    # first rt/lowstate, the xr-frame-v3 tracking channel answers and, when
    # enabled, WebRTC camera signaling is up.
    for _ in {1..45}; do
        runtime_status="$("${SSH[@]}" "$ROBOT" \
            'systemctl is-active hb_teleop_cert.service hb_teleop_runtime.service' 2>/dev/null || true)"
        listening="$("${SSH[@]}" "$ROBOT" \
            "ss -H -ltn 'sport = :$runtime_port' 2>/dev/null" || true)"
        sidecar_process="$("${SSH[@]}" "$ROBOT" "pgrep -af '[t]eleop.hardware.high_level_sidecar.*--stream-contract h4' 2>/dev/null || true" 2>/dev/null || true)"
        sidecar_ready="$("${SSH[@]}" "$ROBOT" "latest=\$(find '$DEST/teleop/logs/robot_runtime' -name sidecar.log -printf '%T@ %p\\n' 2>/dev/null | sort -nr | head -1 | cut -d' ' -f2-); test -n \"\$latest\" && grep -F '[SIDECAR] initial rt/lowstate ready; H4 STOP preamble armed' \"\$latest\" 2>/dev/null" 2>/dev/null || true)"
        tracking_health="$("${SSH[@]}" "$ROBOT" "curl -fkSs --max-time 2 'https://127.0.0.1:$runtime_port/tracking/status'" 2>/dev/null || true)"
        if [[ "$runtime_camera_enabled" == "1" ]]; then
            camera_health="$("${SSH[@]}" "$ROBOT" "curl -fkSs --max-time 2 'https://127.0.0.1:$runtime_port/camera/health'" 2>/dev/null || true)"
        else
            camera_health='{"transport": "webrtc"}'
        fi
        if [[ "$(grep -c '^active$' <<<"$runtime_status")" -eq 2 && -n "$listening" &&
              -n "$sidecar_process" && -n "$sidecar_ready" &&
              "$camera_health" == *'"transport": "webrtc"'* &&
              "$tracking_health" == *'"tracking_protocol": "xr-frame-v3"'* ]]; then
            ready=1
            break
        fi
        sleep 1
    done
    if (( ! ready )); then
        teleop_fail "teleop runtime not ready (service/WebXR/H4 sidecar/rt-lowstate, port $runtime_port)"
        printf '%s\n' "$runtime_status" >&2
        printf '[H4 sidecar process]\n%s\n' "$sidecar_process" >&2
        printf '[H4 initial rt/lowstate]\n%s\n' "${sidecar_ready:-not ready}" >&2
        "${SSH[@]}" "$ROBOT" \
            'journalctl -u hb_teleop_runtime.service -n 40 --no-pager -o cat' >&2 || true
        return 1
    fi
    teleop_ok "WebXR/vendor IK and H4 sidecar active; port $runtime_port listening."
    teleop_ok "xr-frame-v3 tracking ready; confirm real XR samples after ENTER VR on the Quest."
    verify_remote_quest_endpoint
    # Read only this invocation: old warnings must not describe a new run.
    invocation="$("${SSH[@]}" "$ROBOT" 'systemctl show hb_teleop_runtime.service -p InvocationID --value')"
    runtime_report="$("${SSH[@]}" "$ROBOT" "journalctl _SYSTEMD_INVOCATION_ID=$invocation --no-pager -o cat 2>/dev/null" || true)"
    grep -E '^\[RUNTIME\] (Quest URL|Target limit mode):' <<<"$runtime_report" | tail -2 || true
    if grep -qE '^\[RUNTIME PREFLIGHT\] (DEGRADED|WARN)' <<<"$runtime_report"; then
        camera_errors="$(grep -E '^\[RUNTIME PREFLIGHT\] (DEGRADED|WARN)' <<<"$runtime_report")"
        camera_deploy_error=1
        teleop_fail "DEPLOY DEGRADED: robot camera not ready; button A may not show it. Arm/head teleop still works in passthrough."
        printf '%s%s%s\n' "$RED" "$camera_errors" "$RESET" >&2
    elif [[ "$runtime_camera_enabled" == "1" ]]; then
        if grep -q '^\[RUNTIME PREFLIGHT\] OK R1 videohub camera read' <<<"$runtime_report"; then
            teleop_ok "Source camera read and decoded; still check the WebRTC image on the Quest."
        else
            camera_deploy_error=1
            teleop_fail "source camera not confirmed in this run; check the journal (signaling alone is not a working camera)."
        fi
    fi
    quest_hostname="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
    echo "     Quest bookmark: https://$quest_hostname:$runtime_port/?ws=wss://$quest_hostname:$runtime_port"
    # Nonzero reports an incomplete camera deployment while teleop keeps running.
    (( camera_deploy_error == 0 )) || return 2
}

case "$COMMAND" in
    diff)
        sync_root --dry-run
        sync_high --dry-run
        sync_voice --dry-run
        sync_integration --dry-run
        sync_presets --dry-run
        sync_dance --dry-run
        sync_teleop --dry-run
        ;;
    deploy)
        if [[ -n "$OPTION" && "$OPTION" != "--no-restart" && \
              "$OPTION" != "--restart-voice" && "$OPTION" != "--accept-policy" ]]; then
            echo "Usage: $0 deploy [--no-restart|--restart-voice|--accept-policy]" >&2
            exit 2
        fi
        if [[ -z "$OPTION" || "$OPTION" == "--accept-policy" ]]; then
            load_robot_teleop_settings
        fi
        if [[ "$OPTION" == "--accept-policy" ]]; then
            bash "$SCRIPT_DIR/update_model_manifest.sh" --accept
            OPTION=""
        fi
        hb_info "Deploying to $ROBOT"
        "${SSH[@]}" "$ROBOT" "mkdir -p '$DEST/controller' '$DEST/voice' '$DEST/integration' '$DEST/voice_presets' '$DEST/teleop' '$DEST/../HB_backups'; tar -czf '$DEST/../HB_backups/HB_$(date +%Y%m%d_%H%M%S).tar.gz' -C '$DEST' --exclude='*/build' --exclude='*/.venv' --exclude='*/logs' --exclude='*/results' --exclude='*/.env' --exclude='*/.env.*' controller voice integration voice_presets teleop 2>/dev/null || true"
        sync_root
        sync_high
        sync_voice
        sync_integration
        sync_presets
        sync_dance
        sync_teleop
        hb_ok "files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh'"
        if [[ "$OPTION" == "--restart-voice" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo -n HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart && sudo -n systemctl restart hb_integration.service hb_voice.service hb_voice_presets.service && bash '$DEST/integration/scripts/health_check.sh' --wait-voice --quiet"
        elif [[ "$OPTION" == "--no-restart" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo -n HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart"
        else
            "${SSH[@]}" -t "$ROBOT" "sudo -n HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'"
            activate_robot_teleop_runtime
        fi
        hb_ok "stack deploy complete"
        ;;
    deploy-high)
        # C++ controller: phai build lai tren robot va restart service (guard DISARMED
        # nam trong activate_services.sh, khong bo qua duoc).
        sync_high
        hb_ok "high-level files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' high"
        if [[ "$OPTION" == "--no-restart" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo -n HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart"
        else
            "${SSH[@]}" -t "$ROBOT" "sudo -n HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'"
        fi
        hb_ok "high-level deploy complete"
        ;;
    deploy-voice)
        # Python + r1_bridge (C++). Do not pull hb_high_level through the
        # Wants= dependency when it is intentionally stopped or manually owned.
        sync_voice
        hb_ok "voice files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' voice"
        "${SSH[@]}" -t "$ROBOT" "sudo -n systemctl --job-mode=ignore-dependencies restart hb_voice.service && bash '$DEST/integration/scripts/health_check.sh' --wait-voice --quiet"
        hb_ok "voice deploy complete"
        ;;
    deploy-integration)
        # hb_integration (C++): cong voice/PTT. Khong dung high-level.
        sync_integration
        hb_ok "integration files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' integration"
        "${SSH[@]}" -t "$ROBOT" "sudo -n systemctl restart hb_integration.service && bash '$DEST/integration/scripts/health_check.sh' --quiet"
        hb_ok "integration deploy complete"
        ;;
    deploy-presets)
        # Offline preset service: sync the isolated package plus its read-only
        # coordinator protocol. No high-level source/binary/restart is involved.
        sync_presets
        sync_integration
        hb_ok "preset and integration files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' presets"
        "${SSH[@]}" -t "$ROBOT" "sudo -n HB_ROOT='$DEST' bash '$DEST/integration/scripts/install_services.sh' && sudo -n systemctl restart hb_integration.service hb_voice_presets.service && bash '$DEST/integration/scripts/health_check.sh' --presets --quiet"
        hb_ok "preset deploy complete"
        ;;
    deploy-robot-teleop)
        # Sync and activate the robot-owned WebXR/vendor IK route.
        load_robot_teleop_settings
        sync_teleop
        activate_robot_teleop_runtime
        ;;
    status)
        "${SSH[@]}" "$ROBOT" "bash '$DEST/integration/scripts/stack_ctl.sh' status"
        ;;
    pull)
        DRY=()
        [[ "$OPTION" == "--dry-run" ]] && DRY=(--dry-run)
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
            --exclude 'build/' --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
            "$ROBOT:$DEST/controller/" "$HB_ROOT/controller/"
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
            --exclude 'build/' --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
            "$ROBOT:$DEST/voice/" "$HB_ROOT/voice/"
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" --exclude 'build/' \
            "$ROBOT:$DEST/integration/" "$HB_ROOT/integration/"
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
            "$ROBOT:$DEST/voice_presets/" "$PRESETS_DIR/"
        ;;
    rollback)
        "${SSH[@]}" -t "$ROBOT" "set -e; latest=\$(ls -1t '$DEST/../HB_backups'/HB_*.tar.gz 2>/dev/null | head -1); test -n \"\$latest\"; tar -xzf \"\$latest\" -C '$DEST'; HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh'; sudo -n HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'; if [ -f '$DEST/teleop/scripts/install_robot_runtime.sh' ]; then sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start; fi"
        ;;
    start-*|stop-*|restart-*|logs-*)
        # Guard DISARMED và danh sách lệnh hợp lệ nằm trong stack_ctl.sh, dùng
        # chung với `./hb` chạy trực tiếp trên robot.
        "${SSH[@]}" -t "$ROBOT" "bash '$DEST/integration/scripts/stack_ctl.sh' $COMMAND"
        ;;
    *)
        cat >&2 <<'USAGE'
Usage (thường gọi qua ./hb ở gốc repo):
  deploy [--accept-policy|--no-restart|--restart-voice]   toàn bộ stack
  deploy-high [--no-restart] | deploy-voice | deploy-integration | deploy-presets
  deploy-robot-teleop                  sync teleop + restart runtime Quest (không cần DISARMED)
  diff | status | pull [--dry-run] | rollback
  {start|stop|restart}-{all|high|integration|voice|presets|teleop}
  logs-{all|high|integration|voice|presets|teleop}
USAGE
        exit 2
        ;;
esac
