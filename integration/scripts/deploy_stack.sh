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

# Dùng chung cơ chế dò robot với các script controller. Có thể đặt sẵn
# ROBOT=unitree@<ip-or-hostname> để bỏ qua bước dò.
# shellcheck disable=SC1091
source "$HB_ROOT/controller/scripts/_find_robot.sh"
find_robot

# Use a per-run socket by default. A fixed ControlPath can refer to a dead
# master from a previous deploy and produce `mux_client... Broken pipe`.
SSH_CONTROL_PATH="${HB_SSH_CONTROL_PATH:-/tmp/hb-ssh-${BASHPID}-%C}"
SSH_CONNECT_TIMEOUT="${HB_DEPLOY_CONNECT_TIMEOUT:-10}"
SSH_OPTS=(
    -o ControlMaster=auto
    -o ControlPersist=300
    -o "ControlPath=$SSH_CONTROL_PATH"
    -o ConnectionAttempts=1
    -o "ConnectTimeout=$SSH_CONNECT_TIMEOUT"
    -o ServerAliveInterval=5
    -o ServerAliveCountMax=2
)
SSH=(ssh "${SSH_OPTS[@]}")
RSYNC_SSH="ssh -o ControlMaster=auto -o ControlPersist=300 -o ControlPath=$SSH_CONTROL_PATH -o ConnectionAttempts=1 -o ConnectTimeout=$SSH_CONNECT_TIMEOUT -o ServerAliveInterval=5 -o ServerAliveCountMax=2"
RSYNC_BASE=(-az --no-perms --omit-dir-times -e "$RSYNC_SSH")
COMMON_EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/' --exclude 'logs/')
PRESETS_DIR="$HB_ROOT/voice_presets"
teleop_ok() { hb_ok "$*"; }
teleop_fail() { hb_fail "$*"; }

sync_high() {
    rsync "${RSYNC_BASE[@]}" "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'build/' --exclude 'thirdparty/onnxruntime/' --exclude 'docs/' \
        "$HB_ROOT/controller/" "$ROBOT:$DEST/controller/"
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
        --exclude 'backup/' \
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

activate_robot_teleop_runtime() {
    local runtime_status runtime_info listening local_sha remote_sha ready=0
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
                "sudo HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start"
            teleop_ok "robot_runtime.enabled=false; WebXR/vendor IK stopped and disabled."
            return 0
            ;;
        1) ;;
    esac

    "${SSH[@]}" "$ROBOT" "HB_ROOT='$DEST' bash '$DEST/teleop/scripts/bootstrap_robot_runtime.sh'"
    "${SSH[@]}" -t "$ROBOT" \
        "sudo HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start"
    for _ in {1..45}; do
        runtime_status="$("${SSH[@]}" "$ROBOT" \
            'systemctl is-active hb_teleop_cert.service hb_teleop_runtime.service' 2>/dev/null || true)"
        listening="$("${SSH[@]}" "$ROBOT" \
            "ss -H -ltn 'sport = :$ROBOT_TELEOP_PORT' 2>/dev/null" || true)"
        if [[ "$(grep -c '^active$' <<<"$runtime_status")" -eq 2 && -n "$listening" ]]; then
            ready=1
            break
        fi
        sleep 1
    done
    if (( ! ready )); then
        teleop_fail "WebXR/vendor IK not ready: service/port $ROBOT_TELEOP_PORT"
        "${SSH[@]}" "$ROBOT" \
            'journalctl -u hb_teleop_runtime.service -n 12 --no-pager -o cat' >&2 || true
        return 1
    fi
    teleop_ok "WebXR/vendor IK active; port $ROBOT_TELEOP_PORT listening."
    runtime_info="$("${SSH[@]}" "$ROBOT" \
        'journalctl -u hb_teleop_runtime.service -b -n 200 --no-pager -o cat 2>/dev/null | grep -E "^\[RUNTIME\] (Quest URL|Target limit mode):" | tail -2' || true)"
    [[ -z "$runtime_info" ]] || printf '%s\n' "$runtime_info"
}

case "$COMMAND" in
    diff)
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
        sync_high
        sync_voice
        sync_integration
        sync_presets
        sync_dance
        sync_teleop
        hb_ok "files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh'"
        if [[ "$OPTION" == "--restart-voice" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart && sudo systemctl restart hb_integration.service hb_voice.service hb_voice_presets.service && bash '$DEST/integration/scripts/health_check.sh' --wait-voice --quiet"
        elif [[ "$OPTION" == "--no-restart" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart"
        else
            "${SSH[@]}" -t "$ROBOT" "sudo HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'"
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
            "${SSH[@]}" -t "$ROBOT" "sudo HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh' --no-restart"
        else
            "${SSH[@]}" -t "$ROBOT" "sudo HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'"
        fi
        hb_ok "high-level deploy complete"
        ;;
    deploy-voice)
        # Python + r1_bridge (C++). Do not pull hb_high_level through the
        # Wants= dependency when it is intentionally stopped or manually owned.
        sync_voice
        hb_ok "voice files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' voice"
        "${SSH[@]}" -t "$ROBOT" "sudo systemctl --job-mode=ignore-dependencies restart hb_voice.service && bash '$DEST/integration/scripts/health_check.sh' --wait-voice --quiet"
        hb_ok "voice deploy complete"
        ;;
    deploy-integration)
        # hb_integration (C++): cong voice/PTT. Khong dung high-level.
        sync_integration
        hb_ok "integration files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' integration"
        "${SSH[@]}" -t "$ROBOT" "sudo systemctl restart hb_integration.service && bash '$DEST/integration/scripts/health_check.sh' --quiet"
        hb_ok "integration deploy complete"
        ;;
    deploy-presets)
        # Offline preset service: sync the isolated package plus its read-only
        # coordinator protocol. No high-level source/binary/restart is involved.
        sync_presets
        sync_integration
        hb_ok "preset and integration files synchronized"
        "${SSH[@]}" "$ROBOT" "HB_COLOR='$HB_COLOR' HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh' presets"
        "${SSH[@]}" -t "$ROBOT" "sudo HB_ROOT='$DEST' bash '$DEST/integration/scripts/install_services.sh' && sudo systemctl restart hb_integration.service hb_voice_presets.service && bash '$DEST/integration/scripts/health_check.sh' --presets --quiet"
        hb_ok "preset deploy complete"
        ;;
    deploy-teleop)
        # Sync only. Robot-local service keeps its old code until restarted.
        sync_teleop
        echo "[teleop] synced; robot-local runtime was not restarted."
        ;;
    deploy-robot-teleop)
        # Sync and activate the robot-owned WebXR/vendor IK route.
        load_robot_teleop_settings
        sync_teleop
        activate_robot_teleop_runtime
        ;;
    deploy-dance)
        sync_dance
        hb_ok "dance assets synchronized; restart high-level to load them"
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
        "${SSH[@]}" -t "$ROBOT" "set -e; latest=\$(ls -1t '$DEST/../HB_backups'/HB_*.tar.gz 2>/dev/null | head -1); test -n \"\$latest\"; tar -xzf \"\$latest\" -C '$DEST'; HB_ROOT='$DEST' bash '$DEST/integration/scripts/build_on_robot.sh'; sudo HB_ROOT='$DEST' bash '$DEST/integration/scripts/activate_services.sh'; if [ -f '$DEST/teleop/scripts/install_robot_runtime.sh' ]; then sudo HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start; fi"
        ;;
    stop-all|stop-high|stop-integration|stop-voice|stop-presets|restart-all|\
    start-high|start-integration|start-voice|start-presets|start-all)
        # Guard DISARMED nằm trong stack_ctl.sh, dùng chung với `make` trên robot.
        "${SSH[@]}" -t "$ROBOT" "bash '$DEST/integration/scripts/stack_ctl.sh' $COMMAND"
        ;;
    *)
        cat >&2 <<'USAGE'
Usage:
  TOAN BO
    deploy [--accept-policy]      backup + sync + build + activate, including robot teleop
    deploy --no-restart|--restart-voice  sync + build; do not restart robot teleop
    diff                                                    xem truoc, khong ghi gi len robot
  TUNG PHAN
    deploy-high [--no-restart]   controller (C++)  -> build + restart (can DISARMED)
    deploy-voice                 voice            -> build + restart hb_voice
    deploy-integration           integration      -> build + restart hb_integration
    deploy-presets               offline voice presets + coordinator -> no high-level restart
    deploy-teleop                teleop/              -> chi sync, khong build/restart
    deploy-robot-teleop          teleop + ARM64 vendor IK/WebXR service -> sync + restart robot
    deploy-dance                 policies/dance      -> chi sync
  KHAC
    status | pull [--dry-run] | rollback
    stop-all|stop-high|stop-integration|stop-voice|stop-presets
    restart-all|start-high|start-integration|start-voice|start-presets|start-all
USAGE
        exit 2
        ;;
esac
