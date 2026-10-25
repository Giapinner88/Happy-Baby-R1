#!/usr/bin/env bash
# Deploy máy trạm -> robot. `./hb deploy|diff|rollback|status|...` gọi script này.
#
#   deploy [auto|controller|voice|teleop|all] [--no-restart] [--dry-run]
#
# auto (mặc định) so máy trạm với robot bằng rsync --dry-run rồi chỉ làm phần đổi,
# đúng thứ tự. Thành phần:
#   controller  controller/ (C++, mọi config, policy, dance) -> restart hb_high_level,
#               CHỈ khi DISARMED. Robot ARMED: không ghi gì của controller.
#   voice       voice/ + voice_presets/ + integration (service PTT) -> không đụng high-level.
#   teleop      teleop/ -> runtime Quest.
#   công cụ     integration/{scripts,systemd,config} + ./hb: chép, không restart gì.
# Màn hình: mỗi bước một dòng ✓/✗/⚠; log đầy đủ ở $HB_DEPLOY_LOG_DIR.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HIGH_DIR="$HB_ROOT/controller"
if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then HB_COLOR=1; else HB_COLOR="${HB_COLOR:-0}"; fi
source "$SCRIPT_DIR/status_log.sh"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/config_reader.sh"
ROBOT="${ROBOT:-}"
DEST="${DEST:-/home/unitree/HB}"
COMMAND="${1:-diff}"
[[ $# -gt 0 ]] && shift

usage() {
    cat >&2 <<'USAGE'
Usage (thường gọi qua ./hb ở gốc repo):
  deploy [auto|controller|voice|teleop|all] [--no-restart] [--dry-run]
  diff | status | pull [--dry-run] | rollback
  {start|stop|restart}-{all|high|integration|voice|presets|teleop}
  logs-{all|high|integration|voice|presets|teleop}
USAGE
    exit 2
}

TARGET=auto
NO_RESTART=0
DRY_RUN=0
OPTION=""
case "$COMMAND" in
    deploy|diff)
        [[ "$COMMAND" == diff ]] && DRY_RUN=1
        for arg in "$@"; do
            case "$arg" in
                auto|all|controller|voice|teleop) TARGET="$arg" ;;
                --no-restart) NO_RESTART=1 ;;
                --dry-run) DRY_RUN=1 ;;
                *) usage ;;
            esac
        done
        COMMAND=deploy
        ;;
    pull) OPTION="${1:-}" ;;
    status|rollback|start-*|stop-*|restart-*|logs-*) ;;
    *) usage ;;
esac

# Dò robot (thư viện dùng chung của integration). Có thể đặt sẵn
# ROBOT=unitree@<ip-or-hostname> để bỏ qua bước dò.
# shellcheck disable=SC1091
source "$SCRIPT_DIR/find_robot.sh"
find_robot

# shellcheck disable=SC1091
source "$SCRIPT_DIR/deploy_auth.sh"
# Use a per-run socket by default. A fixed ControlPath can refer to a dead
# master from a previous deploy and produce `mux_client... Broken pipe`.
hb_ssh_setup 900
COMMON_EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/' --exclude 'logs/')
# Training checkpoints are never loaded on the robot (runtime uses ONNX only).
TRAINING_EXCLUDES=(--exclude '*.pt' --exclude '*.pth')
PRESETS_DIR="$HB_ROOT/voice_presets"
teleop_ok() { hb_ok "teleop: $*"; }
teleop_fail() { hb_fail "teleop: $*"; }

# Open the actual SSH transport before any long local/remote work. Deploy enables
# sudo only after the plan shows there is something to write, then uses sudo -n
# so it never pauses for a password after files or a build were started.
hb_deploy_auth_open_ssh "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT"
case "$COMMAND" in
    deploy|status|pull|logs-*) ;;
    *) hb_deploy_auth_enable_sudo "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT" ;;
esac
TMP_DIR="$(mktemp -d /tmp/hb-deploy.XXXXXX)"
cleanup() {
    rm -rf -- "$TMP_DIR"
    hb_deploy_auth_cleanup "$SSH_CONTROL_PATH" "$ROBOT"
}
trap cleanup EXIT

# ------------------------------------------------------------------ sync ----

sync_high() {
    rsync "${RSYNC_BASE[@]}" "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'build/' --exclude 'build_*/' --exclude 'thirdparty/onnxruntime/' --exclude 'docs/' \
        --exclude '/policies/dance/' "${TRAINING_EXCLUDES[@]}" \
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

# Công cụ integration: scripts, unit systemd, config. model_manifest.conf thuộc
# controller (chỉ deploy controller ghi, sau preflight).
sync_tools() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude '/config/model_manifest.conf' \
        --include '/scripts/***' --include '/systemd/***' --include '/config/***' --exclude '*' \
        "$HB_ROOT/integration/" "$ROBOT:$DEST/integration/"
}

# Service hb_integration (cổng PTT/audio): mọi thứ còn lại của integration/.
sync_integration_service() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" --exclude 'build/' \
        --exclude '/scripts/' --exclude '/systemd/' --exclude '/config/' \
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
        --exclude 'backup/' --exclude 'artifacts/' --exclude '*.onnx.*' "${TRAINING_EXCLUDES[@]}" \
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

# ---------------------------------------------------------------- teleop ----

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
    teleop_ok "Quest endpoint OK ($report)"
}

# Gọi dạng `activate_robot_teleop_runtime || rc=$?`: mọi bước hỏng phải tự return.
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
            "${SSH[@]}" "$ROBOT" \
                "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start" \
                >>"$LOG" 2>&1 || { teleop_fail "install_robot_runtime.sh failed (log)"; return 1; }
            teleop_ok "robot_runtime.enabled=false; WebXR/vendor IK stopped and disabled."
            return 0
            ;;
        1) ;;
    esac

    "${SSH[@]}" "$ROBOT" "HB_ROOT='$DEST' bash '$DEST/teleop/scripts/bootstrap_robot_runtime.sh'" \
        >>"$LOG" 2>&1 || { teleop_fail "bootstrap_robot_runtime.sh failed (log)"; return 1; }
    "${SSH[@]}" "$ROBOT" \
        "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start" \
        >>"$LOG" 2>&1 || { teleop_fail "install_robot_runtime.sh failed (log)"; return 1; }
    runtime_settings="$("${SSH[@]}" "$ROBOT" 'source /etc/hb/teleop/runtime.env; printf "%s %s\n" "$HB_TELEOP_RUNTIME_PORT" "$HB_TELEOP_RUNTIME_CAMERA_ENABLED"')" || true
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
        {
            printf '%s\n' "$runtime_status"
            printf '[H4 sidecar process]\n%s\n' "$sidecar_process"
            printf '[H4 initial rt/lowstate]\n%s\n' "${sidecar_ready:-not ready}"
            "${SSH[@]}" "$ROBOT" \
                'journalctl -u hb_teleop_runtime.service -n 40 --no-pager -o cat' || true
        } >>"$LOG" 2>&1
        return 1
    fi
    teleop_ok "WebXR/vendor IK + H4 sidecar + xr-frame-v3 tracking ready (port $runtime_port)"
    verify_remote_quest_endpoint || return 1
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

# -------------------------------------------------------- bước + báo cáo ----

LOG_DIR="${HB_DEPLOY_LOG_DIR:-${XDG_STATE_HOME:-$HOME/.local/state}/hb/deploy}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG=/dev/null
if [[ "$COMMAND" == deploy || "$COMMAND" == rollback ]]; then
    LOG="$LOG_DIR/$RUN_ID.log"
    mkdir -p "$LOG_DIR"
    : >"$LOG"
    # Giữ 30 log gần nhất.
    ls -1t "$LOG_DIR"/*.log 2>/dev/null | tail -n +31 | xargs -r rm -f --
fi
STEP_FILE="$TMP_DIR/step.out"
STEP_RC=0
STEP_TEXT=""
RUN_START=$SECONDS
IMPACT=()      # "red|..." robot không chạy được / chưa cập nhật; "yellow|..." gián đoạn, cần thao tác
FAILED=0

fmt_secs() {
    local s="$1"
    if (( s >= 60 )); then printf '%dm%02ds' $((s / 60)) $((s % 60)); else printf '%ds' "$s"; fi
}

impact() { IMPACT+=("$1|$2"); [[ "$1" != red ]] || FAILED=1; }

# step <nhãn> <lệnh...>: chạy lệnh trong subshell `set -e`, output đầy đủ vào $LOG,
# màn hình một dòng ✓/✗ + thời gian; lỗi thì in vài dòng liên quan. Kết quả ở
# STEP_RC/STEP_TEXT. LUÔN gọi như một câu lệnh thường, không đặt trong if/||/&&:
# bash bỏ qua set -e bên trong hàm được gọi trong điều kiện.
step() {
    local label="$1" start=$SECONDS line
    shift
    printf '\n=== %s ===\n' "$label" >>"$LOG"
    set +e
    ( set -e; "$@" ) >"$STEP_FILE" 2>&1
    STEP_RC=$?
    set -e
    cat "$STEP_FILE" >>"$LOG"
    STEP_TEXT="$(cat "$STEP_FILE")"
    line="$label ${HB_STATUS_DIM}$(fmt_secs $((SECONDS - start)))${HB_STATUS_RESET}"
    if (( STEP_RC == 0 )); then
        hb_ok "$line"
    else
        hb_fail "$line"
        { grep -Ei 'error|fail|✗|not found|no space|refused|denied|cannot|lỗi|thiếu' "$STEP_FILE" ||
            tail -n 6 "$STEP_FILE"; } | tail -n 8 | sed 's/^/    /' >&2
    fi
    return 0
}

text_get() { sed -n "s/^$1=//p" <<<"$2" | head -n 1; }

# robot <bước> [args]: chạy robot_steps.sh trên robot (đẩy file qua stdin).
robot() {
    "${SSH[@]}" "$ROBOT" "HB_DEST=$(printf '%q' "$DEST") bash -s -- $(printf '%q ' "$@")" \
        <"$SCRIPT_DIR/robot_steps.sh"
}

report() {
    local entry level msg
    printf '%s\n' "────────────────────────────────────────"
    if (( FAILED )); then
        printf '%s✗ DEPLOY CHƯA XONG%s  (%s)\n' "$HB_STATUS_RED$HB_STATUS_BOLD" "$HB_STATUS_RESET" \
            "$(fmt_secs $((SECONDS - RUN_START)))"
    else
        printf '%s✓ DEPLOY OK%s  (%s)\n' "$HB_STATUS_GREEN$HB_STATUS_BOLD" "$HB_STATUS_RESET" \
            "$(fmt_secs $((SECONDS - RUN_START)))"
    fi
    if (( ${#IMPACT[@]} )); then
        echo "Ảnh hưởng tới robot:"
        for entry in "${IMPACT[@]}"; do
            level="${entry%%|*}"
            msg="${entry#*|}"
            if [[ "$level" == red ]]; then
                printf '  %s✗%s %s\n' "$HB_STATUS_RED" "$HB_STATUS_RESET" "$msg"
            else
                printf '  %s⚠%s %s\n' "$HB_STATUS_YELLOW" "$HB_STATUS_RESET" "$msg"
            fi
        done
    else
        echo "Ảnh hưởng tới robot: không có (không restart service nào)."
    fi
    printf '%slog: %s%s\n' "$HB_STATUS_DIM" "$LOG" "$HB_STATUS_RESET"
}

# ------------------------------------------------------------ phát hiện ----

# changed_files <sync-fn>: đường dẫn sẽ đổi trên robot (rsync --dry-run).
# Dòng bắt đầu bằng "-" là file sẽ bị xoá.
changed_files() {
    "$@" --dry-run --out-format='%i %n' | awk '
        $1 == "*deleting" { sub(/^[^ ]+ +/, ""); print "-" $0; next }
        $1 ~ /^[<>c][fLDS]/ { sub(/^[^ ]+ +/, ""); print }'
}

# Bỏ tài liệu/test: đổi chúng không cần build hay restart.
runtime_only() { grep -Ev '^-?(.*/)?(tests?|docs)/|\.md$' || true; }
count_lines() { [[ -z "$1" ]] && echo 0 || wc -l <<<"$1" | tr -d ' '; }

# Tệp integration mà ExecStartPre của các service dùng (preflight.sh và thư viện).
HIGH_START_TOOLS=(scripts/preflight.sh scripts/config_reader.sh
                  scripts/dance_preflight_summary.sh scripts/status_log.sh)

detect_changes() {
    CTRL_FILES="$( { changed_files sync_high; changed_files sync_dance | sed -E 's#^(-?)#\1policies/dance/#'; } )"
    TOOLS_FILES="$( { changed_files sync_tools; changed_files sync_root | sed 's#^#(root)/#'; } )"
    VOICE_FILES="$(changed_files sync_voice)"
    PRESETS_FILES="$(changed_files sync_presets)"
    INTEG_FILES="$(changed_files sync_integration_service)"
    TELEOP_FILES="$(changed_files sync_teleop)"
    PROBE="$(robot probe)"
}

probe() { text_get "$1" "$PROBE"; }
wants() { [[ "$TARGET" == auto || "$TARGET" == all || "$TARGET" == "$1" ]]; }

plan() {
    local f local_manifest_sha
    DO_CONTROLLER=0; CTRL_REASON=""; HELD_BACK=0
    DO_TOOLS=0; TOOLS_VERIFY=0; TOOLS_HIGH_UNIT=0; INSTALL_UNITS=0
    DO_TELEOP=0; TELEOP_REASON=""
    BUILD_VOICE=0; BUILD_INTEG=0; SYNC_VOICE_PARTS=0
    VOICE_UNITS=()
    local -A unit_reason=()

    # Công cụ: phân loại theo hệ quả.
    if [[ -n "$TOOLS_FILES" ]]; then
        DO_TOOLS=1
        while IFS= read -r f; do
            f="${f#-}"
            for t in "${HIGH_START_TOOLS[@]}"; do [[ "$f" == "$t" ]] && TOOLS_VERIFY=1; done
            case "$f" in
                systemd/hb_high_level.service.in) TOOLS_HIGH_UNIT=1; INSTALL_UNITS=1 ;;
                systemd/hb_integration.service.in) INSTALL_UNITS=1; unit_reason[hb_integration]="unit systemd đổi" ;;
                systemd/hb_voice.service.in) INSTALL_UNITS=1; unit_reason[hb_voice]="unit systemd đổi" ;;
                systemd/hb_voice_presets.service.in) INSTALL_UNITS=1; unit_reason[hb_voice_presets]="unit systemd đổi" ;;
                systemd/*) INSTALL_UNITS=1 ;;
            esac
        done <<<"$TOOLS_FILES"
    fi

    # Controller.
    local_manifest_sha="$(sha256sum "$HB_ROOT/integration/config/model_manifest.conf" | awk '{print $1}')"
    if wants controller; then
        if [[ "$TARGET" == all || "$TARGET" == controller ]]; then CTRL_REASON="yêu cầu"; fi
        [[ -n "$(runtime_only <<<"$CTRL_FILES")" ]] && CTRL_REASON="${CTRL_REASON:-file đổi}"
        (( TOOLS_HIGH_UNIT )) && CTRL_REASON="${CTRL_REASON:-unit hb_high_level đổi}"
        [[ "$(probe pending_hb_high_level)" == 1 ]] && CTRL_REASON="${CTRL_REASON:-chép rồi nhưng chưa restart}"
        [[ "$(probe bin_run_r1)" == 1 ]] || CTRL_REASON="${CTRL_REASON:-robot thiếu run_r1}"
        [[ "$(probe manifest_sha)" == "$local_manifest_sha" ]] || CTRL_REASON="${CTRL_REASON:-manifest lệch}"
        [[ -n "$CTRL_REASON" ]] && DO_CONTROLLER=1
    fi
    if (( DO_CONTROLLER && ! NO_RESTART )) && [[ "$(probe high)" == armed ]]; then
        DO_CONTROLLER=0
        HELD_BACK=1
    fi

    # Voice: integration service, voice, presets.
    if wants voice; then
        if [[ "$TARGET" == all || "$TARGET" == voice ]]; then
            for f in hb_integration hb_voice hb_voice_presets; do unit_reason[$f]="${unit_reason[$f]:-yêu cầu}"; done
            BUILD_VOICE=1; BUILD_INTEG=1
        fi
        if [[ -n "$(runtime_only <<<"$INTEG_FILES" | grep -E '^-?(src/|CMakeLists\.txt$)')" ]]; then
            unit_reason[hb_integration]="${unit_reason[hb_integration]:-file đổi}"; BUILD_INTEG=1
        fi
        if [[ "$(probe bin_integration)" != 1 ]]; then
            unit_reason[hb_integration]="${unit_reason[hb_integration]:-robot thiếu hb_integration}"; BUILD_INTEG=1
        fi
        if [[ -n "$(runtime_only <<<"$VOICE_FILES")" ]]; then
            unit_reason[hb_voice]="${unit_reason[hb_voice]:-file đổi}"; BUILD_VOICE=1
            # Presets dùng chung venv và r1_bridge của voice.
            if grep -qE '^-?(unitree_bridge/|pyproject\.toml$|uv\.lock$)' <<<"$VOICE_FILES"; then
                unit_reason[hb_voice_presets]="${unit_reason[hb_voice_presets]:-venv/bridge voice đổi}"
            fi
        fi
        if [[ "$(probe bin_r1_bridge)" != 1 || "$(probe bin_venv)" != 1 ]]; then
            unit_reason[hb_voice]="${unit_reason[hb_voice]:-robot thiếu r1_bridge/venv}"
            unit_reason[hb_voice_presets]="${unit_reason[hb_voice_presets]:-robot thiếu r1_bridge/venv}"
            BUILD_VOICE=1
        fi
        [[ -n "$(runtime_only <<<"$PRESETS_FILES")" ]] && \
            unit_reason[hb_voice_presets]="${unit_reason[hb_voice_presets]:-file đổi}"
        for f in hb_integration hb_voice hb_voice_presets; do
            [[ "$(probe "pending_$f")" == 1 ]] && unit_reason[$f]="${unit_reason[$f]:-chép rồi nhưng chưa restart}"
        done
    fi
    # Unit voice chỉ restart khi lệnh có phạm vi voice; ngoài phạm vi (vd unit
    # systemd đổi trong `deploy controller`) thì đặt marker để lần sau restart.
    PENDING_ONLY=()
    for f in hb_integration hb_voice hb_voice_presets; do
        [[ -n "${unit_reason[$f]:-}" ]] || continue
        if wants voice; then VOICE_UNITS+=("$f"); else PENDING_ONLY+=("$f"); fi
    done
    if (( ${#VOICE_UNITS[@]} )); then SYNC_VOICE_PARTS=1; fi
    VOICE_REASON=""
    for f in "${VOICE_UNITS[@]}"; do VOICE_REASON+="${f#hb_}: ${unit_reason[$f]}; "; done

    # Teleop.
    if wants teleop; then
        [[ "$TARGET" == all || "$TARGET" == teleop ]] && TELEOP_REASON="yêu cầu"
        [[ -n "$(runtime_only <<<"$TELEOP_FILES")" ]] && TELEOP_REASON="${TELEOP_REASON:-file đổi}"
        [[ "$(probe pending_hb_teleop_runtime)" == 1 ]] && TELEOP_REASON="${TELEOP_REASON:-chép rồi nhưng chưa restart}"
        [[ -n "$TELEOP_REASON" ]] && DO_TELEOP=1
    fi

    [[ "$TARGET" == all ]] && INSTALL_UNITS=1
    # Robot ARMED: không chép phần làm thay đổi lần khởi động kế của hb_high_level.
    TOOLS_HOLD=()
    if (( ! DO_CONTROLLER )); then
        TOOLS_HOLD=(--exclude '/systemd/hb_high_level.service.in')
    fi
    return 0
}

# Dòng tóm tắt một thành phần: ● số file (vài đường dẫn) hoặc · không đổi.
plan_line() {
    local name="$1" active="$2" files="$3" reason="$4" limit="$5" scope="${6:-}" n shown
    n="$(count_lines "$files")"
    # Căn theo số ký tự (printf %-Ns đếm byte, lệch với tiếng Việt).
    name="$name$(printf '%*s' $((11 - ${#name})) '')"
    if (( active )); then
        shown="$(head -n "$limit" <<<"$files" | paste -sd, - | sed 's/,/, /g')"
        if (( n > limit )); then shown+=", +$((n - limit))"; fi
        if (( n > 0 )); then shown="$n file${shown:+: $shown}  "; fi
        printf '  %s●%s %s %s%s(%s)%s\n' "$HB_STATUS_BOLD" "$HB_STATUS_RESET" "$name" "$shown" \
            "$HB_STATUS_DIM" "$reason" "$HB_STATUS_RESET"
    elif (( n > 0 )) && [[ -n "$scope" ]] && ! wants "$scope"; then
        printf '  %s· %s %s file đổi, ngoài phạm vi lần này%s\n' "$HB_STATUS_DIM" "$name" "$n" "$HB_STATUS_RESET"
    elif (( n > 0 )); then
        printf '  %s· %s %s file tài liệu/test, bỏ qua%s\n' "$HB_STATUS_DIM" "$name" "$n" "$HB_STATUS_RESET"
    else
        printf '  %s· %s không đổi%s\n' "$HB_STATUS_DIM" "$name" "$HB_STATUS_RESET"
    fi
}

print_plan() {
    local limit=3
    (( DRY_RUN )) && limit=15
    printf '%sHB deploy %s%s → %s%s\n' "$HB_STATUS_BOLD" "$TARGET" \
        "$( ((NO_RESTART)) && echo ' --no-restart')" "$ROBOT" "$HB_STATUS_RESET"
    if (( HELD_BACK )); then
        printf '  %s✗ controller  có thay đổi nhưng robot đang ARMED (%s): KHÔNG chép%s\n' \
            "$HB_STATUS_RED" "$(probe high_state)" "$HB_STATUS_RESET"
    else
        plan_line controller "$DO_CONTROLLER" "$CTRL_FILES" "$CTRL_REASON" "$limit" controller
    fi
    plan_line voice "$SYNC_VOICE_PARTS" "$(printf '%s\n%s\n%s' "$VOICE_FILES" "$PRESETS_FILES" "$INTEG_FILES" | sed '/^$/d')" \
        "${VOICE_REASON%; }" "$limit" voice
    plan_line teleop "$DO_TELEOP" "$TELEOP_FILES" "$TELEOP_REASON" "$limit" teleop
    plan_line "công cụ" "$DO_TOOLS" "$TOOLS_FILES" "chép, không restart" "$limit"
}

# ------------------------------------------------------------ controller ----

ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime/lib"
case "$(uname -m)" in aarch64|arm64) ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime_aarch64/lib" ;; esac
PREFLIGHT_LOG="$TMP_DIR/local-preflight.log"
MANIFEST_TMP="$TMP_DIR/model_manifest.conf"

# Build run_r1 tăng dần trên máy trạm + preflight (không xuất lệnh motor), rồi sinh
# manifest từ chính runtime. Không có gì được ghi lên robot nếu bước này hỏng.
local_controller_check() {
    local selected selected_rel selected_contract teleop_sel teleop_rel_cfg line
    local model_rel contract input output teleop_rel sha
    [[ -f "$HIGH_DIR/build/CMakeCache.txt" ]] || \
        cmake -S "$HIGH_DIR" -B "$HIGH_DIR/build" -DCMAKE_BUILD_TYPE=Release
    cmake --build "$HIGH_DIR/build" --target run_r1 --parallel "${HB_BUILD_JOBS:-$(nproc)}"
    selected="$(hb_selected_policy "$HIGH_DIR/config/tuning.yaml" walk_policy)"
    read -r selected_rel selected_contract <<<"$selected"
    teleop_sel="$(hb_selected_policy "$HIGH_DIR/config/tuning.yaml" teleop_policy)"
    read -r teleop_rel_cfg _ <<<"$teleop_sel"
    if ! LD_LIBRARY_PATH="$ORT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" HB_PROJECT_DIR="$HIGH_DIR" \
        "$HIGH_DIR/build/run_r1" --preflight >"$PREFLIGHT_LOG" 2>&1; then
        tail -n 15 "$PREFLIGHT_LOG"
        echo "run_r1 --preflight lỗi trên máy trạm"
        return 1
    fi
    line="$(grep -m1 '^\[Preflight\] MODEL ' "$PREFLIGHT_LOG" || true)"
    [[ -n "$line" ]] || { echo "preflight không in [Preflight] MODEL"; return 1; }
    model_rel="$(sed -n 's/.* rel=\([^ ]*\).*/\1/p' <<<"$line")"
    contract="$(sed -n 's/.* contract=\([^ ]*\).*/\1/p' <<<"$line")"
    input="$(sed -n 's/.* input=\([^ ]*\).*/\1/p' <<<"$line")"
    output="$(sed -n 's/.* output=\([^ ]*\).*/\1/p' <<<"$line")"
    [[ "$contract" == "$selected_contract" ]] || { echo "runtime contract=$contract nhưng config chọn $selected_contract"; return 1; }
    [[ "$model_rel" == "$selected_rel" ]] || { echo "runtime nạp '$model_rel', config chọn '$selected_rel'"; return 1; }
    line="$(grep -m1 '^\[Preflight\] TELEOP_MODEL ' "$PREFLIGHT_LOG" || true)"
    teleop_rel="$(sed -n 's/.* rel=\([^ ]*\).*/\1/p' <<<"$line")"
    [[ "$teleop_rel" == "$teleop_rel_cfg" ]] || { echo "runtime nạp teleop '$teleop_rel', config chọn '$teleop_rel_cfg'"; return 1; }
    [[ -s "$HIGH_DIR/$model_rel" ]] || { echo "thiếu model: $HIGH_DIR/$model_rel"; return 1; }
    sha="$(sha256sum "$HIGH_DIR/$model_rel" | awk '{print $1}')"
    cat >"$MANIFEST_TMP" <<EOF
# Walking policy accepted by the current controller runner.
MODEL_REL=$model_rel
MODEL_SHA256=$sha
MODEL_CONTRACT=$selected_contract
MODEL_INPUT=$input
MODEL_OUTPUT=$output
EOF
    model_rel="${model_rel#policies/locomotion/}"
    teleop_rel="${teleop_rel#policies/locomotion/}"
    echo "walk=${model_rel%/policy.onnx} teleop=${teleop_rel%/policy.onnx}"
}

# In slot dance lỗi (⚠) và ghi vào báo cáo cuối: nút nhảy đó sẽ không chạy.
dance_report() {
    local out line
    out="$(HB_COLOR=0 NO_COLOR=1 HB_DANCE_SUMMARY_QUIET=1 bash -c \
        'source "$1/status_log.sh"; source "$1/dance_preflight_summary.sh"; hb_summarize_dance_preflight "$2" "$3"' \
        _ "$SCRIPT_DIR" "$HIGH_DIR/config/dance.yaml" "$PREFLIGHT_LOG" 2>&1)" || true
    while IFS= read -r line; do
        [[ "$line" == "✗ Dance "* ]] || continue
        hb_warn "${line#✗ }"
        impact yellow "${line#✗ } -> slot nhảy này không chạy"
    done <<<"$out"
    return 0
}

sync_controller() {
    sync_high
    sync_dance
    rsync "${RSYNC_BASE[@]}" "$MANIFEST_TMP" "$ROBOT:$DEST/integration/config/model_manifest.conf"
}

# Trả bản trước (config/policy/binary, cùng công cụ nếu đã chép). Kết quả ở STEP_RC,
# chỉ theo bước trả file: cài lại unit lỗi không được chặn việc chạy lại bản cũ.
controller_restore() {
    local parts=(controller) rc
    if (( DO_TOOLS )); then parts+=(tools); fi
    step "controller: trả lại bản trước" robot restore "$BACKUP" "${parts[@]}"
    rc=$STEP_RC
    if (( rc == 0 && DO_TOOLS && INSTALL_UNITS )); then
        step "cài lại unit systemd bản trước" robot install-services
        if (( STEP_RC != 0 )); then impact red "cài lại unit systemd bản trước lỗi: kiểm tra ./hb status trước khi reboot"; fi
    fi
    STEP_RC=$rc
}

# Bản mới hỏng trước khi restart: tiến trình đang chạy vẫn là bản cũ, chỉ cần trả file.
controller_abort() {
    local why="$1"
    controller_restore
    if (( STEP_RC == 0 )); then
        robot unmark hb_high_level >>"$LOG" 2>&1 || true
        impact red "controller chưa cập nhật ($why); đã trả bản cũ, robot chạy bản cũ bình thường"
    else
        impact red "controller chưa cập nhật ($why) và trả bản cũ lỗi: tiến trình đang chạy vẫn là bản cũ, nhưng lần restart/boot sau có thể không lên. Đừng reboot; chạy lại ./hb deploy"
    fi
}

apply_controller() {
    local high_was
    high_was="$(probe high)"
    step "controller: sync" sync_controller
    if (( STEP_RC != 0 )); then controller_abort "sync lỗi"; return 0; fi
    step "controller: build trên robot" robot build high
    if (( STEP_RC != 0 )); then controller_abort "build trên robot lỗi"; return 0; fi
    step "controller: preflight trên robot" robot preflight high
    if (( STEP_RC != 0 )); then controller_abort "preflight trên robot lỗi"; return 0; fi
    install_manifest
    if (( NO_RESTART )); then
        hb_skip "controller: --no-restart, chưa restart hb_high_level"
        impact yellow "controller đã chép + preflight OK nhưng CHƯA chạy: robot vẫn chạy bản cũ đến khi ./hb deploy (hoặc reboot)"
        return 0
    fi
    step "controller: restart hb_high_level" robot restart-high
    case "$STEP_RC" in
        0)
            robot unmark hb_high_level >>"$LOG" 2>&1 || true
            if [[ "$high_was" == inactive ]]; then
                impact yellow "hb_high_level đã khởi động: robot ở DISARMED, phải arm lại mới đi được"
            else
                impact yellow "hb_high_level đã restart: robot ở DISARMED, phải arm lại mới đi được"
            fi
            ;;
        3)
            impact red "controller đã chép + preflight OK nhưng CHƯA restart: robot vừa ARMED. Robot vẫn chạy bản cũ (lần boot sau chạy bản mới). DISARMED rồi chạy lại ./hb deploy"
            ;;
        *)
            controller_restore
            if (( STEP_RC == 0 )); then
                step "controller: chạy lại bản cũ" robot restart-high
            fi
            if (( STEP_RC == 0 )); then
                robot unmark hb_high_level >>"$LOG" 2>&1 || true
                restore_manifest
                impact red "bản controller mới không chạy được -> đã quay về bản cũ; hb_high_level chạy lại, robot ở DISARMED"
            else
                impact red "ROBOT KHÔNG CÓ CONTROLLER: hb_high_level không chạy -> robot không đứng/đi bằng run_r1. Xem ./hb logs high"
            fi
            ;;
    esac
    return 0
}

# Manifest máy trạm đi theo bản đang nằm trên robot, để lần sau so được.
install_manifest() {
    cp -f -- "$HB_ROOT/integration/config/model_manifest.conf" "$TMP_DIR/manifest.prev"
    cmp -s "$MANIFEST_TMP" "$HB_ROOT/integration/config/model_manifest.conf" || \
        cp -f -- "$MANIFEST_TMP" "$HB_ROOT/integration/config/model_manifest.conf"
}
restore_manifest() { cp -f -- "$TMP_DIR/manifest.prev" "$HB_ROOT/integration/config/model_manifest.conf"; }

# ----------------------------------------------------------------- voice ----

sync_voice_parts() {
    [[ -z "$VOICE_FILES$PRESETS_FILES$INTEG_FILES" && "$TARGET" != all ]] && return 0
    sync_voice
    sync_presets
    sync_integration_service
}

apply_voice() {
    local unit mode failed=0
    step "voice: sync" sync_voice_parts
    if (( STEP_RC != 0 )); then impact red "voice/integration chưa cập nhật (sync lỗi)"; return 0; fi
    if (( BUILD_INTEG )); then
        step "voice: build hb_integration" robot build integration
        if (( STEP_RC != 0 )); then failed=1; fi
    fi
    if (( BUILD_VOICE && ! failed )); then
        step "voice: build r1_bridge + venv" robot build voice
        if (( STEP_RC != 0 )); then failed=1; fi
    fi
    if (( ! failed )); then
        for unit in "${VOICE_UNITS[@]}"; do
            case "$unit" in
                hb_integration) mode=integration ;;
                hb_voice) mode=voice ;;
                hb_voice_presets) mode=presets ;;
            esac
            step "voice: preflight $mode" robot preflight "$mode"
            if (( STEP_RC != 0 )); then failed=1; fi
        done
    fi
    if (( failed )); then
        impact red "voice/integration bản mới lỗi -> CHƯA restart, đang chạy bản cũ (motor không bị ảnh hưởng); nếu reboot, service đó có thể không lên"
        return 0
    fi
    if (( NO_RESTART )); then
        hb_skip "voice: --no-restart, chưa restart ${VOICE_UNITS[*]}"
        impact yellow "voice đã chép nhưng CHƯA chạy (${VOICE_UNITS[*]}); chạy ./hb deploy để kích hoạt"
        return 0
    fi
    step "voice: restart ${VOICE_UNITS[*]}" robot restart-units "${VOICE_UNITS[@]}"
    if (( STEP_RC != 0 )); then
        impact red "voice/integration không lên sau restart: hội thoại/PTT không hoạt động (motor không bị ảnh hưởng). Xem ./hb logs voice"
        return 0
    fi
    robot unmark "${VOICE_UNITS[@]}" >>"$LOG" 2>&1 || true
    for unit in "${VOICE_UNITS[@]}"; do
        case "$unit" in
            hb_integration) impact yellow "hb_integration restart: PTT/mic ngắt vài giây (không đụng motor)" ;;
            hb_voice) impact yellow "hb_voice restart: hội thoại gián đoạn vài giây (không đụng motor)" ;;
            hb_voice_presets) impact yellow "hb_voice_presets restart: nút giọng mẫu ngắt vài giây (không đụng motor)" ;;
        esac
    done
    step "voice: health check" robot health
    if (( STEP_RC != 0 )); then
        while IFS= read -r unit; do
            impact yellow "health: ${unit#✗ }"
        done < <(grep '✗' <<<"$STEP_TEXT" | grep -v 'health check failed' || true)
    fi
    return 0
}

# --------------------------------------------------------------- teleop ----

apply_teleop() {
    local rc=0
    step "teleop: sync" sync_teleop
    if (( STEP_RC != 0 )); then impact red "teleop chưa cập nhật (sync lỗi)"; return 0; fi
    if (( NO_RESTART )); then
        robot mark hb_teleop_runtime >>"$LOG" 2>&1 || true
        hb_skip "teleop: --no-restart, chưa restart runtime"
        impact yellow "teleop đã chép nhưng CHƯA chạy; chạy ./hb deploy để kích hoạt"
        return 0
    fi
    load_robot_teleop_settings || { impact red "teleop.yaml không hợp lệ -> chưa kích hoạt runtime mới"; return 0; }
    activate_robot_teleop_runtime || rc=$?
    case "$rc" in
        0)
            robot unmark hb_teleop_runtime >>"$LOG" 2>&1 || true
            impact yellow "teleop runtime restart: phiên Quest bị ngắt, vào lại VR; nếu đang teleop, tay về tư thế nền"
            ;;
        2) impact yellow "teleop chạy nhưng camera chưa sẵn sàng (nút A có thể không có hình)" ;;
        *) impact red "teleop runtime không sẵn sàng: Quest không điều khiển được. Xem ./hb logs teleop" ;;
    esac
    return 0
}

# ---------------------------------------------------------------- deploy ----

# Các hàm gọi `step` luôn được gọi trong thân if, không qua &&/|| (xem step()).
cmd_deploy() {
    local -a backup_parts=() mark_units=()
    detect_changes
    plan
    print_plan
    if (( HELD_BACK )); then
        impact red "controller CHƯA deploy: robot đang ARMED ($(probe high_state)). Robot chạy bản cũ bình thường. DISARMED (giữ R1+R2 khi đứng yên) rồi chạy lại ./hb deploy"
    fi
    if (( ! DO_CONTROLLER && ! DO_TOOLS && ! SYNC_VOICE_PARTS && ! DO_TELEOP && ! INSTALL_UNITS )); then
        if (( HELD_BACK )); then report; else hb_ok "robot đã khớp máy trạm: không có gì để deploy"; fi
        return 0
    fi
    if (( DRY_RUN )); then
        hb_skip "dry-run: không ghi gì lên robot"
        return 0
    fi

    if (( DO_CONTROLLER )); then
        step "máy trạm: build + preflight run_r1" local_controller_check
        if (( STEP_RC != 0 )); then
            impact red "preflight trên máy trạm lỗi: CHƯA ghi gì lên robot"
            report
            return 0
        fi
        hb_info "    $(tail -n 1 <<<"$STEP_TEXT")"
        dance_report
    fi

    if ! hb_deploy_auth_enable_sudo "$ROBOT" "$SSH_CONTROL_PATH" "$SSH_CONNECT_TIMEOUT"; then
        impact red "không có sudo trên robot: CHƯA ghi gì lên robot"
        report
        return 0
    fi

    if (( DO_CONTROLLER )); then backup_parts+=(controller); fi
    if (( DO_TOOLS )); then backup_parts+=(tools); fi
    if (( SYNC_VOICE_PARTS )); then backup_parts+=(voice); fi
    if (( DO_TELEOP )); then backup_parts+=(teleop); fi
    step "backup trên robot" robot backup "$RUN_ID" "${HB_BACKUP_KEEP:-3}" "${backup_parts[@]}"
    if (( STEP_RC != 0 )); then
        impact red "backup lỗi: CHƯA ghi gì lên robot"
        report
        return 0
    fi
    BACKUP="$(text_get backup "$STEP_TEXT")"

    # Marker trước khi ghi: deploy đứt giữa chừng thì lần sau vẫn biết phải restart.
    if (( DO_CONTROLLER )); then mark_units+=(hb_high_level); fi
    mark_units+=("${VOICE_UNITS[@]}" "${PENDING_ONLY[@]}")
    if (( ${#mark_units[@]} )); then robot mark "${mark_units[@]}" >>"$LOG" 2>&1 || true; fi

    if (( DO_TOOLS )); then
        step "công cụ: sync" sync_tools "${TOOLS_HOLD[@]}"
        if (( STEP_RC != 0 )); then impact red "công cụ integration chưa cập nhật (sync lỗi)"; fi
        sync_root >>"$LOG" 2>&1 || true
    fi
    if (( INSTALL_UNITS )); then
        step "cài unit systemd" robot install-services
        if (( STEP_RC != 0 )); then impact red "cài unit systemd lỗi: service có thể không lên ở lần restart/boot sau"; fi
    fi

    if (( DO_CONTROLLER )); then apply_controller; fi
    if (( SYNC_VOICE_PARTS )); then apply_voice; fi
    if (( ${#PENDING_ONLY[@]} )); then
        impact yellow "unit systemd của ${PENDING_ONLY[*]} đã đổi nhưng CHƯA restart (ngoài phạm vi lần này); chạy ./hb deploy để kích hoạt"
    fi
    if (( TOOLS_VERIFY )); then
        # Kiểm preflight.sh mới với bản đang có trên robot, như lần restart/boot sau sẽ chạy.
        step "công cụ: preflight service đang chạy (không restart)" robot preflight-active
        if (( STEP_RC != 0 )); then impact red "preflight.sh mới lỗi: service có thể không lên ở lần restart/boot sau"; fi
    fi
    if (( DO_TELEOP )); then apply_teleop; fi
    report
}

# -------------------------------------------------------------- rollback ----

# Trả lại backup mới nhất (= trạng thái trước lần deploy gần nhất) cho đúng các
# thành phần có trong tarball. Dance không nằm trong tarball.
cmd_rollback() {
    local members high
    PROBE="$(robot probe)"
    high="$(probe high)"
    step "tìm backup mới nhất" robot latest-backup
    if (( STEP_RC != 0 )); then impact red "không có backup để rollback"; report; return 0; fi
    BACKUP="$(text_get backup "$STEP_TEXT")"
    members="$(sed -n 's/^member=//p' <<<"$STEP_TEXT")"
    hb_info "    $BACKUP"
    if grep -q '^controller/' <<<"$members" && [[ "$high" == armed ]]; then
        impact red "rollback có controller nhưng robot đang ARMED ($(probe high_state)): KHÔNG ghi gì. DISARMED rồi chạy lại"
        report
        return 0
    fi
    step "giải nén backup" robot rollback-extract "$BACKUP"
    if (( STEP_RC != 0 )); then
        impact red "giải nén backup lỗi; robot có thể ở trạng thái lẫn lộn, kiểm tra trước khi arm"
        report
        return 0
    fi
    if grep -qE '^integration/(scripts|systemd)' <<<"$members"; then
        step "cài unit systemd" robot install-services
    fi
    if grep -q '^controller/' <<<"$members"; then
        step "controller: build trên robot" robot build high
        if (( STEP_RC == 0 )); then
            step "controller: preflight trên robot" robot preflight high
        fi
        if (( STEP_RC == 0 )); then
            step "controller: restart hb_high_level" robot restart-high
            if (( STEP_RC == 0 )); then
                impact yellow "hb_high_level đã restart: robot ở DISARMED, phải arm lại mới đi được"
            else
                impact red "hb_high_level không chạy sau rollback -> robot không đứng/đi bằng run_r1. Xem ./hb logs high"
            fi
        else
            impact red "bản controller trong backup lỗi build/preflight: chưa restart, robot vẫn chạy tiến trình hiện tại"
        fi
    fi
    if grep -qE '^(voice|voice_presets)/|^integration/src' <<<"$members"; then
        step "voice: build" robot build presets
        if (( STEP_RC == 0 )); then
            step "voice: restart" robot restart-units hb_integration hb_voice hb_voice_presets
            if (( STEP_RC == 0 )); then
                impact yellow "voice/integration restart: hội thoại/PTT gián đoạn vài giây (không đụng motor)"
            else
                impact red "voice/integration không lên sau rollback (motor không bị ảnh hưởng)"
            fi
        else
            impact red "voice trong backup lỗi build: chưa restart"
        fi
    fi
    if grep -q '^teleop/' <<<"$members"; then
        step "teleop: cài lại runtime" "${SSH[@]}" "$ROBOT" \
            "sudo -n HB_ROOT='$DEST' bash '$DEST/teleop/scripts/install_robot_runtime.sh' --start"
        if (( STEP_RC == 0 )); then
            impact yellow "teleop runtime restart: phiên Quest bị ngắt"
        else
            impact red "teleop runtime lỗi sau rollback"
        fi
    fi
    report
}

case "$COMMAND" in
    deploy) cmd_deploy ;;
    rollback) cmd_rollback ;;
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
    start-*|stop-*|restart-*|logs-*)
        # Guard DISARMED và danh sách lệnh hợp lệ nằm trong stack_ctl.sh, dùng
        # chung với `./hb` chạy trực tiếp trên robot.
        "${SSH[@]}" -t "$ROBOT" "bash '$DEST/integration/scripts/stack_ctl.sh' $COMMAND"
        ;;
esac
