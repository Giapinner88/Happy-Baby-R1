#!/usr/bin/env bash
# Các bước deploy chạy TRÊN ROBOT. deploy_stack.sh đẩy chính file này qua SSH
# (`ssh robot bash -s -- <bước> ...`), nên robot không cần bản mới của nó trước khi
# deploy. Mỗi bước in key=value (để máy trạm đọc) hoặc log thường.
#
# hb_high_level giữ rt/lowcmd: chỉ restart khi run_r1 báo DISARMED (hoặc service
# không chạy). Voice/integration/presets restart với ignore-dependencies để không
# bao giờ kéo hb_high_level lên qua Wants=.
set -euo pipefail

DEST="${HB_DEST:?HB_DEST is required}"
STATUS="${HB_STATUS_FILE:-/run/hb/status.env}"
MARKS="$DEST/.hb_deploy"
BACKUPS="$(dirname "$DEST")/HB_backups"
OPS="$DEST/integration/scripts"
HB_UNITS=(hb_high_level hb_integration hb_voice hb_voice_presets hb_teleop_runtime)

unit_active() { systemctl is-active --quiet "$1.service"; }

# disarmed | inactive -> an toàn để restart; armed -> không.
high_state() {
    if ! unit_active hb_high_level; then
        echo inactive
    elif grep -qx 'high_alive=1' "$STATUS" 2>/dev/null &&
         grep -qx 'high_armed=0' "$STATUS" 2>/dev/null &&
         grep -q '^high_state=DISARMED' "$STATUS" 2>/dev/null; then
        echo disarmed
    else
        echo armed
    fi
}

# Marker "đã chép nhưng chưa restart". Hết hiệu lực khi unit đã khởi động lại sau
# lúc đặt marker (reboot, ./hb restart ...).
pending() {
    local unit="$1" mark="$MARKS/pending_$1" started
    [[ -f "$mark" ]] || { echo 0; return; }
    if unit_active "$unit"; then
        started="$(systemctl show "$unit.service" -p ActiveEnterTimestamp --value)"
        started="$(date -d "$started" +%s 2>/dev/null || echo 0)"
        if (( started > $(stat -c %Y "$mark") )); then
            rm -f "$mark"
            echo 0
            return
        fi
    fi
    echo 1
}

# Chờ unit chạy ổn định (cùng PID, active) trong ~3 giây.
wait_stable() {
    local unit="$1" old_pid="${2:-}" timeout="${3:-60}" pid last="" stable=0 i
    for ((i = 0; i < timeout * 2; i++)); do
        pid="$(systemctl show "$unit.service" -p MainPID --value)"
        if unit_active "$unit" && [[ "$pid" != 0 && "$pid" != "$old_pid" ]]; then
            if [[ "$pid" == "$last" ]]; then
                stable=$((stable + 1))
            else
                stable=0
                last="$pid"
            fi
            (( stable >= 6 )) && return 0
        else
            stable=0
        fi
        sleep 0.5
    done
    return 1
}

step_probe() {
    local unit high_sha=none
    echo "high=$(high_state)"
    echo "high_state=$(sed -n 's/^high_state=//p' "$STATUS" 2>/dev/null || true)"
    for unit in "${HB_UNITS[@]}"; do
        echo "pending_$unit=$(pending "$unit")"
    done
    if [[ -r "$DEST/integration/config/model_manifest.conf" ]]; then
        high_sha="$(sha256sum "$DEST/integration/config/model_manifest.conf" | awk '{print $1}')"
    fi
    echo "manifest_sha=$high_sha"
    echo "bin_run_r1=$([[ -x "$DEST/controller/build/run_r1" ]] && echo 1 || echo 0)"
    echo "bin_r1_bridge=$([[ -x "$DEST/voice/unitree_bridge/build/r1_bridge" ]] && echo 1 || echo 0)"
    echo "bin_integration=$([[ -x "$DEST/integration/build/hb_integration" ]] && echo 1 || echo 0)"
    echo "bin_venv=$([[ -x "$DEST/voice/.venv/bin/python" ]] && echo 1 || echo 0)"
    echo "teleop_installed=$(systemctl is-enabled --quiet hb_teleop_runtime.service 2>/dev/null && echo 1 || echo 0)"
}

# backup <id> <keep> <controller|tools|voice|teleop>...
# Tarball chỉ chứa phần sắp bị ghi đè. Không chứa build/, thirdparty/,
# policies/dance (1.9 GB, đã có backup/ riêng) và file secret.
step_backup() {
    local id="$1" keep="$2" part free_mb tarball compress=()
    shift 2
    local -a paths=()
    for part in "$@"; do
        case "$part" in
            # Manifest đi cùng controller: config cũ + manifest mới thì ExecStartPre fail.
            controller) paths+=(controller integration/config/model_manifest.conf) ;;
            tools) paths+=(integration/scripts integration/systemd integration/config) ;;
            voice) paths+=(voice voice_presets integration/src integration/CMakeLists.txt) ;;
            teleop) paths+=(teleop) ;;
            *) echo "unknown backup part: $part" >&2; return 2 ;;
        esac
    done
    local -a present=()
    for part in "${paths[@]}"; do
        [[ -e "$DEST/$part" ]] && present+=("$part")
    done
    mkdir -p "$BACKUPS" "$MARKS"
    if [[ " $* " == *" controller "* && -x "$DEST/controller/build/run_r1" ]]; then
        cp -p "$DEST/controller/build/run_r1" "$DEST/controller/build/run_r1.prev"
    fi
    if (( ${#present[@]} == 0 )); then
        echo "backup=none"
        return 0
    fi
    free_mb="$(df -Pm "$BACKUPS" | awk 'NR == 2 {print $4}')"
    if (( free_mb < 1024 )); then
        echo "robot chỉ còn ${free_mb} MiB trống trong $BACKUPS; cần >= 1024 MiB" >&2
        return 1
    fi
    command -v pigz >/dev/null && compress=(-I pigz) || compress=(-z)
    tarball="$BACKUPS/HB_${id}.tar.gz"
    tar "${compress[@]}" -cf "$tarball" -C "$DEST" \
        --exclude='controller/build' --exclude='controller/thirdparty' \
        --exclude='controller/policies/dance' --exclude='controller/docs' \
        --exclude='*/build' --exclude='*/.venv' --exclude='*/logs' --exclude='*/results' \
        --exclude='__pycache__' --exclude='*/.env' --exclude='*/.env.*' --exclude='*.env' \
        "${present[@]}"
    # Giữ <keep> bản mới nhất; chỉ đụng HB_*.tar.gz do deploy tạo.
    ls -1t "$BACKUPS"/HB_*.tar.gz | tail -n +"$((keep + 1))" | xargs -r rm -f --
    echo "backup=$tarball"
    echo "size=$(du -h "$tarball" | cut -f1)"
}

# restore <tarball> <controller|tools>...: trả lại phần runtime của bản trước
# (config, policy, binary run_r1, công cụ integration). Không trả src/ để build
# dir không lệch mtime; binary cũ đến từ run_r1.prev.
step_restore() {
    local tarball="$1" part listing
    shift
    local -a wanted=() members=()
    for part in "$@"; do
        case "$part" in
            controller) wanted+=(controller/config controller/policies controller/motions controller/src/audio
                                 integration/config/model_manifest.conf) ;;
            tools) wanted+=(integration/scripts integration/systemd integration/config) ;;
            *) echo "unknown restore part: $part" >&2; return 2 ;;
        esac
    done
    if [[ -f "$tarball" ]]; then
        listing="$(tar -tzf "$tarball")"
        for part in "${wanted[@]}"; do
            grep -qE "^$part(/|$)" <<<"$listing" && members+=("$part")
        done
        (( ${#members[@]} == 0 )) || tar -xzmf "$tarball" -C "$DEST" "${members[@]}"
        echo "restored=${members[*]:-none}"
    else
        echo "restored=none (no tarball)"
    fi
    if [[ " $* " == *" controller "* ]]; then
        if [[ -x "$DEST/controller/build/run_r1.prev" ]]; then
            cp -p "$DEST/controller/build/run_r1.prev" "$DEST/controller/build/run_r1.tmp"
            mv -f "$DEST/controller/build/run_r1.tmp" "$DEST/controller/build/run_r1"
            echo "binary=run_r1.prev"
        else
            echo "binary=missing-prev" >&2
            return 1
        fi
    fi
}

# nice: build/preflight có thể chạy khi robot đang ARMED (deploy voice/teleop);
# nhường CPU cho run_r1.
step_preflight() {
    sudo -n nice -n 19 env HB_ROOT="$DEST" HB_DANCE_SUMMARY_QUIET=1 bash "$OPS/preflight.sh" "$1"
}

# Preflight cho từng service đang chạy: đúng những gì ExecStartPre sẽ kiểm ở lần
# restart/boot sau.
step_preflight_active() {
    local unit mode failed=0
    for unit in hb_high_level hb_integration hb_voice hb_voice_presets; do
        unit_active "$unit" || continue
        case "$unit" in
            hb_high_level) mode=high ;;
            hb_integration) mode=integration ;;
            hb_voice) mode=voice ;;
            hb_voice_presets) mode=presets ;;
        esac
        step_preflight "$mode" || failed=1
    done
    return "$failed"
}

step_build() {
    HB_ROOT="$DEST" HB_COLOR=0 nice -n 19 bash "$OPS/build_on_robot.sh" "$1"
}

step_install_services() {
    sudo -n env HB_ROOT="$DEST" bash "$OPS/install_services.sh"
}

step_mark() {
    mkdir -p "$MARKS"
    local unit
    for unit in "$@"; do touch "$MARKS/pending_$unit"; done
}

step_unmark() {
    local unit
    for unit in "$@"; do rm -f "$MARKS/pending_$unit"; done
}

# Exit 3: robot ARMED, không restart. Exit 4: service không lên.
step_restart_high() {
    local state old_pid i
    state="$(high_state)"
    if [[ "$state" == armed ]]; then
        echo "refused: high_state=$(sed -n 's/^high_state=//p' "$STATUS" 2>/dev/null || echo unknown)"
        return 3
    fi
    old_pid="$(systemctl show hb_high_level.service -p MainPID --value)"
    sudo -n systemctl restart hb_high_level.service
    wait_stable hb_high_level "$old_pid" 90 || {
        journalctl -u hb_high_level.service -n 30 --no-pager -o cat
        return 4
    }
    # Có coordinator thì đợi heartbeat DISARMED của tiến trình mới.
    if unit_active hb_integration; then
        for ((i = 0; i < 40; i++)); do
            [[ "$(high_state)" == disarmed ]] && { echo "high=disarmed"; return 0; }
            sleep 0.5
        done
        echo "high-level chạy nhưng chưa thấy heartbeat DISARMED" >&2
        journalctl -u hb_high_level.service -n 30 --no-pager -o cat
        return 4
    fi
    echo "high=active"
}

step_restart_units() {
    local unit failed=0
    local -a services=()
    for unit in "$@"; do services+=("$unit.service"); done
    sudo -n systemctl --job-mode=ignore-dependencies restart "${services[@]}"
    for unit in "$@"; do
        if ! wait_stable "$unit" "" 30; then
            journalctl -u "$unit.service" -n 20 --no-pager -o cat
            echo "$unit không lên" >&2
            failed=1
        fi
    done
    return "$failed"
}

step_health() {
    bash "$OPS/health_check.sh" --wait-voice --quiet
}

step_latest_backup() {
    local latest
    latest="$(ls -1t "$BACKUPS"/HB_*.tar.gz 2>/dev/null | head -1)"
    [[ -n "$latest" ]] || { echo "không có tarball trong $BACKUPS" >&2; return 1; }
    echo "backup=$latest"
    tar -tzf "$latest" | awk -F/ '{print $1 "/" $2}' | sort -u | sed 's/^/member=/'
}

# rollback-extract <tarball>: giải nén toàn bộ với mtime mới (-m) để build
# tăng dần biên dịch lại đúng src cũ.
step_rollback_extract() {
    tar -xzmf "$1" -C "$DEST"
}

main() {
    local step="${1:-}"
    [[ $# -gt 0 ]] && shift
    case "$step" in
        probe) step_probe ;;
        backup) step_backup "$@" ;;
        restore) step_restore "$@" ;;
        preflight) step_preflight "$@" ;;
        preflight-active) step_preflight_active ;;
        build) step_build "$@" ;;
        install-services) step_install_services ;;
        mark) step_mark "$@" ;;
        unmark) step_unmark "$@" ;;
        restart-high) step_restart_high ;;
        restart-units) step_restart_units "$@" ;;
        health) step_health ;;
        latest-backup) step_latest_backup ;;
        rollback-extract) step_rollback_extract "$@" ;;
        *) echo "robot_steps: unknown step '$step'" >&2; return 2 ;;
    esac
}

# Script tới qua stdin của `bash -s`: tiến trình con không được đọc stdin đó.
main "$@" </dev/null
