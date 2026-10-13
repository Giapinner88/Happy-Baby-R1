#!/usr/bin/env bash
# Stop stale local Quest teleop processes before starting a new session.
# Only commands belonging to this HB teleop pipeline are eligible.
set -euo pipefail

PORT="${1:-8012}"
declare -a TELEOP_PIDS=()
declare -a UNKNOWN_PIDS=()

is_teleop_command() {
    local args="$1"
    case "$args" in
        *"/HB/teleop/"*|*"quest_bridge.py"*|*"run_r1_vendor_ik_stream.py"*|\
        *"run_r1_vendor_targets.py"*|*"run_r1_quest3_hardware_targets.py"*|\
        *"teleop.hardware.high_level_sidecar"*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

is_skipped_pid() {
    local candidate="$1" current="$PPID"
    while [[ "$current" =~ ^[0-9]+$ && "$current" -gt 1 ]]; do
        [[ "$candidate" == "$current" ]] && return 0
        current="$(ps -o ppid= -p "$current" 2>/dev/null | tr -d ' ')"
    done
    return 1
}

add_candidate() {
    local pid="$1" args
    [[ "$pid" =~ ^[0-9]+$ && "$pid" -gt 1 ]] || return 0
    is_skipped_pid "$pid" && return 0
    for existing in "${TELEOP_PIDS[@]}" "${UNKNOWN_PIDS[@]}"; do
        [[ "$existing" == "$pid" ]] && return 0
    done
    args="$(ps -o args= -p "$pid" 2>/dev/null || true)"
    if is_teleop_command "$args"; then
        TELEOP_PIDS+=("$pid")
    else
        UNKNOWN_PIDS+=("$pid")
    fi
}

# Find the process listening on the configured Vuer port.
while read -r pid; do
    add_candidate "$pid"
done < <(
    ss -ltnpH "sport = :$PORT" 2>/dev/null \
        | grep -oE 'pid=[0-9]+' \
        | cut -d= -f2 \
        | sort -u
)

# Find orphaned stages left after a previous pipeline was interrupted.
while read -r pid; do
    add_candidate "$pid"
done < <(
    ps -eo pid=,args= \
        | awk '/quest_bridge\.py|run_r1_vendor_ik_stream\.py|run_r1_vendor_targets\.py|run_r1_quest3_hardware_targets\.py|teleop\.hardware\.high_level_sidecar/ {print $1}'
)

if ((${#UNKNOWN_PIDS[@]} > 0)); then
    for pid in "${UNKNOWN_PIDS[@]}"; do
        echo "[CLEANUP] Bỏ qua listener không xác định PID=$pid: $(ps -o args= -p "$pid" 2>/dev/null || true)" >&2
    done
    echo "[FAIL] Cổng $PORT đang do tiến trình không thuộc HB teleop chiếm; không tự kill." >&2
    exit 1
fi

if ((${#TELEOP_PIDS[@]} == 0)); then
    echo "[CLEANUP] Không có phiên teleop cũ cần dọn (port $PORT)."
    exit 0
fi

echo "[CLEANUP] Dừng phiên teleop cũ: ${TELEOP_PIDS[*]}"
kill -TERM "${TELEOP_PIDS[@]}" 2>/dev/null || true
for _ in {1..20}; do
    alive=()
    for pid in "${TELEOP_PIDS[@]}"; do
        kill -0 "$pid" 2>/dev/null && alive+=("$pid") || true
    done
    ((${#alive[@]} == 0)) && break
    sleep 0.1
done

for pid in "${TELEOP_PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
done
echo "[CLEANUP] Đã giải phóng pipeline teleop cũ và port $PORT."
