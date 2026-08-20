#!/usr/bin/env bash
# Đồng bộ teleop lên robot. CHỈ copy file: không enable service, không arm motor.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DEST="${DEST:-/home/unitree/HB}"
COMMAND="${1:-diff}"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/_find_robot.sh"
find_robot

SSH_CONTROL_PATH="${HB_SSH_CONTROL_PATH:-/tmp/hb-ssh-%C}"
RSYNC_SSH="ssh -o ControlMaster=auto -o ControlPersist=300 -o ControlPath=$SSH_CONTROL_PATH"
RSYNC_BASE=(-az --no-perms --omit-dir-times --itemize-changes -e "$RSYNC_SSH")
EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/'
          --exclude 'logs/' --exclude 'docs/' --exclude '*.env')

sync_teleop() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${EXCLUDES[@]}" \
        "$TELEOP_DIR/" "$ROBOT:$DEST/teleop/"
}

case "$COMMAND" in
    diff)
        echo "== dry-run: sẽ thay đổi những gì =="
        sync_teleop --dry-run
        ;;
    deploy)
        "$SCRIPT_DIR/preflight.sh"
        sync_teleop
        echo
        echo "[OK] Đã copy file lên $ROBOT:$DEST/teleop/"
        echo "     KHÔNG có service nào được bật và KHÔNG có motor nào được ghi."
        echo "     Bật service là thao tác thủ công; đọc docs/hardware_gate.md trước."
        ;;
    status)
        # shellcheck disable=SC2029
        ssh "$ROBOT" "cat $DEST/teleop/src/SOURCE.txt 2>/dev/null || echo 'chưa deploy'"
        ;;
    *)
        echo "Dùng: $0 {diff|deploy|status}" >&2
        exit 2
        ;;
esac
