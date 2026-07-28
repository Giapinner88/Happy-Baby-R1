#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROBOT="${ROBOT:-}"
DEST="${DEST:-/home/unitree/HB}"
COMMAND="${1:-diff}"
OPTION="${2:-}"

# Dùng chung cơ chế dò robot với các script high_level_2. Có thể đặt sẵn
# ROBOT=unitree@<ip-or-hostname> để bỏ qua bước dò.
# shellcheck disable=SC1091
source "$HB_ROOT/high_level_2/scripts/_find_robot.sh"
find_robot

SSH_CONTROL_PATH="${HB_SSH_CONTROL_PATH:-/tmp/hb-ssh-%C}"
SSH_OPTS=(-o ControlMaster=auto -o ControlPersist=300 -o "ControlPath=$SSH_CONTROL_PATH")
SSH=(ssh "${SSH_OPTS[@]}")
RSYNC_SSH="ssh -o ControlMaster=auto -o ControlPersist=300 -o ControlPath=$SSH_CONTROL_PATH"
RSYNC_BASE=(-az --no-perms --omit-dir-times --itemize-changes -e "$RSYNC_SSH")
COMMON_EXCLUDES=(--exclude '__pycache__/' --exclude '*.pyc' --exclude '.cache/' --exclude 'logs/')

sync_high() {
    rsync "${RSYNC_BASE[@]}" "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'build/' --exclude 'thirdparty/onnxruntime/' --exclude 'docs/' \
        "$HB_ROOT/high_level_2/" "$ROBOT:$DEST/high_level_2/"
}

sync_voice() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
        --exclude 'unitree_bridge/build/' \
        "$HB_ROOT/voice_r1/" "$ROBOT:$DEST/voice_r1/"
}

sync_integration() {
    rsync "${RSYNC_BASE[@]}" --delete-delay "$@" "${COMMON_EXCLUDES[@]}" --exclude 'build/' \
        "$HB_ROOT/r1_integration/" "$ROBOT:$DEST/r1_integration/"
}

# Mirror CHỈ policies/dance/ với --delete: khi bạn xóa policy/npz cũ bên dev thì
# robot cũng xóa theo, để mỗi folder dance chỉ còn đúng 1 .onnx + 1 .npz và
# ScanDanceFolder không vớ nhầm file cũ (khỏi phải lên NoMachine xóa tay).
# Chừa 'backup/' để KHÔNG bao giờ lỡ xóa bản lưu trên robot.
sync_dance() {
    rsync "${RSYNC_BASE[@]}" --delete "$@" "${COMMON_EXCLUDES[@]}" \
        --exclude 'backup/' \
        "$HB_ROOT/high_level_2/policies/dance/" "$ROBOT:$DEST/high_level_2/policies/dance/"
}

case "$COMMAND" in
    diff)
        sync_high --dry-run
        sync_voice --dry-run
        sync_integration --dry-run
        sync_dance --dry-run
        ;;
    deploy)
        if [[ -n "$OPTION" && "$OPTION" != "--no-restart" && \
              "$OPTION" != "--restart-voice" && "$OPTION" != "--accept-policy" ]]; then
            echo "Usage: $0 deploy [--no-restart|--restart-voice|--accept-policy]" >&2
            exit 2
        fi
        if [[ "$OPTION" == "--accept-policy" ]]; then
            bash "$SCRIPT_DIR/update_model_manifest.sh" --accept
            OPTION=""
        fi
        "${SSH[@]}" "$ROBOT" "mkdir -p '$DEST/high_level_2' '$DEST/voice_r1' '$DEST/r1_integration' '$DEST/../HB_backups'; tar -czf '$DEST/../HB_backups/HB_$(date +%Y%m%d_%H%M%S).tar.gz' -C '$DEST' --exclude='*/build' --exclude='*/.venv' --exclude='*/logs' --exclude='*/.env' --exclude='*/.env.*' high_level_2 voice_r1 r1_integration 2>/dev/null || true"
        sync_high
        sync_voice
        sync_integration
        sync_dance
        "${SSH[@]}" "$ROBOT" "HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/build_on_robot.sh'"
        if [[ "$OPTION" == "--restart-voice" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/activate_services.sh' --no-restart && sudo systemctl restart hb_integration.service hb_voice.service && bash '$DEST/r1_integration/scripts/health_check.sh' --wait-voice; echo 'DEPLOY_OK restart=integration+voice'"
        elif [[ "$OPTION" == "--no-restart" ]]; then
            "${SSH[@]}" -t "$ROBOT" "sudo HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/activate_services.sh' --no-restart"
        else
            "${SSH[@]}" -t "$ROBOT" "sudo HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/activate_services.sh'"
        fi
        ;;
    status)
        "${SSH[@]}" "$ROBOT" "bash '$DEST/r1_integration/scripts/health_check.sh'; systemctl show hb_high_level hb_integration hb_voice -p Id -p ActiveState -p SubState -p NRestarts --no-pager"
        ;;
    pull)
        DRY=()
        [[ "$OPTION" == "--dry-run" ]] && DRY=(--dry-run)
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
            --exclude 'build/' --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
            "$ROBOT:$DEST/high_level_2/" "$HB_ROOT/high_level_2/"
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" \
            --exclude 'build/' --exclude '.venv/' --exclude '.env' --exclude '.env.plan0.bak' \
            "$ROBOT:$DEST/voice_r1/" "$HB_ROOT/voice_r1/"
        rsync "${RSYNC_BASE[@]}" "${DRY[@]}" "${COMMON_EXCLUDES[@]}" --exclude 'build/' \
            "$ROBOT:$DEST/r1_integration/" "$HB_ROOT/r1_integration/"
        ;;
    rollback)
        "${SSH[@]}" -t "$ROBOT" "latest=\$(ls -1t '$DEST/../HB_backups'/HB_*.tar.gz 2>/dev/null | head -1); test -n \"\$latest\"; tar -xzf \"\$latest\" -C '$DEST'; HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/build_on_robot.sh'; sudo HB_ROOT='$DEST' bash '$DEST/r1_integration/scripts/activate_services.sh'"
        ;;
    *)
        echo "Usage: $0 diff|deploy [--no-restart|--restart-voice|--accept-policy]|status|pull [--dry-run]|rollback" >&2
        exit 2
        ;;
esac
