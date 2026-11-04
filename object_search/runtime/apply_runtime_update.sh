#!/usr/bin/env bash
# Apply a staged canonical bundle and select search mode, after read-only checks.
set -euo pipefail
[[ "$EUID" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
HB_ROOT="${HB_ROOT:-/home/unitree/HB}"
stage_dir=$(cd "$(dirname "$0")/.." && pwd)
APP="$HB_ROOT/object_search"
bash "$stage_dir/runtime/runtime_mode.sh" check
"$APP/.venv/bin/python" "$stage_dir/experiments/r1_object_grounding/p6_onboard/preflight.py" --bundle-dir "$stage_dir" --bundle-only
backup_dir=$(mktemp -d "$HB_ROOT/../HB_backups/runtime_update_XXXXXXXX")
tar --exclude=object_search/.venv --exclude='*/__pycache__' -czf "$backup_dir/object_search.tar.gz" -C "$HB_ROOT" object_search
cp -a "$HB_ROOT/voice/config/tuning.yaml" "$backup_dir/voice_tuning.yaml"
bash "$stage_dir/runtime/runtime_mode.sh" check
# No active task is possible while DISARMED; stop only the search server before sync.
systemctl stop r1-p6-search.service
runuser -u unitree -- rsync -a "$stage_dir/" "$APP/"
"$APP/.venv/bin/python" "$APP/experiments/r1_object_grounding/p6_onboard/preflight.py" --bundle-only
bash "$APP/runtime/runtime_mode.sh" search
echo "Update complete; application backup=$backup_dir"
