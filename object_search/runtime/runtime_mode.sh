#!/usr/bin/env bash
# Select exactly one upper-body sender stack; never restart the motor owner.
set -euo pipefail
mode="${1:-}"
case "$mode" in search|quest|check) ;; *) echo "Usage: sudo bash $0 search|quest|check" >&2; exit 2;; esac
[[ "$EUID" -eq 0 ]] || { echo "Run this script with sudo" >&2; exit 1; }
HB_ROOT="${HB_ROOT:-/home/unitree/HB}"
APP="$HB_ROOT/object_search"

check_disarmed() {
  python3 - <<'PY'
from pathlib import Path
import time
status = dict(line.split('=', 1) for line in Path('/run/hb/status.env').read_text().splitlines() if '=' in line)
age = time.monotonic() - int(status['updated_monotonic_ms']) / 1000.0
if not (0 <= age <= 3 and status.get('high_alive') == '1'
        and status.get('high_armed') == '0'
        and status.get('high_state', '').startswith('DISARMED')
        and status.get('high_busy') == '0'):
    raise SystemExit('Refusing mode switch: require fresh DISARMED, high_alive=1, high_busy=0')
PY
}
check_disarmed
if [[ "$mode" == check ]]; then exit 0; fi
"$APP/.venv/bin/python" "$APP/experiments/r1_object_grounding/p6_onboard/preflight.py" --bundle-only
mkdir -p "$HB_ROOT/../HB_backups"
backup_dir=$(mktemp -d "$HB_ROOT/../HB_backups/runtime_mode_XXXXXXXX")
systemctl is-enabled hb_teleop_runtime.service > "$backup_dir/quest_enabled.txt" || true
systemctl is-enabled r1-p6-search.service > "$backup_dir/search_enabled.txt" || true
for name in hb_teleop_runtime r1-p6-search; do
  systemctl cat "$name.service" > "$backup_dir/$name.unit.txt"
done
if [[ -e /etc/hb/object_search_enabled ]]; then touch "$backup_dir/object_search_enabled"; fi
check_disarmed
systemctl stop r1-p6-search.service hb_teleop_runtime.service
install -d -m 0755 /etc/systemd/system/hb_teleop_runtime.service.d /etc/systemd/system/r1-p6-search.service.d
install -m 0644 "$APP/runtime/quest-search-mode.conf" /etc/systemd/system/hb_teleop_runtime.service.d/object-search-mode.conf
install -m 0644 "$APP/runtime/p6-search-mode.conf" /etc/systemd/system/r1-p6-search.service.d/object-search-mode.conf
if [[ "$mode" == search ]]; then
  touch /etc/hb/object_search_enabled
  systemctl disable hb_teleop_runtime.service
  systemctl enable r1-p5-agent.service r1-p6-search.service
else
  if [[ -e /etc/hb/object_search_enabled ]]; then
    mv /etc/hb/object_search_enabled "$backup_dir/marker_removed"
  fi
  systemctl disable r1-p6-search.service
  systemctl enable hb_teleop_runtime.service
fi
systemctl daemon-reload
if [[ "$mode" == search ]]; then
  systemctl start r1-p5-agent.service r1-p6-search.service
else
  systemctl start hb_teleop_runtime.service
fi
systemctl restart hb_voice.service
echo "Runtime mode=$mode; persisted across reboot; backup=$backup_dir"
