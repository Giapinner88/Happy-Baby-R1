#!/usr/bin/env bash
# Prepare the canonical search runtime. Does not stop or restart motor owners.
set -euo pipefail
HB_ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
APP="$HB_ROOT/object_search"
[[ -f "$APP/manifest.json" ]] || { echo "object_search bundle missing" >&2; exit 1; }
[[ -f "$APP/experiments/r1_object_grounding/p6_onboard/requirements-py38.txt" ]] || exit 1
if [[ "$EUID" -ne 0 ]]; then
    if [[ ! -x "$APP/.venv/bin/python" ]]; then
        python3 -m venv "$APP/.venv"
    fi
    "$APP/.venv/bin/python" -m pip install -r "$APP/experiments/r1_object_grounding/p6_onboard/requirements-py38.txt"
else
    [[ -x "$APP/.venv/bin/python" ]] || { echo "Build the venv as unitree first" >&2; exit 1; }
fi
"$APP/.venv/bin/python" "$APP/experiments/r1_object_grounding/p6_onboard/preflight.py" --bundle-only
if [[ "${1:-}" != "--install-units" ]]; then
    echo "Runtime prepared. Pass --install-units to render services after reviewing existing drop-ins."
    exit 0
fi
[[ "$EUID" -eq 0 ]] || { echo "--install-units requires sudo" >&2; exit 1; }
if [[ -e /etc/systemd/system/r1-p6-search.service.d/body-turn.conf ]]; then
    echo "Legacy body-turn.conf overrides canonical ExecStart; move it aside after review." >&2
    exit 1
fi
for name in r1-p5-agent r1-p6-search; do
    sed "s|@HB_ROOT@|$HB_ROOT|g" "$HB_ROOT/integration/systemd/$name.service.in" \
        > "/etc/systemd/system/$name.service"
    chmod 0644 "/etc/systemd/system/$name.service"
done
install -d -m 0755 /etc/systemd/system/hb_teleop_runtime.service.d /etc/systemd/system/r1-p6-search.service.d
install -m 0644 "$APP/runtime/quest-search-mode.conf" /etc/systemd/system/hb_teleop_runtime.service.d/object-search-mode.conf
install -m 0644 "$APP/runtime/p6-search-mode.conf" /etc/systemd/system/r1-p6-search.service.d/object-search-mode.conf
systemctl daemon-reload
touch /etc/hb/object_search_enabled
echo "Units rendered. When DISARMED, select the persistent sender mode:"
echo "sudo bash $APP/runtime/runtime_mode.sh search  # or quest"
