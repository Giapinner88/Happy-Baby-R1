#!/usr/bin/env bash
# Voice assistant tasks on HB's own services: find + face + approach + re-find, camera consent gate.
#   bash install_assistant.sh check                       # as unitree: files, Python envs, voice hook
#   sudo bash install_assistant.sh enable --operator-ready  # DISARMED + supported: render config, start
#   sudo bash install_assistant.sh disable                 # DISARMED: back to hb_local_approach / object_search voice
# enable: hb_high_level accepts nonzero approach commands (R1_APPROACH_PORT/ALLOW_NONZERO in stack.env),
# hb_local_approach is stopped (same camera/instance locks), hb_voice loads assistant/overlay (run_voice.sh).
# Code comes from VLA (scripts/interaction/export_r1_assistant_hb.py); do not edit assistant/ here.
set -euo pipefail
HB_ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
APP="$HB_ROOT/assistant"
ETC=/etc/hb/assistant
MARKER=/etc/hb/assistant_enabled
STACK_ENV=/etc/hb/stack.env
APPROACH_PYTHON="${APPROACH_PYTHON:-/home/unitree/r1_approach_local_env/bin/python}"
RUN_USER="${RUN_USER:-unitree}"
die() { echo "install_assistant: $*" >&2; exit 1; }
need_root() { [[ "$EUID" -eq 0 ]] || die "$1 requires sudo"; }
disarmed() {
    grep -qx 'high_armed=0' /run/hb/status.env 2>/dev/null || die "owner must be DISARMED and the robot supported per SOP"
}

check() {
    [[ -f "$APP/EXPORT.json" && -f "$APP/bin/r1-assistant" ]] || die "assistant/ bundle missing; export it from VLA"
    [[ -x "$APPROACH_PYTHON" ]] || die "approach Python missing: $APPROACH_PYTHON"
    [[ -x "$HB_ROOT/object_search/.venv/bin/python" ]] || die "object_search venv missing (search uses it); run install_object_search.sh"
    [[ -x "$HB_ROOT/voice/.venv/bin/python" ]] || die "voice venv missing"
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$APP/runtime:$APP/app" "$APPROACH_PYTHON" -c \
        'import r1_assistant.service, r1_assistant.backends; from r1_assistant.config import load; load("'"$APP"'/config/robot.json")' \
        || die "assistant runtime does not import"
    PYTHONDONTWRITEBYTECODE=1 R1_ASSISTANT_VOICE=1 R1_P5_VOICE_ENABLED=0 \
        PYTHONPATH="$APP/overlay:$APP/runtime:$HB_ROOT/voice" "$HB_ROOT/voice/.venv/bin/python" -c \
        "from hb_voice.vision_tools import VisionToolController; c=VisionToolController(network_interface='lo'); \
assert {'find_and_point','approach_object','cancel_task','task_status'} <= {t.name for t in c.tools}" \
        || die "voice extension check failed"
    echo "assistant check OK"
}

set_env() {  # set_env KEY VALUE in stack.env (mode 600 kept)
    if grep -q "^$1=" "$STACK_ENV"; then sed -i "s|^$1=.*|$1=$2|" "$STACK_ENV"; else echo "$1=$2" >> "$STACK_ENV"; fi
}

enable() {
    need_root enable
    [[ "${1:-}" == --operator-ready ]] || die "enable requires --operator-ready: supported robot, clear area, E-stop operator per SOP"
    disarmed
    [[ ! -e /etc/r1-assistant/enabled ]] || die "private r1_assistant stack is active; run its rollback first"
    check
    install -d -m 0755 "$ETC" /home/unitree/hb_assistant/runs
    chown "$RUN_USER:$RUN_USER" /home/unitree/hb_assistant /home/unitree/hb_assistant/runs
    python3 - "$APP/config" "$ETC" <<'PY'
import json, os, sys
src, dst = sys.argv[1], sys.argv[2]
robot = json.load(open(os.path.join(src, "robot.json"))); robot["mode"] = "execute"
profile = json.load(open(os.path.join(src, "approach.json")))
profile["execute"] = profile["clear_corridor_confirmed"] = True
profile["planner"]["corridor"]["operator_confirmed"] = True
profile["gait_approach"]["execute_enabled"] = True
for name, value in (("robot.json", robot), ("approach.json", profile)):
    path = os.path.join(dst, name); tmp = path + ".tmp"
    with open(tmp, "w") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False); handle.write("\n"); handle.flush(); os.fsync(handle.fileno())
    os.replace(tmp, path)
PY
    install -m 0644 "$APP/config/adapter_argv.json" "$ETC/adapter_argv.json"
    systemctl is-enabled hb_local_approach >/dev/null 2>&1 && echo enabled > "$ETC/local_approach.state" || echo disabled > "$ETC/local_approach.state"
    [[ -f "$STACK_ENV.assistant.bak" ]] || cp -p "$STACK_ENV" "$STACK_ENV.assistant.bak"
    set_env R1_APPROACH_PORT 5573
    set_env R1_APPROACH_ALLOW_NONZERO 1
    sed -e "s|@HB_ROOT@|$HB_ROOT|g" -e "s|@RUN_USER@|$RUN_USER|g" -e "s|@APPROACH_PYTHON@|$APPROACH_PYTHON|g" \
        "$HB_ROOT/integration/systemd/hb_assistant.service.in" > /etc/systemd/system/hb_assistant.service
    chmod 0644 /etc/systemd/system/hb_assistant.service
    install -d -m 0755 /etc/systemd/system/hb_voice.service.d
    printf '[Service]\n# Camera lease file shared with hb_assistant.\nReadWritePaths=-/run/hb-approach\n' \
        > /etc/systemd/system/hb_voice.service.d/assistant.conf
    systemctl disable --now hb_local_approach >/dev/null 2>&1 || true
    touch "$MARKER"
    sync
    systemctl daemon-reload
    systemctl restart hb_high_level
    systemctl enable --now hb_assistant
    systemctl restart hb_voice
    sync
    echo "assistant enabled; arm with R2+A (walk) and give a voice command"
}

disable() {
    need_root disable
    disarmed
    rm -f "$MARKER"
    systemctl disable --now hb_assistant >/dev/null 2>&1 || true
    rm -f /etc/systemd/system/hb_voice.service.d/assistant.conf
    set_env R1_APPROACH_ALLOW_NONZERO 0
    if [[ "$(cat "$ETC/local_approach.state" 2>/dev/null)" == enabled ]]; then systemctl enable --now hb_local_approach || true; fi
    sync
    systemctl daemon-reload
    systemctl restart hb_high_level
    systemctl restart hb_voice
    sync
    echo "assistant disabled"
}

case "${1:-check}" in
    check) check ;;
    enable) shift; enable "$@" ;;
    disable) disable ;;
    *) die "usage: install_assistant.sh check | enable --operator-ready | disable" ;;
esac
