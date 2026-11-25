#!/usr/bin/env bash
# Voice assistant tasks on HB's own services: find + face + approach + re-find, camera consent gate.
#   bash install_assistant.sh prepare                      # as unitree, once: HB/assistant/.venv + models/dac01
#   bash install_assistant.sh pin                          # as unitree: pin vision/look-at config to HB's walk_policy
#   bash install_assistant.sh check                        # as unitree: files, envs, depth model, policy pin, owner, voice hook
#   sudo bash install_assistant.sh enable --operator-ready  # DISARMED + supported: render config, start
#   sudo bash install_assistant.sh disable                 # DISARMED: back to hb_local_approach / object_search voice
# Self-contained: everything runs from /home/unitree/HB (code, venv, depth model) plus HB's own services; it does not
# use /home/unitree/r1_assistant and refuses to enable while that experimental stack is active.
# enable: hb_high_level accepts nonzero approach commands (R1_APPROACH_PORT/ALLOW_NONZERO in stack.env),
# hb_local_approach is stopped (same camera/instance locks), hb_voice loads assistant/overlay (run_voice.sh).
# Code comes from VLA (scripts/interaction/export_r1_assistant_hb.py); do not edit assistant/ here.
set -euo pipefail
HB_ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
APP="$HB_ROOT/assistant"
ETC=/etc/hb/assistant
MARKER=/etc/hb/assistant_enabled
STACK_ENV=/etc/hb/stack.env
VENV="$APP/.venv"
MODEL="$APP/models/dac01"
LEGACY_MODEL="${LEGACY_DAC_MODEL:-/home/unitree/r1_local_approach/models/dac01}"
APPROACH_PYTHON="${APPROACH_PYTHON:-$VENV/bin/python}"
RUN_USER="${RUN_USER:-unitree}"
die() { echo "install_assistant: $*" >&2; exit 1; }
need_root() { [[ "$EUID" -eq 0 ]] || die "$1 requires sudo"; }
disarmed() {
    grep -qx 'high_armed=0' /run/hb/status.env 2>/dev/null || die "owner must be DISARMED and the robot supported per SOP"
}

verify_model() {  # signature.json pins dac.onnx; engine.json pins the Jetson-built dac.engine
    python3 - "$MODEL" <<'PY'
import hashlib, json, sys
from pathlib import Path
p = Path(sys.argv[1])
signature = json.loads((p / "signature.json").read_text())
assert hashlib.sha256((p / "dac.onnx").read_bytes()).hexdigest() == signature["onnx_sha256"], "dac.onnx hash differs"
engine = json.loads((p / "engine.json").read_text())
assert hashlib.sha256((p / "dac.engine").read_bytes()).hexdigest() == engine["sha256"], "dac.engine hash differs"
PY
}

# The vision/look-at pipeline refuses a policy that differs from the one it was pinned to. HB's selected walk_policy
# (controller/config/locomotion.yaml) can change with any ./hb deploy, so pin to whatever this robot's HB selects.
pin_policy() {  # pin_policy [--verify]
    python3 - "$HB_ROOT" "$APP/app" "${1:-}" <<'PY'
import hashlib, json, os, sys
from pathlib import Path
hb, app, mode = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]

def values(path):
    out = {}
    for line in path.read_text().splitlines():
        key, sep, value = line.split("#", 1)[0].partition(":")
        if sep:
            out[key.strip()] = value.strip().strip("\"'")
    return out

selected = values(hb / "controller/config/locomotion.yaml")["walk_policy"]
bundle = hb / "controller/policies/locomotion" / selected
meta = values(bundle / "policy.yaml")
digest = hashlib.sha256((bundle / "policy.onnx").read_bytes()).hexdigest()
if digest != meta.get("sha256") or not meta.get("contract"):
    sys.exit("walk policy %s: policy.onnx hash/contract do not match its policy.yaml" % selected)
pin = {"policy_bundle": selected, "policy_contract": meta["contract"], "policy_sha256": digest}
stale = []
for name in ("experiments/r1_target_control/config.json", "agent/target_control/config.json"):
    path = app / name
    config = json.loads(path.read_text())
    if any(config.get(k) != v for k, v in pin.items()):
        stale.append(name)
        if mode != "--verify":
            config.update(pin)
            tmp = path.with_suffix(".json.tmp")
            with open(tmp, "w") as handle:
                json.dump(config, handle, indent=2, ensure_ascii=False); handle.write("\n"); handle.flush(); os.fsync(handle.fileno())
            os.replace(tmp, path)
if mode == "--verify" and stale:
    sys.exit("vision config pinned to a different walk policy than this HB selects (%s): run install_assistant.sh pin" % selected)
print("policy pin: %s%s" % (selected, " (updated)" if stale and mode != "--verify" else ""))
PY
}

prepare() {
    [[ "$EUID" -ne 0 ]] || die "run prepare as the robot user, not root"
    [[ -f "$APP/EXPORT.json" ]] || die "assistant/ bundle missing; run ./hb deploy assistant first"
    if [[ ! -x "$VENV/bin/python" ]]; then
        # Jetson system packages (NumPy, OpenCV, TensorRT) are used on purpose; the system pip is not touched.
        python3 -m venv --system-site-packages "$VENV"
    fi
    "$VENV/bin/python" -c 'import numpy, cv2, tensorrt' || die "numpy/cv2/tensorrt not importable from $VENV"
    if [[ ! -f "$MODEL/dac.engine" ]]; then
        [[ -d "$LEGACY_MODEL" ]] || die "depth model missing: put dac01 (dac.onnx, signature.json, dac.engine, engine.json) in $MODEL, or build it with build_engine.sh"
        install -d "$(dirname "$MODEL")"
        cp -a "$LEGACY_MODEL" "$MODEL"      # copy only; the old directory is never modified
    fi
    verify_model || die "depth model verification failed in $MODEL"
    echo "prepare OK: $VENV and $MODEL"
}

check() {
    [[ -f "$APP/EXPORT.json" && -f "$APP/bin/r1-assistant" ]] || die "assistant/ bundle missing; export it from VLA"
    [[ -x "$APPROACH_PYTHON" ]] || die "approach Python missing: $APPROACH_PYTHON (run: bash install_assistant.sh prepare)"
    [[ -d "$MODEL" ]] && verify_model || die "depth model missing/invalid in $MODEL (run: bash install_assistant.sh prepare)"
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$APP/app:$APP/app/experiments/r1_object_grounding/p6_robot_local" "$APPROACH_PYTHON" -c \
        'import cv2, numpy, dac_runtime; from ai_modules.vision.feature_target import FeatureTarget' \
        || die "perception modules do not import with $APPROACH_PYTHON"
    pin_policy --verify || die "policy pin out of date"
    # The owner build must contain the approach receiver (HB develop controller has it; an old build does not).
    grep -aq R1_APPROACH_ALLOW_NONZERO "$HB_ROOT/controller/build/run_r1" 2>/dev/null \
        || die "controller/build/run_r1 lacks the approach receiver; run ./hb deploy controller from develop"
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
    if [[ -e /etc/r1-assistant/enabled || -e /etc/systemd/system/hb_voice.service.d/r1-assistant.conf ]] \
       || systemctl is-active --quiet hb_r1_assistant 2>/dev/null || systemctl is-enabled --quiet hb_r1_assistant 2>/dev/null; then
        die "the experimental r1_assistant stack is installed; run its rollback first (r1_assistant/releases/*/bin/r1-assistant rollback --activation \$(cat /etc/r1-assistant/enabled))"
    fi
    systemctl is-active --quiet r1-p5-agent || die "r1-p5-agent is not running: select search mode first (sudo bash $HB_ROOT/object_search/runtime/runtime_mode.sh search)"
    pin_policy
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
    prepare) prepare ;;
    pin) [[ "$EUID" -ne 0 ]] || die "run pin as the robot user, not root"; pin_policy ;;
    *) die "usage: install_assistant.sh prepare | pin | check | enable --operator-ready | disable" ;;
esac
