#!/usr/bin/env bash
# Read-only preflight for the robot-local WebXR runtime.
set -euo pipefail

PYTHON="${HB_TELEOP_RUNTIME_PYTHON:-$HOME/.local/share/hb/teleop/miniforge3/envs/hb_teleop/bin/python}"
HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}"
CERT_FILE="${HB_TELEOP_RUNTIME_CERT_FILE:-/etc/hb/teleop/cert.pem}"
KEY_FILE="${HB_TELEOP_RUNTIME_KEY_FILE:-/etc/hb/teleop/key.pem}"
ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    GREEN=$'\033[32m'; RED=$'\033[31m'; RESET=$'\033[0m'
else
    GREEN=; RED=; RESET=
fi

fail=0
ok() { printf '%s[RUNTIME PREFLIGHT] OK%s %s\n' "$GREEN" "$RESET" "$*"; }
bad() { printf '%s[RUNTIME PREFLIGHT] FAIL%s %s\n' "$RED" "$RESET" "$*" >&2; fail=1; }

[[ "$(uname -m)" == "aarch64" ]] && ok "architecture=aarch64" || bad "robot must be aarch64"
[[ -x "$PYTHON" ]] && ok "python=$PYTHON" || bad "missing Python=$PYTHON"
if [[ -x "$PYTHON" ]]; then
    PYTHONPATH="$ROOT/teleop/src:$ROOT/teleop/third_party/xr_teleoperate/teleop/televuer/src:/home/unitree/Foundation_Unitree/unitree_sdk2_python"
    if PYTHONPATH="$PYTHONPATH" "$PYTHON" -c 'import numpy, aiohttp, vuer, televuer, casadi, pinocchio, unitree_sdk2py' >/dev/null 2>&1; then
        ok "WebXR, vendor IK, NumPy and SDK imports"
    else
        bad "WebXR/SDK Python imports"
    fi
    if [[ -f "$ROOT/teleop/scripts/teleop/run_r1_vendor_ik_stream.py" ]] && \
       timeout 45s bash -c 'PYTHONPATH="$1" "$2" "$3" --stats-path "$(mktemp)" </dev/null >/dev/null 2>&1' _ \
           "$PYTHONPATH" "$PYTHON" "$ROOT/teleop/scripts/teleop/run_r1_vendor_ik_stream.py"; then
        ok "pinned vendor IK load"
    else
        bad "pinned vendor IK load"
    fi
fi
[[ -r "$CERT_FILE" && -r "$KEY_FILE" ]] && ok "cert/key present" || bad "missing cert/key"
ip -4 addr show dev "$HOST_INTERFACE" | grep -q 'inet ' && ok "network=$HOST_INTERFACE" || bad "no IPv4 on $HOST_INTERFACE"
[[ -f "$ROOT/teleop/src/assets/R1.urdf" ]] && ok "R1 URDF present" || bad "R1 URDF missing"
[[ -f "$ROOT/teleop/scripts/teleop/run_r1_vendor_ik_stream.py" ]] && ok "vendor IK stream present" || bad "vendor IK stream missing"
[[ -f "$ROOT/teleop/third_party/xr_teleoperate_r1_a5_845b25b/teleop/robot_control/robot_arm_ik.py" ]] && ok "pinned vendor R1-A5 source present" || bad "vendor R1-A5 source missing"
if (( fail )); then
    printf '%s[RUNTIME PREFLIGHT] NOT READY%s; service will remain stopped/restarting safely\n' "$RED" "$RESET" >&2
    exit 1
fi
printf '%s[RUNTIME PREFLIGHT] READY%s\n' "$GREEN" "$RESET"
