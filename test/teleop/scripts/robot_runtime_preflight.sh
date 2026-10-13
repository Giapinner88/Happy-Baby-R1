#!/usr/bin/env bash
# Read-only preflight for the robot-local WebXR runtime.
set -euo pipefail

PYTHON="${HB_TELEOP_RUNTIME_PYTHON:-$HOME/.local/share/hb/teleop/miniforge3/envs/hb_teleop/bin/python}"
HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}"
CERT_FILE="${HB_TELEOP_RUNTIME_CERT_FILE:-/etc/hb/teleop/cert.pem}"
KEY_FILE="${HB_TELEOP_RUNTIME_KEY_FILE:-/etc/hb/teleop/key.pem}"
ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
HOSTNAME_ALIAS="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
CAMERA_ENABLED="${HB_TELEOP_RUNTIME_CAMERA_ENABLED:-1}"
CAMERA_INTERFACE="${HB_TELEOP_RUNTIME_CAMERA_INTERFACE:-eth10}"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    GREEN=$'\033[32m'; RED=$'\033[31m'; RESET=$'\033[0m'
else
    GREEN=; RED=; RESET=
fi

fail=0
camera_degraded=0
ok() { printf '%s[RUNTIME PREFLIGHT] OK%s %s\n' "$GREEN" "$RESET" "$*"; }
bad() { printf '%s[RUNTIME PREFLIGHT] FAIL%s %s\n' "$RED" "$RESET" "$*" >&2; fail=1; }
degraded() { printf '%s[RUNTIME PREFLIGHT] DEGRADED%s %s\n' "$RED" "$RESET" "$*" >&2; camera_degraded=1; }

# Check dependencies inside this service so transient boot failures reach its
# Restart=on-failure policy, rather than leaving a cancelled dependency job.
for dependency in hb_high_level.service hb_teleop_cert.service; do
    if systemctl is-active --quiet "$dependency"; then
        ok "dependency=$dependency"
    else
        bad "dependency=$dependency not ready; waiting for automatic recovery"
    fi
done
(( fail == 0 )) || exit 1

[[ "$(uname -m)" == "aarch64" ]] && ok "architecture=aarch64" || bad "robot must be aarch64"
export XDG_CACHE_HOME="${HB_TELEOP_RUNTIME_LOG_DIR:-$ROOT/teleop/logs/robot_runtime}/.cache"
export MPLCONFIGDIR="$XDG_CACHE_HOME/matplotlib"
if mkdir -p "$MPLCONFIGDIR" && [[ -w "$MPLCONFIGDIR" ]]; then
    ok "log/import cache writable=$MPLCONFIGDIR"
else
    bad "log/import cache path is not writable: $MPLCONFIGDIR"
fi
(( fail == 0 )) || exit 1

[[ -x "$PYTHON" ]] && ok "python=$PYTHON" || bad "missing Python=$PYTHON"
if [[ -x "$PYTHON" ]]; then
    PYTHONPATH="$ROOT/teleop/src:$ROOT/teleop/third_party/xr_teleoperate/teleop/televuer/src:/home/unitree/Foundation_Unitree/unitree_sdk2_python"
    if envelope="$(PYTHONPATH="$PYTHONPATH" "$PYTHON" - "$ROOT/controller/config/tuning.yaml" <<'PY'
import sys
from pathlib import Path
from teleop.hardware.high_level_sidecar import receiver_head_limits
yaw, pitch = receiver_head_limits(Path(sys.argv[1]))
print(f'yaw={yaw:g} pitch={pitch:g}')
PY
    )"; then
        ok "H4 receiver head envelope $envelope (always enforced)"
    else
        bad "cannot read high-level head receiver envelope"
    fi
    if PYTHONPATH="$PYTHONPATH" "$PYTHON" -c 'import numpy, aiohttp, vuer, televuer, casadi, pinocchio, unitree_sdk2py' >/dev/null 2>&1; then
        ok "WebXR, vendor IK, NumPy and SDK imports"
    else
        bad "WebXR/SDK Python imports"
    fi
    if [[ "$CAMERA_ENABLED" == "1" ]]; then
        if "$PYTHON" - <<'PY' >/dev/null 2>&1
from importlib.metadata import version
assert version("aiortc") == "1.14.0"
from aiortc.codecs.vpx import Vp8Encoder
from vuer.schemas import WebRTCVideoPlane
from av import VideoFrame
from fractions import Fraction
encoder = Vp8Encoder()
frame = VideoFrame(640, 360, "yuv420p")
frame.pts, frame.time_base = 0, Fraction(1, 90000)
assert encoder.encode(frame)[0]
PY
        then
            ok "pinned WebRTC + VP8 encoder + Quest scene"
        else
            bad "WebRTC/VP8 dependencies; rerun bootstrap_robot_runtime.sh"
        fi
        if [[ "${HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT:-8013}" == "${HB_TELEOP_RUNTIME_PORT:-8012}" ]]; then
            bad "camera_media_port must differ from WebXR port"
        elif ss -H -ltn "sport = :${HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT:-8013}" | grep -q .; then
            bad "camera media port occupied before runtime start"
        fi
        if camera_probe="$(PYTHONPATH="$PYTHONPATH" timeout 8s "$PYTHON" \
            "$ROOT/teleop/scripts/teleop/probe_robot_camera.py" "$CAMERA_INTERFACE" 2>&1)"; then
            ok "R1 videohub camera read on $CAMERA_INTERFACE: $camera_probe"
        else
            degraded "Camera robot unavailable on $CAMERA_INTERFACE: ${camera_probe:-probe timeout}; camera robot cannot be used, arm/head teleop remains available in Quest passthrough"
        fi
    elif [[ "$CAMERA_ENABLED" != "0" ]]; then
        bad "camera_enabled must be 0 or 1"
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
if systemctl is-active --quiet avahi-daemon.service; then
    runtime_ip="$(ip -4 -o addr show dev "$HOST_INTERFACE" scope global | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
    if command -v avahi-resolve-host-name >/dev/null 2>&1 && \
       avahi-resolve-host-name -4 "$HOSTNAME_ALIAS" 2>/dev/null | awk -v ip="$runtime_ip" '$2 == ip { found=1 } END { exit !found }'; then
        ok "mDNS=$HOSTNAME_ALIAS -> $runtime_ip"
    else
        bad "mDNS does not publish $HOSTNAME_ALIAS at current $HOST_INTERFACE address"
    fi
else
    bad "avahi-daemon inactive; Quest cannot use $HOSTNAME_ALIAS"
fi
[[ -f "$ROOT/teleop/src/assets/R1.urdf" ]] && ok "R1 URDF present" || bad "R1 URDF missing"
[[ -f "$ROOT/teleop/scripts/teleop/run_r1_vendor_ik_stream.py" ]] && ok "vendor IK stream present" || bad "vendor IK stream missing"
[[ -f "$ROOT/teleop/third_party/xr_teleoperate_r1_a5_845b25b/teleop/robot_control/robot_arm_ik.py" ]] && ok "pinned vendor R1-A5 source present" || bad "vendor R1-A5 source missing"
if (( fail )); then
    printf '%s[RUNTIME PREFLIGHT] NOT READY%s; service will remain stopped/restarting safely\n' "$RED" "$RESET" >&2
    exit 1
fi
if (( camera_degraded )); then
    printf '%s[RUNTIME PREFLIGHT] READY_WITH_CAMERA_ERROR%s; arm/head teleop available, robot camera unavailable\n' "$RED" "$RESET"
else
    printf '%s[RUNTIME PREFLIGHT] READY%s\n' "$GREEN" "$RESET"
fi
