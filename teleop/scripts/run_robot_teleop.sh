#!/usr/bin/env bash
# Robot-local Quest -> IK -> high-level sidecar runtime.
#
# This is deliberately a local pipeline: no SSH hop and no laptop process is
# involved after the service starts.  hb_high_level remains the only motor
# publisher; this process can only send its loopback UDP teleop packet.
set -euo pipefail

ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
cd "$ROOT"

PYTHON="${HB_TELEOP_RUNTIME_PYTHON:-$HOME/.local/share/hb/teleop/miniforge3/envs/hb_teleop/bin/python}"
IK_BACKEND="${HB_TELEOP_RUNTIME_IK_BACKEND:-vendor_r1_a5}"
HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}"
HOST_IP="${HB_TELEOP_RUNTIME_HOST_IP:-auto}"
HOSTNAME_ALIAS="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
PORT="${HB_TELEOP_RUNTIME_PORT:-8012}"
CERT_FILE="${HB_TELEOP_RUNTIME_CERT_FILE:-/etc/hb/teleop/cert.pem}"
KEY_FILE="${HB_TELEOP_RUNTIME_KEY_FILE:-/etc/hb/teleop/key.pem}"
ROBOT_INTERFACE="${HB_TELEOP_ROBOT_INTERFACE:-eth10}"
DURATION_S="${HB_TELEOP_RUNTIME_DURATION_S:-0}"
FIRST_INPUT_TIMEOUT_S="${HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S:-120}"
BRIDGE_HZ="${HB_TELEOP_RUNTIME_BRIDGE_HZ:-30}"
CONTROL_HZ="${HB_TELEOP_RUNTIME_CONTROL_HZ:-10}"
SEND_HZ="${HB_TELEOP_RUNTIME_SEND_HZ:-100}"
SOURCE_TIMEOUT_S="${HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S:-0.60}"
RELEASE_DEBOUNCE_S="${HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S:-120}"
JOINT_LIMITS_ONLY="${HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY:-1}"
MAX_OFFSET_RAD="${HB_TELEOP_RUNTIME_MAX_OFFSET_RAD:-0.15}"
HEAD_YAW_MAX_RAD="${HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD:-0.65}"
HEAD_PITCH_MAX_RAD="${HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD:-0.70}"
LOG_ROOT="${HB_TELEOP_RUNTIME_LOG_DIR:-$ROOT/teleop/logs/robot_runtime}"
SDK_PYTHONPATH="${HB_TELEOP_RUNTIME_SDK_PYTHONPATH:-/home/unitree/Foundation_Unitree/unitree_sdk2_python}"

if [[ "$HOST_IP" == "auto" || -z "$HOST_IP" ]]; then
    HOST_IP="$(ip -4 -o addr show dev "$HOST_INTERFACE" scope global 2>/dev/null \
        | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
fi
[[ -n "$HOST_IP" ]] || { echo "[RUNTIME] ERROR no IPv4 on $HOST_INTERFACE" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "[RUNTIME] ERROR Python missing: $PYTHON" >&2; exit 2; }
[[ "$JOINT_LIMITS_ONLY" == "0" || "$JOINT_LIMITS_ONLY" == "1" ]] || {
    echo "[RUNTIME] ERROR joint_limits_only must be 0 or 1" >&2
    exit 2
}
[[ -r "$CERT_FILE" && -r "$KEY_FILE" ]] || {
    echo "[RUNTIME] ERROR missing cert/key: $CERT_FILE $KEY_FILE" >&2; exit 2;
}
if ss -H -ltn "sport = :$PORT" 2>/dev/null | grep -q .; then
    echo "[RUNTIME] ERROR port $PORT is already occupied" >&2
    exit 2
fi

export PYTHONNOUSERSITE=0
export PYTHONPATH="$ROOT/teleop/src:$ROOT/teleop/third_party/xr_teleoperate/teleop/televuer/src:$SDK_PYTHONPATH${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$LOG_ROOT"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_robot_quest3"
RUN_DIR="$LOG_ROOT/$RUN_ID"
mkdir -p "$RUN_DIR"

"$PYTHON" - <<'PY'
import sys
import numpy
import casadi, pinocchio
from televuer import TeleVuerWrapper
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
print(f"[RUNTIME] Python {sys.version.split()[0]} numpy={numpy.__version__}", file=sys.stderr)
print(f"[RUNTIME] WebXR + vendor IK + SDK imports OK casadi={casadi.__version__}", file=sys.stderr)
PY

[[ "$IK_BACKEND" == "vendor_r1_a5" ]] || { echo "[RUNTIME] ERROR only vendor_r1_a5 is allowed" >&2; exit 2; }
echo "[RUNTIME] Quest URL: https://$HOST_IP:$PORT/?ws=wss://$HOST_IP:$PORT"
echo "[RUNTIME] Hostname URL (if mDNS/DHCP reservation resolves): https://$HOSTNAME_ALIAS:$PORT/?ws=wss://$HOSTNAME_ALIAS:$PORT"
echo "[RUNTIME] H4 IK: vendor_r1_a5 (CasADi/Pinocchio, ARM64); START gates high-level authority"
echo "[RUNTIME] Target limit mode: $(if [[ "$JOINT_LIMITS_ONLY" == "1" ]]; then echo joint-limits-only; else echo velocity-and-acceleration-limited; fi)"
echo "[RUNTIME] Robot-local pipeline; laptop is not required after this service starts"

set +e
LIMIT_ARGS=()
[[ "$JOINT_LIMITS_ONLY" == "1" ]] && LIMIT_ARGS+=(--joint-limits-only)
"$PYTHON" teleop/scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" --port "$PORT" --duration-s "$DURATION_S" \
    --frequency-hz "$BRIDGE_HZ" --deadman-source right_trigger \
    --trigger-value-threshold 5.0 --cert-file "$CERT_FILE" --key-file "$KEY_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
    2> >(tee "$RUN_DIR/bridge.log" >&2) \
| tee "$RUN_DIR/quest_commands.jsonl" \
| "$PYTHON" teleop/scripts/teleop/run_r1_vendor_ik_stream.py \
    --passthrough --stats-path "$RUN_DIR/vendor_ik_stats.json" \
    2> >(tee "$RUN_DIR/vendor_ik.log" >&2) \
| "$PYTHON" teleop/scripts/teleop/run_r1_vendor_targets.py \
    --duration-s "$DURATION_S" --control-hz "$CONTROL_HZ" \
    --source-timeout-s "$SOURCE_TIMEOUT_S" --stream-contract h4 \
    --release-debounce-s "$RELEASE_DEBOUNCE_S" \
    "${LIMIT_ARGS[@]}" \
    --diagnostics-log "$RUN_DIR/vendor_targets_diagnostics.jsonl" \
    2> >(tee "$RUN_DIR/vendor_targets.log" >&2) \
| tee "$RUN_DIR/targets.jsonl" \
| HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 "$PYTHON" -m teleop.hardware.high_level_sidecar \
    --stream-contract h4 --interface "$ROBOT_INTERFACE" --udp-host 127.0.0.1 \
    --udp-port 5560 --confirm-suspended-with-estop --confirm-dev-mode \
    --duration-s "$DURATION_S" --first-input-timeout-s "$FIRST_INPUT_TIMEOUT_S" \
    --input-timeout-s 0.75 --source-timeout-s "$SOURCE_TIMEOUT_S" --state-timeout-s 0.20 \
    --send-hz "$SEND_HZ" --max-offset-rad "$MAX_OFFSET_RAD" \
    --head-yaw-max-rad "$HEAD_YAW_MAX_RAD" --head-pitch-max-rad "$HEAD_PITCH_MAX_RAD" \
    --expected-mode-machine 1 --log-dir "$LOG_ROOT" "${LIMIT_ARGS[@]}" \
    2> >(tee "$RUN_DIR/sidecar.log" >&2) \
| tee "$RUN_DIR/sidecar_output.log"
status=("${PIPESTATUS[@]}")
printf '%s\n' "${status[*]}" > "$RUN_DIR/pipeline_exit_codes.txt"
echo "[RUNTIME] pipeline exit codes: ${status[*]}" >&2
for code in "${status[@]}"; do (( code == 0 )) || exit "$code"; done
