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
FIRST_INPUT_TIMEOUT_S="${HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S:-0}"
INITIAL_STATE_TIMEOUT_S="${HB_TELEOP_RUNTIME_INITIAL_STATE_TIMEOUT_S:-0}"
BRIDGE_HZ="${HB_TELEOP_RUNTIME_BRIDGE_HZ:-30}"
CONTROL_HZ="${HB_TELEOP_RUNTIME_CONTROL_HZ:-30}"
SEND_HZ="${HB_TELEOP_RUNTIME_SEND_HZ:-100}"
SOURCE_TIMEOUT_S="${HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S:-0.60}"
RELEASE_DEBOUNCE_S="${HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S:-0}"
JOINT_LIMITS_ONLY="${HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY:-1}"
MAX_OFFSET_RAD="${HB_TELEOP_RUNTIME_MAX_OFFSET_RAD:-0.15}"
HEAD_YAW_MAX_RAD="${HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD:-0.65}"
HEAD_PITCH_MAX_RAD="${HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD:-0.70}"
CPU_AFFINITY="${HB_TELEOP_RUNTIME_CPU_AFFINITY:-auto}"
LOG_ROOT="${HB_TELEOP_RUNTIME_LOG_DIR:-$ROOT/teleop/logs/robot_runtime}"
SDK_PYTHONPATH="${HB_TELEOP_RUNTIME_SDK_PYTHONPATH:-/home/unitree/Foundation_Unitree/unitree_sdk2_python}"
CAMERA_ENABLED="${HB_TELEOP_RUNTIME_CAMERA_ENABLED:-1}"
CAMERA_INTERFACE="${HB_TELEOP_RUNTIME_CAMERA_INTERFACE:-eth10}"
CAMERA_CAPTURE_HZ="${HB_TELEOP_RUNTIME_CAMERA_CAPTURE_HZ:-0}"
CAMERA_MEDIA_PORT="${HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT:-8013}"
CAMERA_WIDTH="${HB_TELEOP_RUNTIME_CAMERA_WIDTH:-640}"
CAMERA_HEIGHT="${HB_TELEOP_RUNTIME_CAMERA_HEIGHT:-360}"
CAMERA_STARTUP_TIMEOUT_S="${HB_TELEOP_RUNTIME_CAMERA_STARTUP_TIMEOUT_S:-5.0}"
CAMERA_ENABLE_MAX_AGE_S="${HB_TELEOP_RUNTIME_CAMERA_ENABLE_MAX_AGE_S:-0.50}"
CAMERA_FALLBACK_MAX_AGE_S="${HB_TELEOP_RUNTIME_CAMERA_FALLBACK_MAX_AGE_S:-1.00}"

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
[[ "$CAMERA_ENABLED" == "0" || "$CAMERA_ENABLED" == "1" ]] || {
    echo "[RUNTIME] ERROR camera_enabled must be 0 or 1" >&2
    exit 2
}
[[ "$CPU_AFFINITY" == "auto" || "$CPU_AFFINITY" == "off" ]] || {
    echo "[RUNTIME] ERROR cpu_affinity must be auto or off" >&2
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
# systemd ProtectHome makes ~/.config and ~/.cache read-only. Keep import
# caches in the existing allowed log tree instead of recreating them in /tmp.
export MPLCONFIGDIR="$LOG_ROOT/.cache/matplotlib"
export XDG_CACHE_HOME="$LOG_ROOT/.cache"
mkdir -p "$MPLCONFIGDIR"

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
[[ "$CAMERA_ENABLED" == "1" ]] && echo "[RUNTIME] Camera R1 enabled: A phải đổi passthrough/camera"

# The WebXR/Vuer process and CasADi IPOPT solver are CPU-heavy.  With four or
# more online CPUs, pin their process trees apart.  Keep the mapper and
# sidecar together: they are light but latency-sensitive.  On smaller or
# unusual systems, preserving the kernel's default scheduler is safer than
# restricting a solver to too few CPUs.
BRIDGE_CPUSET=""
MAPPER_CPUSET=""
SIDECAR_CPUSET=""
IK_CPUSET=""
CAMERA_CPUSET=""
if [[ "$CPU_AFFINITY" == "auto" ]] && command -v taskset >/dev/null 2>&1 && command -v lscpu >/dev/null 2>&1; then
    mapfile -t ONLINE_CPUS < <(lscpu -p=CPU,ONLINE 2>/dev/null | awk -F, '$1 !~ /^#/ && $2 == "Y" {print $1}')
    if (( ${#ONLINE_CPUS[@]} >= 4 )); then
        BRIDGE_CPUSET="${ONLINE_CPUS[0]}"
        MAPPER_CPUSET="${ONLINE_CPUS[1]}"
        SIDECAR_CPUSET="${ONLINE_CPUS[1]}"
        if (( ${#ONLINE_CPUS[@]} >= 5 )); then
            CAMERA_CPUSET="${ONLINE_CPUS[2]}"
            IK_CPUSET="$(IFS=,; echo "${ONLINE_CPUS[*]:3}")"
        else
            IK_CPUSET="$(IFS=,; echo "${ONLINE_CPUS[*]:2}")"
        fi
    fi
fi
run_on_cpus() {
    local cpuset="$1"
    shift
    if [[ -z "$cpuset" ]]; then
        "$@"
    else
        taskset --cpu-list "$cpuset" "$@"
    fi
}
if [[ -n "$IK_CPUSET" ]]; then
    echo "[RUNTIME] CPU affinity: bridge=$BRIDGE_CPUSET mapper+sidecar=$MAPPER_CPUSET IK=$IK_CPUSET camera=${CAMERA_CPUSET:-kernel-default}"
else
    echo "[RUNTIME] CPU affinity: kernel default (mode=$CPU_AFFINITY)"
fi

# Media has its own process, latest-frame mailbox, encoder and UDP transport.
CAMERA_PID=""
cleanup_media() {
    if [[ -n "$CAMERA_PID" ]]; then
        kill "$CAMERA_PID" 2>/dev/null || true
        wait "$CAMERA_PID" 2>/dev/null || true
    fi
}
trap cleanup_media EXIT
if [[ "$CAMERA_ENABLED" == "1" ]]; then
    MEDIA_PREFIX=()
    [[ -n "$CAMERA_CPUSET" ]] && MEDIA_PREFIX=(taskset --cpu-list "$CAMERA_CPUSET")
    "${MEDIA_PREFIX[@]}" "$PYTHON" teleop/scripts/teleop/robot_camera_webrtc.py \
        --host-ip "$HOST_IP" --port "$CAMERA_MEDIA_PORT" --interface "$CAMERA_INTERFACE" \
        --capture-hz "$CAMERA_CAPTURE_HZ" --width "$CAMERA_WIDTH" --height "$CAMERA_HEIGHT" \
        --stale-s "$CAMERA_FALLBACK_MAX_AGE_S" --log "$RUN_DIR/camera_webrtc.jsonl" \
        >"$RUN_DIR/camera_webrtc.log" 2>&1 &
    CAMERA_PID=$!
    ready=0
    for attempt in {1..50}; do
        if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
            cat "$RUN_DIR/camera_webrtc.log" >&2
            exit 2
        fi
        if curl -fsS --max-time 0.3 "http://127.0.0.1:$CAMERA_MEDIA_PORT/health" >/dev/null 2>&1; then
            ready=1
            break
        fi
        sleep 0.1
    done
    (( ready )) || { echo "[RUNTIME] ERROR WebRTC media not ready" >&2; exit 2; }
    echo "[RUNTIME] Camera transport: WebRTC VP8; capture_hz=$CAMERA_CAPTURE_HZ (0=auto); same HTTPS origin"
fi

set +e
LIMIT_ARGS=()
[[ "$JOINT_LIMITS_ONLY" == "1" ]] && LIMIT_ARGS+=(--joint-limits-only)
CAMERA_ARGS=()
if [[ "$CAMERA_ENABLED" == "1" ]]; then
    CAMERA_ARGS=(--robot-camera --robot-camera-media-port "$CAMERA_MEDIA_PORT" --robot-camera-interface "$CAMERA_INTERFACE"
        --robot-camera-capture-hz "$CAMERA_CAPTURE_HZ" --robot-camera-width "$CAMERA_WIDTH"
        --robot-camera-height "$CAMERA_HEIGHT" --camera-startup-timeout-s "$CAMERA_STARTUP_TIMEOUT_S" --camera-enable-max-age-s "$CAMERA_ENABLE_MAX_AGE_S"
        --camera-fallback-max-age-s "$CAMERA_FALLBACK_MAX_AGE_S")
fi
run_on_cpus "$BRIDGE_CPUSET" "$PYTHON" teleop/scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" --port "$PORT" --duration-s "$DURATION_S" \
    --frequency-hz "$BRIDGE_HZ" --deadman-source right_trigger \
    --trigger-value-threshold 5.0 --cert-file "$CERT_FILE" --key-file "$KEY_FILE" \
    "${CAMERA_ARGS[@]}" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
    2> >(tee "$RUN_DIR/bridge.log" >&2) \
| "$PYTHON" -m teleop.hardware.stream_io --tee-log "$RUN_DIR/quest_commands.jsonl" \
| run_on_cpus "$IK_CPUSET" "$PYTHON" teleop/scripts/teleop/run_r1_vendor_ik_stream.py \
    --passthrough --wrist-reference body_relative --arm-filter responsive \
    --stats-path "$RUN_DIR/vendor_ik_stats.json" \
    2> >(tee "$RUN_DIR/vendor_ik.log" >&2) \
| run_on_cpus "$MAPPER_CPUSET" "$PYTHON" teleop/scripts/teleop/run_r1_vendor_targets.py \
    --duration-s "$DURATION_S" --control-hz "$CONTROL_HZ" \
    --source-timeout-s "$SOURCE_TIMEOUT_S" --stream-contract h4 \
    --release-debounce-s "$RELEASE_DEBOUNCE_S" \
    "${LIMIT_ARGS[@]}" \
    --diagnostics-log "$RUN_DIR/vendor_targets_diagnostics.jsonl" \
    2> >(tee "$RUN_DIR/vendor_targets.log" >&2) \
| "$PYTHON" -m teleop.hardware.stream_io --tee-log "$RUN_DIR/targets.jsonl" \
| HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 run_on_cpus "$SIDECAR_CPUSET" "$PYTHON" -m teleop.hardware.high_level_sidecar \
    --stream-contract h4 --interface "$ROBOT_INTERFACE" --udp-host 127.0.0.1 \
    --udp-port 5560 --confirm-suspended-with-estop --confirm-dev-mode \
    --duration-s "$DURATION_S" --initial-state-timeout-s "$INITIAL_STATE_TIMEOUT_S" \
    --first-input-timeout-s "$FIRST_INPUT_TIMEOUT_S" \
    --input-timeout-s 0.75 --source-timeout-s "$SOURCE_TIMEOUT_S" --state-timeout-s 0.20 \
    --send-hz "$SEND_HZ" --max-offset-rad "$MAX_OFFSET_RAD" \
    --head-yaw-max-rad "$HEAD_YAW_MAX_RAD" --head-pitch-max-rad "$HEAD_PITCH_MAX_RAD" \
    --receiver-config "$ROOT/controller/config/tuning.yaml" \
    --expected-mode-machine 1 --log-dir "$LOG_ROOT" "${LIMIT_ARGS[@]}" \
    2> >(tee "$RUN_DIR/sidecar.log" >&2) \
| tee "$RUN_DIR/sidecar_output.log"
status=("${PIPESTATUS[@]}")
printf '%s\n' "${status[*]}" > "$RUN_DIR/pipeline_exit_codes.txt"
echo "[RUNTIME] pipeline exit codes: ${status[*]}" >&2
for code in "${status[@]}"; do (( code == 0 )) || exit "$code"; done
