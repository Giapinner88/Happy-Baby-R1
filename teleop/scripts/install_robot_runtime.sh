#!/usr/bin/env bash
# Install/enable the robot-local WebXR runtime service.
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
START=0
[[ "${1:-}" == "--start" ]] && START=1
[[ $# -le 1 ]] || { echo "Usage: $0 [--start]" >&2; exit 2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
RUN_USER="${HB_RUN_USER:-${SUDO_USER:-$(stat -c %U "$HB_ROOT")}}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
RUNTIME_ENV_DIR=/etc/hb/teleop
RUNTIME_ENV="$RUNTIME_ENV_DIR/runtime.env"
install -d -m 0755 "$RUNTIME_ENV_DIR"

CONFIG="$HB_ROOT/teleop/config/teleop.yaml"
LOADER="$HB_ROOT/teleop/scripts/load_teleop_config.py"
if [[ -r "$CONFIG" && -r "$LOADER" ]]; then
    # YAML is the operator-facing source of truth; shell variables below are
    # only deployment-time overrides for a different robot/user.
    eval "$(python3 "$LOADER" --config "$CONFIG" --shell)"
fi
[[ "${HB_TELEOP_RUNTIME_ENABLED:-1}" == "1" ]] || {
    for unit in hb_teleop_runtime.service hb_teleop_cert.service; do
        if systemctl cat "$unit" >/dev/null 2>&1; then
            systemctl disable --now "$unit"
        fi
    done
    echo "[INSTALL] robot_runtime.enabled=false; services stopped and disabled" >&2
    exit 0
}
RUNTIME_PYTHON="${HB_TELEOP_RUNTIME_PYTHON:-$RUN_HOME/.local/share/hb/teleop/miniforge3/envs/hb_teleop/bin/python}"
install -d -o "$RUN_USER" -g "$RUN_USER" -m 0755 "$HB_ROOT/teleop/logs/robot_runtime"

cat >"$RUNTIME_ENV" <<EOF
HB_ROOT=$HB_ROOT
HB_RUN_USER=$RUN_USER
HB_TELEOP_RUNTIME_PYTHON=$RUNTIME_PYTHON
HB_TELEOP_RUNTIME_ENV=hb_teleop
HB_TELEOP_RUNTIME_IK_BACKEND=${HB_TELEOP_RUNTIME_IK_BACKEND:-vendor_r1_a5}
HB_TELEOP_RUNTIME_HOST_INTERFACE=${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}
HB_TELEOP_RUNTIME_HOST_IP=auto
HB_TELEOP_RUNTIME_HOSTNAME=${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}
HB_TELEOP_RUNTIME_PORT=${HB_TELEOP_RUNTIME_PORT:-8012}
HB_TELEOP_RUNTIME_CERT_FILE=${HB_TELEOP_RUNTIME_CERT_FILE:-$RUNTIME_ENV_DIR/cert.pem}
HB_TELEOP_RUNTIME_KEY_FILE=${HB_TELEOP_RUNTIME_KEY_FILE:-$RUNTIME_ENV_DIR/key.pem}
HB_TELEOP_RUNTIME_LOG_DIR=$HB_ROOT/teleop/logs/robot_runtime
HB_TELEOP_ROBOT_INTERFACE=${HB_TELEOP_ROBOT_INTERFACE:-eth10}
HB_TELEOP_RUNTIME_DURATION_S=${HB_TELEOP_RUNTIME_DURATION_S:-0}
HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S=${HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S:-120}
HB_TELEOP_RUNTIME_BRIDGE_HZ=${HB_TELEOP_RUNTIME_BRIDGE_HZ:-30}
HB_TELEOP_RUNTIME_CONTROL_HZ=${HB_TELEOP_RUNTIME_CONTROL_HZ:-10}
HB_TELEOP_RUNTIME_SEND_HZ=${HB_TELEOP_RUNTIME_SEND_HZ:-100}
HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S=${HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S:-0.60}
HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S=${HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S:-120}
HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY=${HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY:-1}
HB_TELEOP_RUNTIME_MAX_OFFSET_RAD=${HB_TELEOP_RUNTIME_MAX_OFFSET_RAD:-0.15}
HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD=${HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD:-0.65}
HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD=${HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD:-0.70}
HB_TELEOP_RUNTIME_SDK_PYTHONPATH=/home/unitree/Foundation_Unitree/unitree_sdk2_python
EOF
chown root:"$RUN_USER" "$RUNTIME_ENV"
chmod 0640 "$RUNTIME_ENV"

HB_ROOT="$HB_ROOT" HB_RUN_USER="$RUN_USER" \
  HB_TELEOP_RUNTIME_HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}" \
  HB_TELEOP_RUNTIME_HOSTNAME="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}" \
  HB_TELEOP_RUNTIME_CERT_FILE="$RUNTIME_ENV_DIR/cert.pem" \
  HB_TELEOP_RUNTIME_KEY_FILE="$RUNTIME_ENV_DIR/key.pem" \
  bash "$SCRIPT_DIR/refresh_robot_cert.sh"

render() {
    sed -e "s|@HB_ROOT@|$HB_ROOT|g" -e "s|@RUN_USER@|$RUN_USER|g" \
        "$1" >"$2"
    chmod 0644 "$2"
}
render "$HB_ROOT/teleop/systemd/hb_teleop_cert.service.in" /etc/systemd/system/hb_teleop_cert.service
render "$HB_ROOT/teleop/systemd/hb_teleop_runtime.service.in" /etc/systemd/system/hb_teleop_runtime.service
systemctl daemon-reload
systemctl enable hb_teleop_cert.service hb_teleop_runtime.service
if (( START )); then
    systemctl start hb_teleop_cert.service
    systemctl restart hb_teleop_runtime.service
fi
echo "[INSTALL] robot-local vendor IK teleop installed for $RUN_USER; start=$START"
