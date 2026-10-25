#!/usr/bin/env bash
# Install/enable the robot-local WebXR runtime service.
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
START=0
for arg in "$@"; do
    case "$arg" in
        --start) START=1 ;;
        *) echo "Usage: $0 [--start]" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
RUN_USER="${HB_RUN_USER:-${SUDO_USER:-$(stat -c %U "$HB_ROOT")}}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
# Teleop restarts without waiting for DISARMED: it never writes rt/lowcmd. When
# its loopback stream stops, run_r1 drops teleop authority after
# teleop_timeout_ms (arms return to their base posture, as on a deadman
# release), and the new sidecar sends only STOP until the trigger is released
# and pressed again.
warn_if_armed() {
    if systemctl is-active --quiet hb_high_level.service &&
       ! grep -qx 'high_armed=0' /run/hb/status.env 2>/dev/null; then
        echo "[TELEOP] WARN high-level đang ARMED: tay sẽ về tư thế nền như khi nhả deadman; bóp lại cò sau khi runtime lên." >&2
    fi
}

RUNTIME_ENV_DIR=/etc/hb/teleop
RUNTIME_ENV="$RUNTIME_ENV_DIR/runtime.env"
install -d -m 0755 "$RUNTIME_ENV_DIR"

CONFIG="$HB_ROOT/teleop/config/teleop.yaml"
LOADER="$HB_ROOT/teleop/scripts/load_teleop_config.py"
if [[ -r "$CONFIG" && -r "$LOADER" ]]; then
    # YAML is the operator-facing source of truth; shell variables below are
    # only deployment-time overrides for a different robot/user.
    config_assignments="$(python3 "$LOADER" --config "$CONFIG" --shell)"
    eval "$config_assignments"
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

# Quest resolves the robot with mDNS.  Configure the Avahi host label rather
# than changing the operating-system hostname used by other HB services.
HOSTNAME_ALIAS="${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}"
HOST_INTERFACE="${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}"
HOST_LABEL="${HOSTNAME_ALIAS%.local}"
[[ "$HOSTNAME_ALIAS" == *.local && "$HOST_LABEL" =~ ^[a-z0-9][a-z0-9-]*$ ]] || {
    echo "[INSTALL] invalid robot-runtime hostname: $HOSTNAME_ALIAS" >&2
    exit 2
}
[[ "$HOST_INTERFACE" =~ ^[[:alnum:]_.:-]{1,15}$ ]] || {
    echo "[INSTALL] invalid robot-runtime host interface: $HOST_INTERFACE" >&2
    exit 2
}
if ! command -v avahi-daemon >/dev/null 2>&1 || ! command -v avahi-resolve-host-name >/dev/null 2>&1; then
    export DEBIAN_FRONTEND=noninteractive
    apt-get update
    apt-get install -y avahi-daemon avahi-utils libnss-mdns
fi
AVAHI_CONF=/etc/avahi/avahi-daemon.conf
[[ -r "$AVAHI_CONF" ]] || { echo "[INSTALL] missing $AVAHI_CONF" >&2; exit 1; }
sed -i -E "s|^#?host-name=.*$|host-name=$HOST_LABEL|" "$AVAHI_CONF"
if grep -Eq '^#?allow-interfaces=' "$AVAHI_CONF"; then
    sed -i -E "s|^#?allow-interfaces=.*$|allow-interfaces=$HOST_INTERFACE|" "$AVAHI_CONF"
else
    sed -i "/^\[server\]/a allow-interfaces=$HOST_INTERFACE" "$AVAHI_CONF"
fi
systemctl enable avahi-daemon.service
systemctl restart avahi-daemon.service

# A robot with UFW enabled otherwise looks healthy locally while Quest cannot
# reach WebXR or receive its mDNS reply.  Do nothing when UFW is absent or
# inactive, so other firewall managers stay under their existing ownership.
if command -v ufw >/dev/null 2>&1 && ufw status 2>/dev/null | grep -q '^Status: active$'; then
    ufw allow "${HB_TELEOP_RUNTIME_PORT:-8012}/tcp"
    ufw allow 5353/udp
    if [[ "${HB_TELEOP_RUNTIME_CAMERA_ENABLED:-1}" == "1" ]]; then
        # ICE media uses ephemeral UDP sockets, scoped to the Quest network.
        quest_cidr="$(ip -4 -o addr show dev "$HOST_INTERFACE" scope global | awk 'NR == 1 {print $4}')"
        [[ -n "$quest_cidr" ]] || { echo "[INSTALL] no Quest subnet on $HOST_INTERFACE" >&2; exit 2; }
        quest_subnet="$(python3 -c 'import ipaddress,sys; print(ipaddress.ip_interface(sys.argv[1]).network)' "$quest_cidr")"
        read -r udp_first udp_last < /proc/sys/net/ipv4/ip_local_port_range
        ufw allow in on "$HOST_INTERFACE" from "$quest_subnet" to any port "$udp_first:$udp_last" proto udp
        echo "[INSTALL] WebRTC UDP $udp_first:$udp_last allowed on $HOST_INTERFACE from $quest_subnet"
    fi
fi

cat >"$RUNTIME_ENV" <<EOF
HB_ROOT=$HB_ROOT
HB_RUN_USER=$RUN_USER
HB_TELEOP_RUNTIME_PYTHON=$RUNTIME_PYTHON
HB_TELEOP_RUNTIME_ENV=hb_teleop
HB_TELEOP_RUNTIME_IK_BACKEND=${HB_TELEOP_RUNTIME_IK_BACKEND:-vendor_r1_a5}
HB_TELEOP_RUNTIME_ELBOW_MODE=${HB_TELEOP_RUNTIME_ELBOW_MODE:-vendor}
HB_TELEOP_RUNTIME_HOST_INTERFACE=${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}
HB_TELEOP_RUNTIME_HOST_IP=auto
HB_TELEOP_RUNTIME_HOSTNAME=${HB_TELEOP_RUNTIME_HOSTNAME:-hb-r1.local}
HB_TELEOP_RUNTIME_PORT=${HB_TELEOP_RUNTIME_PORT:-8012}
HB_TELEOP_RUNTIME_CERT_FILE=${HB_TELEOP_RUNTIME_CERT_FILE:-$RUNTIME_ENV_DIR/cert.pem}
HB_TELEOP_RUNTIME_KEY_FILE=${HB_TELEOP_RUNTIME_KEY_FILE:-$RUNTIME_ENV_DIR/key.pem}
HB_TELEOP_RUNTIME_LOG_DIR=$HB_ROOT/teleop/logs/robot_runtime
HB_TELEOP_ROBOT_INTERFACE=${HB_TELEOP_ROBOT_INTERFACE:-eth10}
HB_TELEOP_RUNTIME_DURATION_S=${HB_TELEOP_RUNTIME_DURATION_S:-0}
HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S=${HB_TELEOP_RUNTIME_FIRST_INPUT_TIMEOUT_S:-0}
HB_TELEOP_RUNTIME_INITIAL_STATE_TIMEOUT_S=${HB_TELEOP_RUNTIME_INITIAL_STATE_TIMEOUT_S:-0}
HB_TELEOP_RUNTIME_BRIDGE_HZ=${HB_TELEOP_RUNTIME_BRIDGE_HZ:-30}
HB_TELEOP_RUNTIME_CONTROL_HZ=${HB_TELEOP_RUNTIME_CONTROL_HZ:-30}
HB_TELEOP_RUNTIME_SEND_HZ=${HB_TELEOP_RUNTIME_SEND_HZ:-100}
HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S=${HB_TELEOP_RUNTIME_SOURCE_TIMEOUT_S:-0.60}
HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S=${HB_TELEOP_RUNTIME_RELEASE_DEBOUNCE_S:-0}
HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY=${HB_TELEOP_RUNTIME_JOINT_LIMITS_ONLY:-1}
HB_TELEOP_RUNTIME_MAX_OFFSET_RAD=${HB_TELEOP_RUNTIME_MAX_OFFSET_RAD:-0.15}
HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD=${HB_TELEOP_RUNTIME_HEAD_YAW_MAX_RAD:-0.65}
HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD=${HB_TELEOP_RUNTIME_HEAD_PITCH_MAX_RAD:-0.70}
HB_TELEOP_RUNTIME_CPU_AFFINITY=${HB_TELEOP_RUNTIME_CPU_AFFINITY:-auto}
HB_TELEOP_RUNTIME_CAMERA_ENABLED=${HB_TELEOP_RUNTIME_CAMERA_ENABLED:-1}
HB_TELEOP_RUNTIME_CAMERA_INTERFACE=${HB_TELEOP_RUNTIME_CAMERA_INTERFACE:-eth10}
HB_TELEOP_RUNTIME_CAMERA_CAPTURE_HZ=${HB_TELEOP_RUNTIME_CAMERA_CAPTURE_HZ:-0}
HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT=${HB_TELEOP_RUNTIME_CAMERA_MEDIA_PORT:-8013}
HB_TELEOP_RUNTIME_CAMERA_WIDTH=${HB_TELEOP_RUNTIME_CAMERA_WIDTH:-640}
HB_TELEOP_RUNTIME_CAMERA_HEIGHT=${HB_TELEOP_RUNTIME_CAMERA_HEIGHT:-360}
HB_TELEOP_RUNTIME_CAMERA_STARTUP_TIMEOUT_S=${HB_TELEOP_RUNTIME_CAMERA_STARTUP_TIMEOUT_S:-5.0}
HB_TELEOP_RUNTIME_CAMERA_ENABLE_MAX_AGE_S=${HB_TELEOP_RUNTIME_CAMERA_ENABLE_MAX_AGE_S:-0.50}
HB_TELEOP_RUNTIME_CAMERA_FALLBACK_MAX_AGE_S=${HB_TELEOP_RUNTIME_CAMERA_FALLBACK_MAX_AGE_S:-1.00}
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

RUNTIME_IP="$(ip -4 -o addr show dev "${HB_TELEOP_RUNTIME_HOST_INTERFACE:-wlan0}" scope global | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
if ! timeout 5 avahi-resolve-host-name -4 "$HOSTNAME_ALIAS" 2>/dev/null | awk -v ip="$RUNTIME_IP" '$2 == ip { found=1 } END { exit !found }'; then
    echo "[INSTALL] mDNS did not publish $HOSTNAME_ALIAS at ${RUNTIME_IP:-missing}" >&2
    exit 1
fi

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
    warn_if_armed
    systemctl start hb_teleop_cert.service
    systemctl restart hb_teleop_runtime.service || {
        rc=$?
        echo "[INSTALL] ERROR restarting hb_teleop_runtime.service; diagnostics follow:" >&2
        systemctl --no-pager -l status hb_teleop_runtime.service >&2 || true
        journalctl -u hb_teleop_runtime.service -b -n 60 --no-pager -o short-iso >&2 || true
        exit "$rc"
    }
fi
echo "[INSTALL] robot-local vendor IK teleop installed for $RUN_USER; start=$START"
