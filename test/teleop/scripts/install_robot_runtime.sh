#!/usr/bin/env bash
# Install/enable the robot-local WebXR runtime service.
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
START=0
APPLY_RATE=0
for arg in "$@"; do
    case "$arg" in
        --start) START=1 ;;
        --apply-rate) APPLY_RATE=1 ;;
        *) echo "Usage: $0 [--start] [--apply-rate]" >&2; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
RUN_USER="${HB_RUN_USER:-${SUDO_USER:-$(stat -c %U "$HB_ROOT")}}"
RUN_HOME="$(getent passwd "$RUN_USER" | cut -d: -f6)"
require_rate_disarmed() {
    python3 - "${1:-RATE}" <<'PY'
from pathlib import Path
import time, sys
p = Path('/run/hb/status.env')
values = dict(line.split('=',1) for line in p.read_text().splitlines() if '=' in line)
age = time.monotonic() * 1000 - float(values.get('updated_monotonic_ms', '0'))
if not (values.get('high_armed') == '0' and values.get('high_alive') == '1'
        and values.get('high_busy') == '0' and 0 <= age < 3000):
    raise SystemExit(f'[{sys.argv[1]}] refuse restart: high-level must be DISARMED, idle, with fresh status')
PY
}
(( ! APPLY_RATE )) || require_rate_disarmed
# Restarting the control pipeline must not interrupt an armed robot, even when
# this invocation only updates Python/WebXR and leaves the C++ rate unchanged.
if (( START && ! APPLY_RATE )) && systemctl is-active --quiet hb_high_level.service; then
    require_rate_disarmed TELEOP
fi

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
if (( APPLY_RATE )); then
    # This action is explicitly requested by the root wrapper's `upgrade`.
    # Refuse a restart while high-level has authority or status is stale.
    require_rate_disarmed
    rate_root="$HB_ROOT/controller"
    rate_stage="$rate_root/build_teleop_rate"
    python3 - "$rate_root" "$rate_stage" <<'PY'
from pathlib import Path
import hashlib,json,sys,re
root,stage=map(Path,sys.argv[1:])
m=json.loads((stage/'rate_manifest.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert m['rate']==4.0 and m['candidate_sha256']==sha(stage/'run_r1'), 'rate candidate changed'
assert m['previous_binary_sha256']==sha(root/'build/run_r1'), 'live binary changed since staging; restage'
p=root/'config/gestures.yaml'
s=p.read_text()
assert len(re.findall(r'^teleop_max_rate_rad_s:.*$',s,re.M))==1, 'ambiguous rate config'
(stage/'gestures.yaml.apply').write_text(re.sub(r'^teleop_max_rate_rad_s:.*$', 'teleop_max_rate_rad_s: 4.0',s,flags=re.M))
PY
    rate_backup="$rate_root/rate_backup_$(date -u +%Y%m%dT%H%M%SZ)"
    mkdir -p "$rate_backup"
    cp -a "$rate_root/build/run_r1" "$rate_root/config/gestures.yaml" "$rate_backup/"
    # Recheck immediately before stopping services, after candidate validation.
    grep -qx 'high_armed=0' /run/hb/status.env || { echo '[RATE] robot became ARMED' >&2; exit 2; }
    systemctl stop hb_teleop_runtime.service
    systemctl stop hb_high_level.service
    install -o "$RUN_USER" -g "$RUN_USER" -m 0755 "$rate_stage/run_r1" "$rate_root/build/run_r1.rate-new"
    mv "$rate_root/build/run_r1.rate-new" "$rate_root/build/run_r1"
    install -o "$RUN_USER" -g "$RUN_USER" -m 0644 "$rate_stage/gestures.yaml.apply" "$rate_root/config/gestures.yaml"
    if ! HB_PROJECT_DIR="$rate_root" "$rate_root/build/run_r1" --preflight >"$rate_stage/active_preflight.log" 2>&1; then
        install -o "$RUN_USER" -g "$RUN_USER" -m 0755 "$rate_backup/run_r1" "$rate_root/build/run_r1.rollback"
        mv "$rate_root/build/run_r1.rollback" "$rate_root/build/run_r1"
        cp -a "$rate_backup/gestures.yaml" "$rate_root/config/gestures.yaml"
        systemctl start hb_high_level.service
        echo "[RATE] preflight failed; restored binary/config from $rate_backup" >&2
        tail -60 "$rate_stage/active_preflight.log" >&2
        exit 1
    fi
    rate_started_ms="$(python3 -c 'import time; print(int(time.monotonic()*1000))')"
    systemctl start hb_high_level.service || true
    rate_ready=0
    for _ in {1..40}; do
        if systemctl is-active --quiet hb_high_level.service && grep -qx 'high_alive=1' /run/hb/status.env && grep -qx 'high_armed=0' /run/hb/status.env &&
           python3 - "$rate_started_ms" <<'PY'
from pathlib import Path
import sys,time
v=dict(line.split('=',1) for line in Path('/run/hb/status.env').read_text().splitlines() if '=' in line)
updated=float(v.get('updated_monotonic_ms',0))
sys.exit(0 if updated > float(sys.argv[1]) and 0 <= time.monotonic()*1000-updated < 3000 else 1)
PY
        then
            rate_ready=1
            break
        fi
        sleep 0.5
    done
    if (( ! rate_ready )); then
        systemctl stop hb_high_level.service
        install -o "$RUN_USER" -g "$RUN_USER" -m 0755 "$rate_backup/run_r1" "$rate_root/build/run_r1.rollback"
        mv "$rate_root/build/run_r1.rollback" "$rate_root/build/run_r1"
        cp -a "$rate_backup/gestures.yaml" "$rate_root/config/gestures.yaml"
        systemctl start hb_high_level.service
        echo "[RATE] startup failed; binary/config restored from $rate_backup" >&2
        exit 1
    fi
    echo "[RATE] arms max=4 rad/s, head limit unchanged; backup=$rate_backup"
fi

if (( START )); then
    if systemctl is-active --quiet hb_high_level.service; then
        require_rate_disarmed TELEOP
    fi
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
