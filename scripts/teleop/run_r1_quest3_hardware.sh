#!/usr/bin/env bash
# Foreground Quest -> arms/head IK -> SSH sidecar -> sole high-level lowcmd owner.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

if [[ "${CONFIRM_SUSPENDED_WITH_ESTOP:-0}" != "1" ]]; then
    if [[ ! -t 0 ]]; then
        echo "[FAIL] Cần terminal tương tác hoặc CONFIRM_SUSPENDED_WITH_ESTOP=1." >&2
        exit 2
    fi
    read -r -p "Robot đã treo/cố định, R3 L2+B sẵn sàng, và high-level ở ZERO TORQUE? Nhập YES: " confirmation
    if [[ "$confirmation" != "YES" ]]; then
        echo "[SAFE] Hủy; không khởi động teleop." >&2
        exit 2
    fi
fi

ROBOT="${ROBOT:-unitree@192.168.1.104}"
DURATION_S="${DURATION_S:-120}"
HOST_IP="${HOST_IP:-10.42.0.1}"
CERT_FILE="${CERT_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/cert.pem}"
KEY_FILE="${KEY_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/key.pem}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_r1_quest3_hardware"
RUN_DIR="$ROOT/results/smoke/$RUN_ID"
STOP_FILE="/tmp/$RUN_ID.stop"
mkdir -p "$RUN_DIR"

cleanup() {
    touch "$STOP_FILE"
}
trap cleanup EXIT INT TERM

HB_TELEOP_HOST_IP="$HOST_IP" \
HB_TELEOP_CERT_FILE="$CERT_FILE" \
HB_TELEOP_KEY_FILE="$KEY_FILE" \
    ./hardware/teleop/scripts/check_vuer.sh
ssh -o BatchMode=yes "$ROBOT" \
    'test "$(systemctl is-active hb_high_level.service 2>/dev/null || true)" = active && test "$(systemctl is-active hb_teleop.service 2>/dev/null || true)" != active && ss -H -lun "sport = :5560" | grep -q "127.0.0.1:5560"'

printf '%q ' "$0" "$@" >"$RUN_DIR/command.txt"
printf '\n' >>"$RUN_DIR/command.txt"

echo "[READY] Quest URL: https://$HOST_IP:8012/?ws=wss://$HOST_IP:8012"
echo "[READY] TRƯỚC CÒ PHẢI: đưa robot arms/head và người vận hành về neutral ban đầu như trong sim."
echo "[READY] Frame đầu tiên khi bóp cò phải được chốt làm source_zero của cả phiên."
echo "[READY] Giữ cò phải để điều khiển; nhả cò để receiver watchdog release và dừng."
echo "[READY] Evidence local: $RUN_DIR"

conda run --no-capture-output -n tv python scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" \
    --duration-s "$DURATION_S" \
    --frequency-hz 30 \
    --deadman-source right_trigger \
    --cert-file "$CERT_FILE" \
    --key-file "$KEY_FILE" \
    --stop-file "$STOP_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
| conda run --no-capture-output -n unitree_sim_env python scripts/teleop/run_r1_quest3_hardware_targets.py \
    --duration-s "$DURATION_S" \
    --control-hz 10 \
| ssh -o BatchMode=yes "$ROBOT" \
    "cd /home/unitree/HB/teleop && HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.high_level_sidecar --interface eth10 --udp-host 127.0.0.1 --udp-port 5560 --confirm-suspended-with-estop --confirm-dev-mode --duration-s '$DURATION_S' --first-input-timeout-s 120 --input-timeout-s 0.75 --state-timeout-s 0.20 --send-hz 100 --max-offset-rad 0.15 --log-dir /home/unitree/HB/teleop/logs" \
| tee "$RUN_DIR/robot_receiver.log"
