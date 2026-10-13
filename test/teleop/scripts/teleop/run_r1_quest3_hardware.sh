#!/usr/bin/env bash
# Foreground Quest -> arms/head IK -> SSH sidecar -> sole high-level lowcmd owner.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

# shellcheck disable=SC1091
source "$ROOT/scripts/load_network_config.sh"
hb_load_network_config

REQUIRE_LOCOMOTION="${HB_TELEOP_REQUIRE_LOCOMOTION:-1}"
LOCOMOTION_WAIT_S="${HB_TELEOP_LOCOMOTION_WAIT_S:-30}"
if [[ "$REQUIRE_LOCOMOTION" != "0" && "$REQUIRE_LOCOMOTION" != "1" ]]; then
    echo "[FAIL] HB_TELEOP_REQUIRE_LOCOMOTION must be 0 or 1" >&2
    exit 2
fi
if [[ ! "$LOCOMOTION_WAIT_S" =~ ^[0-9]+$ ]]; then
    echo "[FAIL] HB_TELEOP_LOCOMOTION_WAIT_S must be a non-negative integer" >&2
    exit 2
fi

if [[ "${CONFIRM_SUSPENDED_WITH_ESTOP:-0}" != "1" ]]; then
    if [[ ! -t 0 ]]; then
        echo "[FAIL] Cần terminal tương tác hoặc CONFIRM_SUSPENDED_WITH_ESTOP=1." >&2
        exit 2
    fi
    if [[ "$REQUIRE_LOCOMOTION" == "1" ]]; then
        read -r -p "Robot đã treo/cố định, R3 L2+B sẵn sàng, và high-level ở LOCOMOTION? Nhập YES: " confirmation
    else
        read -r -p "Robot đã treo/cố định, R3 L2+B sẵn sàng, và high-level ở ZERO TORQUE hoặc LOCOMOTION? Nhập YES: " confirmation
    fi
    if [[ "$confirmation" != "YES" ]]; then
        echo "[SAFE] Hủy; không khởi động teleop." >&2
        exit 2
    fi
fi

if [[ -z "${ROBOT:-}" ]]; then
    # Direct invocation still uses the same central config/discovery path as
    # the wrapper.  Do not fall back to the laptop's old hard-coded IP.
    # shellcheck disable=SC1091
    source "$ROOT/scripts/_find_robot.sh"
    find_robot
fi

wait_for_supported_state() {
    local deadline=$((SECONDS + LOCOMOTION_WAIT_S))
    local last="" status alive armed state key
    while (( SECONDS <= deadline )); do
        status="$(ssh -o BatchMode=yes -o ConnectTimeout=3 "$ROBOT" \
            "grep -E '^(high_alive|high_armed|high_state)=' /run/hb/status.env 2>/dev/null" \
            2>/dev/null || true)"
        alive="$(awk -F= '$1 == "high_alive" {print $2}' <<<"$status")"
        armed="$(awk -F= '$1 == "high_armed" {print $2}' <<<"$status")"
        state="$(awk -F= '$1 == "high_state" {print substr($0, index($0, "=")+1)}' <<<"$status")"
        key="$alive/$armed/$state"
        if [[ "$key" != "$last" ]]; then
            echo "[ROBOT] state=${state:-UNKNOWN} alive=${alive:-0} armed=${armed:-0}"
            last="$key"
        fi
        local state_ok=0
        if [[ "$state" == "LOCOMOTION" ]]; then
            state_ok=1
        elif [[ "$REQUIRE_LOCOMOTION" == "0" && "$state" == ZERO_TORQUE* ]]; then
            state_ok=1
        fi
        if [[ "$alive" == "1" && "$armed" == "1" && "$state_ok" == "1" ]]; then
            echo "[ROBOT] $state ready; H4 STOP preamble enabled"
            return 0
        fi
        sleep 1
    done
    if [[ "$REQUIRE_LOCOMOTION" == "1" ]]; then
        echo "[FAIL] Robot chưa vào LOCOMOTION trong ${LOCOMOTION_WAIT_S}s; bấm R2+A rồi chạy lại." >&2
    else
        echo "[FAIL] Robot chưa ở ZERO_TORQUE hoặc LOCOMOTION trong ${LOCOMOTION_WAIT_S}s." >&2
    fi
    return 1
}

wait_for_supported_state

check_no_stale_sidecar() {
    local active
    if ! active="$(ssh -o BatchMode=yes -o ConnectTimeout=3 "$ROBOT" \
        "ps -eo pid=,args= | grep -E '[t]eleop([./])hardware([./])high_level_sidecar' || true" \
        2>/dev/null)"; then
        echo "[FAIL] Không kiểm tra được sidecar trên robot." >&2
        return 1
    fi
    if [[ -n "${active//[[:space:]]/}" ]]; then
        echo "[FAIL] Sidecar teleop cũ đang chạy trên robot; dừng phiên cũ an toàn rồi chạy lại." >&2
        echo "$active" >&2
        return 1
    fi
}

# Không tự kill tiến trình điều khiển motor: một phiên cũ có thể vẫn đang được
# người vận hành giữ E-stop. Fail-closed để tránh hai phiên tranh quyền UDP.
check_no_stale_sidecar

ROBOT_INTERFACE="${HB_TELEOP_ROBOT_INTERFACE:-eth10}"
# Bao an toan cua phien: moi khop chi duoc lech toi da ngan nay so voi tu the luc bop
# co.  0.15 rad = 8.6 do -> tay chi "dao dong quanh vi tri init", dung nhu thiet ke cua
# lan nghiem thu bounded.  Noi DAN TUNG NAC va chi sau khi da xac nhan dung chieu:
# 0.15 (nghiem thu) -> 0.35 -> 0.60 -> 1.5 (gan nhu khong gioi han).
# Toc do van bi chan boi teleop_max_rate_rad_s = 2.0 rad/s trong controller, nen bao
# rong khong lam tay quat nhanh hon, chi cho phep di XA hon.  Nguoi giu E-stop, treo gia.
MAX_OFFSET_RAD="${HB_TELEOP_MAX_OFFSET_RAD:-0.15}"
DURATION_S="${HB_TELEOP_DURATION_S:-120}"
JOINT_LIMITS_ONLY="${HB_TELEOP_JOINT_LIMITS_ONLY:-0}"
if [[ "$JOINT_LIMITS_ONLY" != "0" && "$JOINT_LIMITS_ONLY" != "1" ]]; then
    echo "[FAIL] HB_TELEOP_JOINT_LIMITS_ONLY must be 0 or 1" >&2
    exit 2
fi

# Nguong "dau phai o giua" ma sidecar kiem TRUOC khi cap dien cho dau.
#
# O ZERO TORQUE dau limp nen no ru xuong theo trong luc: do thuc te 2026-08-26 la
# pitch 0.623 rad (35.7 do), vuot xa mac dinh 0.35 -> sidecar thoat ma 3
# ("head not neutral") va keo ca pipeline chet theo.  Mot minh vua deo Quest vua
# do dau la bat kha thi, nen mo num nay ra.
#
# Gate doc goc NGAY SAU target dau tien (khong phai luc chay script), tuc la vao
# luc ban vua vao VR -- xem high_level_sidecar.py:213.
#
# Noi den dau thi an toan: dau van bi slew-limit boi teleop_max_rate_rad_s, nen
# 0.70 rad mat ~0.35 s de ve giua o toc do 2.0.  Tran sidecar la 1.0; noi sat tran
# thi hanh trinh ve giua du dai de dap vao vai/gia treo -- kiem tra khoang trong truoc.
HEAD_PITCH_MAX_RAD="${HB_TELEOP_HEAD_PITCH_MAX_RAD:-0.35}"
HEAD_YAW_MAX_RAD="${HB_TELEOP_HEAD_YAW_MAX_RAD:-0.60}"
HOST_IP="${HB_TELEOP_HOST_IP:-auto}"
HOST_IP="$(hb_resolve_host_ip "$HOST_IP")"
CERT_FILE="${HB_TELEOP_CERT_FILE}"
KEY_FILE="${HB_TELEOP_KEY_FILE}"
VUER_ENV="${HB_TELEOP_VUER_ENV:-tv}"
VUER_PORT="${HB_TELEOP_VUER_PORT:-8012}"
IK_ENV="${HB_TELEOP_IK_ENV:-unitree_sim_env}"
IK_BACKEND="${HB_TELEOP_IK_BACKEND:-vendor_r1_a5}"
# Vendor R1-A5 is the primary IK route.  Keep coupled_dls only as an explicit
# rollback override; a plain launcher invocation must never silently select it.
# The active high-level policy is the H4 gait contract, so a plain invocation
# also follows that stream instead of sending legacy frames.
STREAM_CONTRACT="${HB_TELEOP_STREAM_CONTRACT:-h4}"
RELEASE_DEBOUNCE_S="${HB_TELEOP_RELEASE_DEBOUNCE_S:-5.0}"
VENDOR_PYTHON="${HB_TELEOP_VENDOR_PYTHON:-$HOME/.local/share/hb/teleop/vendor_ik_env/bin/python}"
BRIDGE_FREQUENCY_HZ="${HB_TELEOP_BRIDGE_FREQUENCY_HZ:-30}"
DEADMAN_SOURCE="${HB_TELEOP_DEADMAN_SOURCE:-right_trigger}"
TRIGGER_VALUE_THRESHOLD="${HB_TELEOP_TRIGGER_VALUE_THRESHOLD:-5.0}"
CONTROL_HZ="${HB_TELEOP_CONTROL_HZ:-10}"
SOURCE_TIMEOUT_S="${HB_TELEOP_SOURCE_TIMEOUT_S:-0.30}"
MAX_JOINT_VELOCITY_RAD_S="${HB_TELEOP_MAX_JOINT_VELOCITY_RAD_S:-0.50}"
MAX_JOINT_ACCELERATION_RAD_S2="${HB_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2:-1.0}"
UDP_HOST="${HB_TELEOP_UDP_HOST:-127.0.0.1}"
UDP_PORT="${HB_TELEOP_UDP_PORT:-5560}"
FIRST_INPUT_TIMEOUT_S="${HB_TELEOP_FIRST_INPUT_TIMEOUT_S:-120}"
INPUT_TIMEOUT_S="${HB_TELEOP_INPUT_TIMEOUT_S:-0.75}"
STATE_TIMEOUT_S="${HB_TELEOP_STATE_TIMEOUT_S:-0.20}"
SEND_HZ="${HB_TELEOP_SEND_HZ:-100}"
EXPECTED_MODE_MACHINE="${HB_TELEOP_EXPECTED_MODE_MACHINE:-1}"
VENDOR_ROOT="$ROOT/third_party/xr_teleoperate_r1_a5_845b25b"
case "$IK_BACKEND" in
    coupled_dls|vendor_r1_a5) ;;
    *) echo "[FAIL] HB_TELEOP_IK_BACKEND must be coupled_dls or vendor_r1_a5" >&2; exit 2 ;;
esac
case "$STREAM_CONTRACT" in
    legacy|h4) ;;
    *) echo "[FAIL] HB_TELEOP_STREAM_CONTRACT must be legacy or h4" >&2; exit 2 ;;
esac
if [[ "$JOINT_LIMITS_ONLY" == "1" && "$IK_BACKEND" != "vendor_r1_a5" ]]; then
    echo "[FAIL] HB_TELEOP_JOINT_LIMITS_ONLY requires HB_TELEOP_IK_BACKEND=vendor_r1_a5" >&2
    exit 2
fi
if [[ "$IK_BACKEND" == vendor_r1_a5 ]]; then
    if [[ ! -x "$VENDOR_PYTHON" ]]; then
        echo "[FAIL] Vendor Python missing: $VENDOR_PYTHON" >&2
        exit 2
    fi
    PYTHONPATH="$VENDOR_ROOT" "$VENDOR_PYTHON" -c 'import casadi, pinocchio, meshcat, matplotlib, logging_mp, numpy' || {
        echo "[FAIL] Vendor IK dependencies missing" >&2
        exit 2
    }
    env PYTHONNOUSERSITE=1 PYTHONPATH="$VENDOR_ROOT" "$VENDOR_PYTHON" \
        scripts/teleop/run_r1_vendor_ik_stream.py --preflight
    conda run --no-capture-output -n "$IK_ENV" env PYTHONNOUSERSITE=1 python \
        scripts/teleop/run_r1_vendor_targets.py --preflight
fi
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_r1_quest3_hardware"
RESULTS_ROOT="${HB_TELEOP_RESULTS_DIR:-results/smoke}"
if [[ "$RESULTS_ROOT" != /* ]]; then RESULTS_ROOT="$ROOT/$RESULTS_ROOT"; fi
RUN_DIR="$RESULTS_ROOT/$RUN_ID"
STOP_FILE="/tmp/$RUN_ID.stop"
mkdir -p "$RUN_DIR"

cleanup() {
    touch "$STOP_FILE"
}
trap cleanup EXIT INT TERM

HB_TELEOP_HOST_IP="$HOST_IP" \
HB_TELEOP_CERT_FILE="$CERT_FILE" \
HB_TELEOP_KEY_FILE="$KEY_FILE" \
    ./scripts/check_vuer.sh
# Default safety contract (also covered by static tests): ss -H -lun "sport = :5560"
# must expose the loopback receiver at 127.0.0.1:5560.
ssh -o BatchMode=yes "$ROBOT" \
    "test \"\$(systemctl is-active hb_high_level.service 2>/dev/null || true)\" = active && test \"\$(systemctl is-active hb_teleop.service 2>/dev/null || true)\" != active && ss -H -lun \"sport = :$UDP_PORT\" | grep -q \"127.0.0.1:$UDP_PORT\""

printf '%q ' "$0" "$@" >"$RUN_DIR/command.txt"
printf '\n' >>"$RUN_DIR/command.txt"

echo "[READY] Quest URL: https://$HOST_IP:$VUER_PORT/?ws=wss://$HOST_IP:$VUER_PORT"
echo "[READY] IK backend: $IK_BACKEND"
echo "[READY] Stream contract: $STREAM_CONTRACT"
echo "[READY] Release grace: ${RELEASE_DEBOUNCE_S}s (bop lai trong cua so nay de resume)"
echo "[READY] Position limit mode: $(if [[ "$JOINT_LIMITS_ONLY" == "1" ]]; then echo joint-limits-only; else echo bounded-session; fi)"
echo "[READY] TRƯỚC CÒ PHẢI: đưa robot arms/head và người vận hành về neutral ban đầu như trong sim."
echo "[READY] Frame đầu tiên khi bóp cò phải được chốt làm source_zero của cả phiên."
echo "[READY] Giữ cò phải để điều khiển; nhả cò phải 5.0s để kết thúc phiên, bóp lại trước đó để resume."
echo "[READY] Evidence local: $RUN_DIR"

set +e
if [[ "$IK_BACKEND" == vendor_r1_a5 ]]; then
    VENDOR_TARGET_MODE_ARGS=()
    VENDOR_TARGET_MODE_ARGS+=(--stream-contract "$STREAM_CONTRACT"
                              --release-debounce-s "$RELEASE_DEBOUNCE_S")
    REMOTE_LIMIT_MODE_ARGS=""
    if [[ "$JOINT_LIMITS_ONLY" == "1" ]]; then
        VENDOR_TARGET_MODE_ARGS+=(--joint-limits-only)
        REMOTE_LIMIT_MODE_ARGS=" --joint-limits-only"
    fi
conda run --no-capture-output -n "$VUER_ENV" env PYTHONNOUSERSITE=1 python scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" \
    --port "$VUER_PORT" \
    --duration-s "$DURATION_S" \
    --frequency-hz "$BRIDGE_FREQUENCY_HZ" \
    --deadman-source "$DEADMAN_SOURCE" \
    --trigger-value-threshold "$TRIGGER_VALUE_THRESHOLD" \
    --cert-file "$CERT_FILE" \
    --key-file "$KEY_FILE" \
    --stop-file "$STOP_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
    2> >(tee "$RUN_DIR/bridge.log" >&2) \
| tee "$RUN_DIR/quest_commands.jsonl" \
| env PYTHONNOUSERSITE=1 PYTHONPATH="$VENDOR_ROOT" "$VENDOR_PYTHON" scripts/teleop/run_r1_vendor_ik_stream.py \
    --passthrough \
    --stats-path "$RUN_DIR/vendor_ik_stats.json" \
    2> >(tee "$RUN_DIR/vendor_ik.log" >&2) \
| tee "$RUN_DIR/vendor_commands.jsonl" \
| conda run --no-capture-output -n "$IK_ENV" env PYTHONNOUSERSITE=1 python scripts/teleop/run_r1_vendor_targets.py \
    --duration-s "$DURATION_S" \
    --control-hz "$CONTROL_HZ" \
    --source-timeout-s "$SOURCE_TIMEOUT_S" \
    --max-joint-velocity-rad-s "$MAX_JOINT_VELOCITY_RAD_S" \
    --max-joint-acceleration-rad-s2 "$MAX_JOINT_ACCELERATION_RAD_S2" \
    --diagnostics-log "$RUN_DIR/vendor_targets_diagnostics.jsonl" \
    "${VENDOR_TARGET_MODE_ARGS[@]}" \
    2> >(tee "$RUN_DIR/mapper.log" >&2) \
| tee "$RUN_DIR/targets.jsonl" \
| ssh -o BatchMode=yes "$ROBOT" \
    "cd /home/unitree/HB/teleop && HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.high_level_sidecar --stream-contract '$STREAM_CONTRACT' --interface '$ROBOT_INTERFACE' --udp-host '$UDP_HOST' --udp-port '$UDP_PORT' --confirm-suspended-with-estop --confirm-dev-mode --duration-s '$DURATION_S' --first-input-timeout-s '$FIRST_INPUT_TIMEOUT_S' --input-timeout-s '$INPUT_TIMEOUT_S' --state-timeout-s '$STATE_TIMEOUT_S' --send-hz '$SEND_HZ' --max-offset-rad '$MAX_OFFSET_RAD' --head-yaw-max-rad '$HEAD_YAW_MAX_RAD' --head-pitch-max-rad '$HEAD_PITCH_MAX_RAD' --expected-mode-machine '$EXPECTED_MODE_MACHINE' --log-dir /home/unitree/HB/teleop/logs$REMOTE_LIMIT_MODE_ARGS" \
    2> >(tee "$RUN_DIR/ssh.log" >&2) \
| tee "$RUN_DIR/robot_receiver.log"
stage_status=("${PIPESTATUS[@]}")
else
    REMOTE_LIMIT_MODE_ARGS=""
    if [[ "$JOINT_LIMITS_ONLY" == "1" ]]; then
        REMOTE_LIMIT_MODE_ARGS=" --joint-limits-only"
    fi
conda run --no-capture-output -n "$VUER_ENV" env PYTHONNOUSERSITE=1 python scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" \
    --port "$VUER_PORT" \
    --duration-s "$DURATION_S" \
    --frequency-hz "$BRIDGE_FREQUENCY_HZ" \
    --deadman-source "$DEADMAN_SOURCE" \
    --trigger-value-threshold "$TRIGGER_VALUE_THRESHOLD" \
    --cert-file "$CERT_FILE" \
    --key-file "$KEY_FILE" \
    --stop-file "$STOP_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
    2> >(tee "$RUN_DIR/bridge.log" >&2) \
| tee "$RUN_DIR/quest_commands.jsonl" \
| conda run --no-capture-output -n "$IK_ENV" env PYTHONNOUSERSITE=1 python scripts/teleop/run_r1_quest3_hardware_targets.py \
    --duration-s "$DURATION_S" \
    --control-hz "$CONTROL_HZ" \
    --ik-max-iterations 6 \
    --on-release stop \
    --stream-contract "$STREAM_CONTRACT" \
    --release-debounce-s "$RELEASE_DEBOUNCE_S" \
    --diagnostics-log "$RUN_DIR/ik_diagnostics.jsonl" \
    2> >(tee "$RUN_DIR/mapper.log" >&2) \
| tee "$RUN_DIR/targets.jsonl" \
| ssh -o BatchMode=yes "$ROBOT" \
    "cd /home/unitree/HB/teleop && HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.high_level_sidecar --stream-contract '$STREAM_CONTRACT' --interface '$ROBOT_INTERFACE' --udp-host '$UDP_HOST' --udp-port '$UDP_PORT' --confirm-suspended-with-estop --confirm-dev-mode --duration-s '$DURATION_S' --first-input-timeout-s '$FIRST_INPUT_TIMEOUT_S' --input-timeout-s '$INPUT_TIMEOUT_S' --state-timeout-s '$STATE_TIMEOUT_S' --send-hz '$SEND_HZ' --max-offset-rad '$MAX_OFFSET_RAD' --head-yaw-max-rad '$HEAD_YAW_MAX_RAD' --head-pitch-max-rad '$HEAD_PITCH_MAX_RAD' --expected-mode-machine '$EXPECTED_MODE_MACHINE' --log-dir /home/unitree/HB/teleop/logs$REMOTE_LIMIT_MODE_ARGS" \
    2> >(tee "$RUN_DIR/ssh.log" >&2) \
| tee "$RUN_DIR/robot_receiver.log"
stage_status=("${PIPESTATUS[@]}")
fi
printf '%s\n' "${stage_status[*]}" >"$RUN_DIR/pipeline_exit_codes.txt"
echo "[PIPELINE] stage exit codes: ${stage_status[*]}" >&2
for code in "${stage_status[@]}"; do
    if (( code != 0 )); then
        exit "$code"
    fi
done
