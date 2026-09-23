#!/usr/bin/env bash
# Foreground Quest -> arms/head IK -> SSH sidecar -> sole high-level lowcmd owner.
#
# The unmodified vendor xr_teleoperate R1_A5_ArmIK is the sole solver.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

ROBOT_CAMERA_WEBRTC_URL="${ROBOT_CAMERA_WEBRTC_URL:-}"
ROBOT_CAMERA_ZMQ_ENDPOINT="${ROBOT_CAMERA_ZMQ_ENDPOINT:-}"
HB_ROBOT_CAMERA="${HB_ROBOT_CAMERA:-1}"
ROBOT_CAMERA_LOCAL_PORT="${ROBOT_CAMERA_LOCAL_PORT:-8765}"
ROBOT_CAMERA_REMOTE_PORT="${ROBOT_CAMERA_REMOTE_PORT:-8765}"
ROBOT_CAMERA_READY_TIMEOUT_S="${ROBOT_CAMERA_READY_TIMEOUT_S:-20}"
HB_TELEOP_MIRROR_SIM="${HB_TELEOP_MIRROR_SIM:-1}"
ISAAC_SIM_PYTHON="${ISAAC_SIM_PYTHON:-/home/ubuntu22/isaacsim_5.1/python.sh}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-$ROOT/../Happy-Baby-R1-vla/third_party/IsaacLab}"
RUN_ID="${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)_r1_quest3_hardware}"
REMOTE_CAMERA_DIR="/home/unitree/HB/teleop/logs/camera/$RUN_ID"
CAMERA_ARGS=()
CAMERA_TRANSPORT_MODE="disabled"
if [[ -n "$ROBOT_CAMERA_WEBRTC_URL" ]]; then
    CAMERA_TRANSPORT_MODE="external_webrtc"
    CAMERA_ARGS+=(--robot-camera-webrtc-url "$ROBOT_CAMERA_WEBRTC_URL")
elif [[ "$HB_ROBOT_CAMERA" == "1" ]]; then
    CAMERA_TRANSPORT_MODE="enabled"
    CAMERA_ARGS+=(--robot-camera-preview-url "http://127.0.0.1:${ROBOT_CAMERA_LOCAL_PORT}/preview.jpg")
elif [[ "$HB_ROBOT_CAMERA" != "0" ]]; then
    echo "[FAIL] HB_ROBOT_CAMERA must be 0 or 1." >&2
    exit 2
fi

# Executable wiring contract for local tests. It exits before safety prompts,
# robot discovery, SSH, Vuer, Isaac or any command transport is opened.
if [[ "${HB_TELEOP_WIRING_DRY_RUN:-0}" == "1" ]]; then
    echo "camera_transport=$CAMERA_TRANSPORT_MODE"
    if [[ "$CAMERA_TRANSPORT_MODE" == "enabled" ]]; then
        echo "camera_manager=python3 scripts/teleop/run_r1_camera_transport.py --robot $ROBOT --local-port $ROBOT_CAMERA_LOCAL_PORT --remote-port $ROBOT_CAMERA_REMOTE_PORT --remote-record-dir $REMOTE_CAMERA_DIR"
        echo "camera_ready_gate=before_control_pipeline"
    fi
    printf 'camera_args='
    printf '%s ' "${CAMERA_ARGS[@]}"
    printf '\n'
    if [[ "$HB_TELEOP_MIRROR_SIM" == "1" ]]; then
        echo "sim_mirror=enabled"
        echo "sim_python=$ISAAC_SIM_PYTHON"
        echo "isaaclab_root=$ISAACLAB_ROOT"
    else
        echo "sim_mirror=disabled"
    fi
    exit 0
fi

if [[ "$HB_TELEOP_MIRROR_SIM" == "1" ]]; then
    if [[ ! -x "$ISAAC_SIM_PYTHON" ]]; then
        echo "[FAIL] Không tìm thấy Isaac Sim Python executable: $ISAAC_SIM_PYTHON" >&2
        exit 2
    fi
    if [[ ! -d "$ISAACLAB_ROOT/source/isaaclab/isaaclab" ]]; then
        echo "[FAIL] Không tìm thấy IsaacLab source: $ISAACLAB_ROOT/source/isaaclab" >&2
        exit 2
    fi
    if ! PYTHONPATH="$ISAACLAB_ROOT/source/isaaclab${PYTHONPATH:+:$PYTHONPATH}" \
        "$ISAAC_SIM_PYTHON" -c 'from isaaclab.app import AppLauncher' >/dev/null; then
        echo "[FAIL] Isaac Sim Python không import được isaaclab.app.AppLauncher." >&2
        exit 2
    fi
    echo "[OK] Isaac mirror runtime: $ISAAC_SIM_PYTHON + $ISAACLAB_ROOT"
fi

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

# Chuẩn hoá dáng robot ngay sau khi bóp cò, trước khi teleop bám tay. Upstream
# ramp tới target vendor đầu tiên để hardware và sim có cùng reference q; coupled legacy
# vẫn ramp về nominal. HB_TELEOP_HOME=0 để bỏ qua.
# Envelope mỗi khớp so với mốc phiên. 0.15 là giá trị cũ; phiên 2026-08-29 bão
# hoà nó trên 10/12 khớp nên mặc định lên 1.0 rad (57 độ). Trần sidecar là 1.0.
HB_TELEOP_MAX_OFFSET_RAD="${HB_TELEOP_MAX_OFFSET_RAD:-1.0}"
# Envelope riêng cho 6 khớp vai. 3.2 phủ trọn khớp rộng nhất (shoulder pitch
# ±3.142 trong asset), nên vai chạy hết tầm và giới hạn khớp của asset — do
# producer kẹp — là lớp chặn duy nhất còn lại. Người vận hành xác nhận an toàn
# trên giá treo ngày 2026-08-29.
HB_TELEOP_MAX_OFFSET_SHOULDER_RAD="${HB_TELEOP_MAX_OFFSET_SHOULDER_RAD:-3.2}"
HB_TELEOP_HOME="${HB_TELEOP_HOME:-1}"
HOME_ARG=""
[[ "$HB_TELEOP_HOME" == "1" ]] && HOME_ARG="--home-to-source"
# Không chốt cứng IP: wlan0 của robot lấy địa chỉ động. Thứ tự ưu tiên là
# ROBOT= trên dòng lệnh, rồi ~/.config/hb/robot.env, rồi dò. assert_robot xác
# minh đúng máy trước khi làm bất cứ gì — một máy lạ giữ IP cũ vẫn trả lời ping.
# shellcheck source=../../hardware/teleop/scripts/_find_robot.sh
source "$ROOT/hardware/teleop/scripts/_find_robot.sh"
find_robot || exit 2
assert_robot "$ROBOT" || exit 2
DURATION_S="${DURATION_S:-120}"
HOST_IP="${HOST_IP:-10.42.0.1}"
CERT_FILE="${CERT_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/cert.pem}"
KEY_FILE="${KEY_FILE:-$HOME/.config/xr_teleoperate/t001_10_42/key.pem}"
SIM_DEVICE="${SIM_DEVICE:-cuda:0}"
SIM_READY_TIMEOUT_S="${SIM_READY_TIMEOUT_S:-180}"
RUN_DIR="$ROOT/results/smoke/$RUN_ID"
STOP_FILE="/tmp/$RUN_ID.stop"
SIM_FIFO="$RUN_DIR/sim_commands.fifo"
SIM_PID=""
CAMERA_RECORDER_PID=""
CAMERA_TRANSPORT_PID=""
CAMERA_TRANSPORT_READY_FILE="$RUN_DIR/camera_transport_ready.json"
CAMERA_TRANSPORT_STATUS_FILE="$RUN_DIR/camera_transport_status.json"
SIM_KEEPALIVE_FD=""
mkdir -p "$RUN_DIR"

cleanup() {
    touch "$STOP_FILE"
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then kill -TERM "$SIM_PID" 2>/dev/null || true; fi
    if [[ -n "$CAMERA_RECORDER_PID" ]] && kill -0 "$CAMERA_RECORDER_PID" 2>/dev/null; then kill -TERM "$CAMERA_RECORDER_PID" 2>/dev/null || true; fi
    if [[ -n "$CAMERA_TRANSPORT_PID" ]] && kill -0 "$CAMERA_TRANSPORT_PID" 2>/dev/null; then kill -TERM "$CAMERA_TRANSPORT_PID" 2>/dev/null || true; fi
    if [[ -n "$SIM_KEEPALIVE_FD" ]]; then exec {SIM_KEEPALIVE_FD}>&-; fi
    [[ -p "$SIM_FIFO" ]] && rm -f "$SIM_FIFO"
}
trap cleanup EXIT INT TERM

HB_TELEOP_HOST_IP="$HOST_IP" \
HB_TELEOP_CERT_FILE="$CERT_FILE" \
HB_TELEOP_KEY_FILE="$KEY_FILE" \
python3 -c '
import sys
from pathlib import Path
from teleop.r1.launcher import ensure_self_signed_certificate

changed = ensure_self_signed_certificate(sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]))
if changed:
    print(f"[OK] Đã tạo/gia hạn cert Quest cho {sys.argv[1]}: {sys.argv[2]}")
' "$HOST_IP" "$CERT_FILE" "$KEY_FILE"

HB_TELEOP_HOST_IP="$HOST_IP" \
HB_TELEOP_CERT_FILE="$CERT_FILE" \
HB_TELEOP_KEY_FILE="$KEY_FILE" \
    ./hardware/teleop/scripts/check_vuer.sh
# Điều kiện thật là "có ĐÚNG MỘT chủ rt/lowcmd và nó đang giữ 5560", chứ không
# phải "service đang active". Bản cô lập high_level_lock chạy foreground và cố ý
# dừng service, nên kiểm theo service sẽ chặn đúng cấu hình hợp lệ. Đọc
# /proc/PID/exe vì bản foreground được gọi bằng đường dẫn tương đối và không
# match được theo command line.
ssh -o BatchMode=yes "$ROBOT" '
    set -e
    owners=""
    for p in $(pgrep -x run_r1 2>/dev/null || true); do
        exe=$(readlink -f /proc/$p/exe 2>/dev/null || true)
        [ -n "$exe" ] && owners="$owners$exe\n"
    done
    n=$(printf "%b" "$owners" | grep -c . || true)
    if [ "$n" -eq 0 ]; then
        echo "[FAIL] Khong co chu rt/lowcmd nao dang chay." >&2
        echo "       Bat service:  sudo systemctl start hb_high_level" >&2
        echo "       Hoac ban co lap:  cd ~/HB/high_level_lock && ./scripts/run_lock_foreground.sh" >&2
        exit 1
    fi
    if [ "$n" -gt 1 ]; then
        echo "[FAIL] Co $n tien trinh run_r1 cung chay -- vi pham D003:" >&2
        printf "%b" "$owners" >&2
        exit 1
    fi
    echo "[OK] Chu rt/lowcmd: $(printf "%b" "$owners" | tr -d "\n")"
    if [ "$(systemctl is-active hb_teleop.service 2>/dev/null || true)" = active ]; then
        echo "[FAIL] hb_teleop.service dang active; phai inactive." >&2
        exit 1
    fi
    if ! ss -H -lun "sport = :5560" | grep -q "127.0.0.1:5560"; then
        echo "[FAIL] Khong co listener UTL1 tren 127.0.0.1:5560." >&2
        exit 1
    fi
    echo "[OK] UTL1 loopback 5560 dang lang nghe"
'

printf '%q ' "$0" "$@" >"$RUN_DIR/command.txt"
printf '\n' >>"$RUN_DIR/command.txt"

if [[ "$CAMERA_TRANSPORT_MODE" == "enabled" ]]; then
    : >"$RUN_DIR/camera_transport.stderr.log"
    python3 scripts/teleop/run_r1_camera_transport.py \
        --robot "$ROBOT" \
        --local-port "$ROBOT_CAMERA_LOCAL_PORT" \
        --remote-port "$ROBOT_CAMERA_REMOTE_PORT" \
        --remote-record-dir "$REMOTE_CAMERA_DIR" \
        --ready-file "$CAMERA_TRANSPORT_READY_FILE" \
        --status-file "$CAMERA_TRANSPORT_STATUS_FILE" \
        --ready-timeout-s "$ROBOT_CAMERA_READY_TIMEOUT_S" \
        2> >(tee -a "$RUN_DIR/camera_transport.stderr.log" >&2) &
    CAMERA_TRANSPORT_PID=$!
    camera_ready_deadline=$((SECONDS + ROBOT_CAMERA_READY_TIMEOUT_S + 5))
    while [[ ! -f "$CAMERA_TRANSPORT_READY_FILE" ]]; do
        if ! kill -0 "$CAMERA_TRANSPORT_PID" 2>/dev/null; then
            echo "[FAIL] Camera transport thoát trước khi ready; không mở control pipeline." >&2
            wait "$CAMERA_TRANSPORT_PID" || true
            exit 2
        fi
        if (( SECONDS >= camera_ready_deadline )); then
            echo "[FAIL] Camera transport chưa ready; không mở control pipeline." >&2
            kill -TERM "$CAMERA_TRANSPORT_PID" 2>/dev/null || true
            wait "$CAMERA_TRANSPORT_PID" || true
            exit 2
        fi
        sleep 0.1
    done
    echo "[READY] Camera R1 qua loopback SSH: http://127.0.0.1:${ROBOT_CAMERA_LOCAL_PORT}/preview.jpg"
fi

echo "[READY] Quest URL: https://$HOST_IP:8012/?ws=wss://$HOST_IP:8012"
echo "[READY] TRƯỚC CÒ PHẢI: đưa robot arms/head và người vận hành về neutral ban đầu như trong sim."
echo "[READY] Frame đầu tiên khi bóp cò phải được chốt làm source_zero của cả phiên."
echo "[READY] Giữ cò phải để điều khiển; nhả cò = PAUSED/STOP nhưng không đóng session."
echo "[READY] Evidence local: $RUN_DIR"
echo "[READY] Solver: upstream R1_A5_ArmIK"
if [[ "$HB_TELEOP_HOME" == "1" ]]; then
    echo "[READY] Source alignment BẬT: GIỮ YÊN đầu và hai controller sau khi bóp cò."
    echo "[READY] Robot ramp chậm tới target vendor đầu tiên; sau đó không cộng offset posture vào q upstream."
    echo "[READY] Robot đang tự đi trong dòng [HOME]; kiểm khoảng trống quanh cả hai tay và đầu."
else
    echo "[READY] Homing TẮT: robot giữ nguyên tư thế hiện tại làm mốc."
fi
echo "[READY] Envelope: ${HB_TELEOP_MAX_OFFSET_RAD} rad/khớp; VAI ${HB_TELEOP_MAX_OFFSET_SHOULDER_RAD} rad (hết tầm)."
echo "[READY] Cò trái = căn lại theo mode hiện tại; nhả cò phải = tay giữ nguyên tư thế."
if [[ "$CAMERA_TRANSPORT_MODE" != "disabled" ]]; then
    echo "[READY] Nút A tay phải = đổi Quest passthrough / camera robot (view ban đầu: Quest)."
fi
echo "[READY] Cò trái có thể bấm khi PAUSED; anchor mới được chốt sau 3 mẫu khi bóp lại cò phải."
if [[ "$HB_TELEOP_MIRROR_SIM" == "1" ]]; then
    echo "[READY] Isaac mirror BẬT: cùng solved sequence với hardware; video ở $RUN_DIR/simulation."
fi
if [[ "$CAMERA_TRANSPORT_MODE" == "enabled" ]]; then
    echo "[READY] JPEG gốc camera robot sẽ được ghi tại $RUN_DIR/robot_camera_original."
elif [[ -n "$ROBOT_CAMERA_ZMQ_ENDPOINT" ]]; then
    echo "[READY] Ghi camera robot BẬT: $ROBOT_CAMERA_ZMQ_ENDPOINT -> $RUN_DIR/robot_camera."
elif [[ -n "$ROBOT_CAMERA_WEBRTC_URL" ]]; then
    echo "[WARN] Camera chỉ hiển thị trong Quest; thiếu ROBOT_CAMERA_ZMQ_ENDPOINT nên chưa ghi MP4 camera robot." >&2
fi

# Nhánh Isaac chỉ đọc cùng solved stream. Fanout ưu tiên stdout hardware và dùng
# queue hữu hạn cho mirror, nên Isaac chậm/crash không được phép chặn robot.
FANOUT_CMD=(cat)
if [[ "$HB_TELEOP_MIRROR_SIM" == "1" ]]; then
    mkfifo "$SIM_FIFO"
    # O_RDWR giữ FIFO mở để shell có thể exec Isaac trước khi command writer
    # xuất hiện. Không FD này, cả Isaac và fanout sẽ chờ nhau ở open().
    exec {SIM_KEEPALIVE_FD}<>"$SIM_FIFO"
    : >"$RUN_DIR/simulator.stderr.log"
    PYTHONPATH="$ISAACLAB_ROOT/source/isaaclab${PYTHONPATH:+:$PYTHONPATH}" \
        "$ISAAC_SIM_PYTHON" scripts/teleop/run_r1_quest3_live.py \
        --output-dir "$RUN_DIR/simulation" \
        --duration-s "$DURATION_S" \
        --control-hz 30 --physics-hz 200 --video-fps 10 \
        --device "$SIM_DEVICE" --disable-self-collisions \
        --upstream-joint-stream-config experiments/r1_teleop/quest3_sim_v1/baseline/config/upstream_stream.json \
        --stop-file "$STOP_FILE" --ready-file "$RUN_DIR/simulator_ready.json" \
        <"$SIM_FIFO" 2> >(tee -a "$RUN_DIR/simulator.stderr.log" >&2) &
    SIM_PID=$!
    ready_deadline=$((SECONDS + SIM_READY_TIMEOUT_S))
    while [[ ! -f "$RUN_DIR/simulator_ready.json" ]]; do
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo "[FAIL] Isaac mirror thoát trước khi ready; không mở hardware pipeline." >&2
            wait "$SIM_PID" || true
            exit 2
        fi
        if (( SECONDS >= ready_deadline )); then
            echo "[FAIL] Isaac mirror chưa ready sau ${SIM_READY_TIMEOUT_S}s; không mở hardware pipeline." >&2
            kill -TERM "$SIM_PID" 2>/dev/null || true
            wait "$SIM_PID" || true
            exit 2
        fi
        sleep 0.2
    done
    echo "[READY] Isaac đã khởi tạo; bắt đầu Quest/hardware stream."
    FANOUT_CMD=(python3 scripts/teleop/fanout_command_stream.py
        --mirror-path "$SIM_FIFO" --stats-path "$RUN_DIR/fanout_stats.json")
fi
if [[ "$CAMERA_TRANSPORT_MODE" == "enabled" && -f "$CAMERA_TRANSPORT_STATUS_FILE" ]]; then
    echo "[FAIL] Camera transport đã thoát trong lúc khởi tạo Isaac; không mở hardware pipeline." >&2
    exit 2
fi
if [[ -n "$ROBOT_CAMERA_ZMQ_ENDPOINT" ]]; then
    : >"$RUN_DIR/robot_camera.stderr.log"
    conda run --no-capture-output -n tv \
        python scripts/teleop/record_robot_camera_zmq.py \
        --endpoint "$ROBOT_CAMERA_ZMQ_ENDPOINT" \
        --output-dir "$RUN_DIR/robot_camera" \
        --duration-s "$DURATION_S" --fps 30 --stop-file "$STOP_FILE" \
        2> >(tee -a "$RUN_DIR/robot_camera.stderr.log" >&2) &
    CAMERA_RECORDER_PID=$!
fi

# Mỗi tầng có stderr và exit code riêng. Trước đây một downstream close lan
# ngược qua cả pipe nhưng chỉ còn lại robot_receiver.log, nên không thể biết IK,
# target producer, SSH hay sidecar là tầng đóng trước.
: >"$RUN_DIR/bridge.stderr.log"
: >"$RUN_DIR/upstream_solver.stderr.log"
: >"$RUN_DIR/hardware_targets.stderr.log"
: >"$RUN_DIR/ssh.stderr.log"
set +e
conda run --no-capture-output -n tv python scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" \
    --duration-s "$DURATION_S" \
    --frequency-hz 30 \
    --deadman-source right_trigger \
    --cert-file "$CERT_FILE" \
    --key-file "$KEY_FILE" \
    --stop-file "$STOP_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
    "${CAMERA_ARGS[@]}" \
    2> >(tee -a "$RUN_DIR/bridge.stderr.log" >&2) \
| conda run --no-capture-output -n tv python scripts/teleop/run_r1_upstream_ik_stream.py \
    --passthrough --stats-path "$RUN_DIR/upstream_solver_stats.json" \
    2> >(tee -a "$RUN_DIR/upstream_solver.stderr.log" >&2) \
| conda run --no-capture-output -n unitree_sim_env python scripts/teleop/run_r1_quest3_hardware_targets.py \
    --duration-s "$DURATION_S" \
    --control-hz 10 \
    --command-log "$RUN_DIR/hardware_targets.jsonl" \
    2> >(tee -a "$RUN_DIR/hardware_targets.stderr.log" >&2) \
| "${FANOUT_CMD[@]}" \
| ssh -o BatchMode=yes "$ROBOT" \
    "cd /home/unitree/HB/teleop && HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.high_level_sidecar --interface eth10 --udp-host 127.0.0.1 --udp-port 5560 --confirm-suspended-with-estop --confirm-dev-mode --duration-s '$DURATION_S' --first-input-timeout-s 120 --input-timeout-s 0.75 --state-timeout-s 0.20 --send-hz 100 --max-offset-rad $HB_TELEOP_MAX_OFFSET_RAD --max-offset-rad-shoulders $HB_TELEOP_MAX_OFFSET_SHOULDER_RAD $HOME_ARG --log-dir /home/unitree/HB/teleop/logs" \
    2> >(tee -a "$RUN_DIR/ssh.stderr.log" >&2) \
| tee "$RUN_DIR/robot_receiver.log"
PIPELINE_STATUS=("${PIPESTATUS[@]}")
set -e

printf '{\n  "bridge": %s,\n  "solver_or_passthrough": %s,\n  "hardware_targets": %s,\n  "nonblocking_fanout": %s,\n  "ssh_sidecar": %s,\n  "receiver_tee": %s\n}\n' \
    "${PIPELINE_STATUS[0]}" "${PIPELINE_STATUS[1]}" "${PIPELINE_STATUS[2]}" \
    "${PIPELINE_STATUS[3]}" "${PIPELINE_STATUS[4]}" "${PIPELINE_STATUS[5]}" \
    >"$RUN_DIR/pipeline_status.json"

PIPELINE_RC=0
for stage_rc in "${PIPELINE_STATUS[@]}"; do
    if [[ "$stage_rc" -ne 0 ]]; then
        PIPELINE_RC="$stage_rc"
    fi
done
touch "$STOP_FILE"
SIM_RC=0
CAMERA_RECORDER_RC=0
CAMERA_TRANSPORT_RC=0
FANOUT_EVIDENCE_RC=0
set +e
if [[ -n "$SIM_PID" ]]; then
    wait "$SIM_PID"
    SIM_RC=$?
fi
if [[ -n "$CAMERA_RECORDER_PID" ]]; then
    wait "$CAMERA_RECORDER_PID"
    CAMERA_RECORDER_RC=$?
fi
if [[ -n "$CAMERA_TRANSPORT_PID" ]]; then
    if kill -0 "$CAMERA_TRANSPORT_PID" 2>/dev/null; then
        kill -TERM "$CAMERA_TRANSPORT_PID" 2>/dev/null || true
    fi
    wait "$CAMERA_TRANSPORT_PID"
    CAMERA_TRANSPORT_RC=$?
fi
if [[ "$HB_TELEOP_MIRROR_SIM" == "1" ]]; then
    python3 -c '
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    stats = json.load(stream)
valid = (
    stats.get("mirror_drop_count") == 0
    and stats.get("mirror_failure") is None
    and stats.get("input_line_count", 0) > 0
    and stats.get("primary_line_count") == stats.get("input_line_count")
    and stats.get("mirror_line_count") == stats.get("input_line_count")
)
raise SystemExit(0 if valid else 1)
' "$RUN_DIR/fanout_stats.json"
    FANOUT_EVIDENCE_RC=$?
fi
set -e
printf '{\n  "simulator": %s,\n  "legacy_robot_camera_recorder": %s,\n  "camera_transport": %s,\n  "fanout_evidence": %s\n}\n' \
    "$SIM_RC" "$CAMERA_RECORDER_RC" "$CAMERA_TRANSPORT_RC" "$FANOUT_EVIDENCE_RC" \
    >"$RUN_DIR/auxiliary_status.json"
if [[ -n "$SIM_KEEPALIVE_FD" ]]; then
    exec {SIM_KEEPALIVE_FD}>&-
    SIM_KEEPALIVE_FD=""
fi
[[ -p "$SIM_FIFO" ]] && rm -f "$SIM_FIFO"
echo "[DONE] Pipeline status: $RUN_DIR/pipeline_status.json"

# Pull the robot-side encoder/target evidence into the same immutable bundle.
# The receiver prints the exact directory only after it has flushed metadata and
# samples; accepting any other remote prefix would make this an arbitrary copy.
REMOTE_FETCH_RC=1
REMOTE_RUN_DIR="$(sed -n 's/^.* evidence: //p' "$RUN_DIR/robot_receiver.log" | tail -n 1)"
if [[ "$REMOTE_RUN_DIR" == /home/unitree/HB/teleop/logs/* ]]; then
    mkdir -p "$RUN_DIR/robot"
    set +e
    rsync -a -e 'ssh -o BatchMode=yes' "$ROBOT:$REMOTE_RUN_DIR/" "$RUN_DIR/robot/"
    REMOTE_FETCH_RC=$?
    set -e
fi

CAMERA_FETCH_RC=0
CAMERA_EVIDENCE_RC=0
if [[ "$CAMERA_TRANSPORT_MODE" == "enabled" ]]; then
    mkdir -p "$RUN_DIR/robot_camera_original"
    set +e
    rsync -a -e 'ssh -o BatchMode=yes' "$ROBOT:$REMOTE_CAMERA_DIR/" "$RUN_DIR/robot_camera_original/"
    CAMERA_FETCH_RC=$?
    if [[ "$CAMERA_FETCH_RC" -eq 0 ]]; then
        python3 -c '
import json
import sys
from pathlib import Path
from scripts.teleop.run_r1_camera_transport import validate_recording_directory

result = validate_recording_directory(Path(sys.argv[1]))
print(json.dumps(result, sort_keys=True))
' "$RUN_DIR/robot_camera_original" >"$RUN_DIR/camera_evidence_validation.json"
        CAMERA_EVIDENCE_RC=$?
    else
        CAMERA_EVIDENCE_RC=1
    fi
    set -e
fi
printf '{\n  "transport": %s,\n  "remote_fetch": %s,\n  "recording_validation": %s\n}\n' \
    "$CAMERA_TRANSPORT_RC" "$CAMERA_FETCH_RC" "$CAMERA_EVIDENCE_RC" \
    >"$RUN_DIR/camera_artifact_status.json"

# Figures and the MP4 are derived from synchronized target/encoder telemetry.
# They are evidence of tracking over time, explicitly not camera footage.
set +e
if [[ -d "$RUN_DIR/simulation" && -f "$RUN_DIR/bridge_connection.jsonl" ]]; then
    cp "$RUN_DIR/bridge_connection.jsonl" "$RUN_DIR/simulation/bridge_connection.jsonl"
fi
COMPARE_RC=0
if [[ "$HB_TELEOP_MIRROR_SIM" == "1" && -d "$RUN_DIR/simulation" && -d "$RUN_DIR/robot" ]]; then
    conda run --no-capture-output -n unitree_sim_env \
        python scripts/teleop/compare_sim_hardware_run.py "$RUN_DIR"
    COMPARE_RC=$?
fi
MPLCONFIGDIR=/tmp/hb-matplotlib conda run --no-capture-output -n unitree_sim_env \
    python scripts/teleop/finalize_r1_run.py hardware "$RUN_DIR"
FINALIZE_RC=$?
SIM_FINALIZE_RC=0
if [[ "$HB_TELEOP_MIRROR_SIM" == "1" && -d "$RUN_DIR/simulation" ]]; then
    MPLCONFIGDIR=/tmp/hb-matplotlib conda run --no-capture-output -n unitree_sim_env \
        python scripts/teleop/finalize_r1_run.py simulation "$RUN_DIR/simulation"
    SIM_FINALIZE_RC=$?
fi
set -e

if [[ "$REMOTE_FETCH_RC" -ne 0 ]]; then
    echo "[ARTIFACT] Không tải được robot-side samples; run bị đánh dấu incomplete." >&2
fi
if [[ "$PIPELINE_RC" -ne 0 ]]; then exit "$PIPELINE_RC"; fi
if [[ "$SIM_RC" -ne 0 ]]; then exit "$SIM_RC"; fi
if [[ "$FANOUT_EVIDENCE_RC" -ne 0 ]]; then exit "$FANOUT_EVIDENCE_RC"; fi
if [[ "$SIM_FINALIZE_RC" -ne 0 ]]; then exit "$SIM_FINALIZE_RC"; fi
if [[ "$COMPARE_RC" -ne 0 ]]; then exit "$COMPARE_RC"; fi
if [[ "$CAMERA_RECORDER_RC" -ne 0 ]]; then exit "$CAMERA_RECORDER_RC"; fi
if [[ "$CAMERA_TRANSPORT_RC" -ne 0 ]]; then exit "$CAMERA_TRANSPORT_RC"; fi
if [[ "$CAMERA_FETCH_RC" -ne 0 ]]; then exit "$CAMERA_FETCH_RC"; fi
if [[ "$CAMERA_EVIDENCE_RC" -ne 0 ]]; then exit "$CAMERA_EVIDENCE_RC"; fi
if [[ "$REMOTE_FETCH_RC" -ne 0 ]]; then exit "$REMOTE_FETCH_RC"; fi
exit "$FINALIZE_RC"
