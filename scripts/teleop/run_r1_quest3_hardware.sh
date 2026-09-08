#!/usr/bin/env bash
# Foreground Quest -> arms/head IK -> SSH sidecar -> sole high-level lowcmd owner.
#
# HB_TELEOP_SOLVER=upstream (default) solves with the unmodified vendor
# xr_teleoperate R1_A5_ArmIK in the `tv` environment, the same solver and the
# same process layout the simulation baseline runs. HB_TELEOP_SOLVER=coupled
# selects this repository's coupled IK instead, for comparison on one trace.
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

# Chuẩn hoá dáng robot về nominal của sim ngay sau khi bóp cò, trước khi teleop
# bám tay. Mặc định BẬT: đường phần cứng là phiên tương đối nên không có bước
# này thì robot giữ nguyên tư thế tay đang buông và không bao giờ tương ứng với
# sim. HB_TELEOP_HOME=0 để bỏ qua.
# Envelope mỗi khớp so với mốc phiên. 0.15 là giá trị cũ; phiên 2026-08-29 bão
# hoà nó trên 10/12 khớp nên mặc định lên 1.0 rad (57 độ). Trần sidecar là 1.0.
HB_TELEOP_MAX_OFFSET_RAD="${HB_TELEOP_MAX_OFFSET_RAD:-1.0}"
# Envelope riêng cho 6 khớp vai. 3.2 phủ trọn khớp rộng nhất (shoulder pitch
# ±3.142 trong asset), nên vai chạy hết tầm và giới hạn khớp của asset — do
# producer kẹp — là lớp chặn duy nhất còn lại. Người vận hành xác nhận an toàn
# trên giá treo ngày 2026-08-29.
HB_TELEOP_MAX_OFFSET_SHOULDER_RAD="${HB_TELEOP_MAX_OFFSET_SHOULDER_RAD:-3.2}"
HB_TELEOP_HOME="${HB_TELEOP_HOME:-1}"
HB_TELEOP_SOLVER="${HB_TELEOP_SOLVER:-upstream}"
case "$HB_TELEOP_SOLVER" in
    upstream|coupled) ;;
    *) echo "[FAIL] HB_TELEOP_SOLVER phải là 'upstream' hoặc 'coupled'." >&2; exit 2 ;;
esac
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

echo "[READY] Quest URL: https://$HOST_IP:8012/?ws=wss://$HOST_IP:8012"
echo "[READY] TRƯỚC CÒ PHẢI: đưa robot arms/head và người vận hành về neutral ban đầu như trong sim."
echo "[READY] Frame đầu tiên khi bóp cò phải được chốt làm source_zero của cả phiên."
echo "[READY] Giữ cò phải để điều khiển; nhả cò để receiver watchdog release và dừng."
echo "[READY] Evidence local: $RUN_DIR"
echo "[READY] Solver: $HB_TELEOP_SOLVER"
if [[ "$HB_TELEOP_HOME" == "1" ]]; then
    echo "[READY] Homing BẬT: sau khi bóp cò, robot TỰ gập khuỷu về nominal rồi mới bám tay bạn."
    echo "[READY] Chỉ CẲNG TAY đi: từ buông thõng lên ngang, hướng ra trước, quét quanh khuỷu ~0.16m."
    echo "[READY] Cánh tay trên và vai gần như không đổi. Kiểm chỗ trống trước hai cẳng tay."
else
    echo "[READY] Homing TẮT: robot giữ nguyên tư thế hiện tại làm mốc."
fi
echo "[READY] Envelope: ${HB_TELEOP_MAX_OFFSET_RAD} rad/khớp; VAI ${HB_TELEOP_MAX_OFFSET_SHOULDER_RAD} rad (hết tầm)."
echo "[READY] Cò trái = đưa về nominal và chốt lại mốc; nhả cò phải = tay giữ nguyên tư thế."

conda run --no-capture-output -n tv python scripts/teleop/quest_bridge.py \
    --host-ip "$HOST_IP" \
    --duration-s "$DURATION_S" \
    --frequency-hz 30 \
    --deadman-source right_trigger \
    --cert-file "$CERT_FILE" \
    --key-file "$KEY_FILE" \
    --stop-file "$STOP_FILE" \
    --connection-log "$RUN_DIR/bridge_connection.jsonl" \
| if [[ "$HB_TELEOP_SOLVER" == "upstream" ]]; then
    # The vendor solver needs CasADi and the Pinocchio 3 bindings, which exist
    # in `tv` and nowhere else here, so it stays a process of its own exactly as
    # it does in simulation. The robot side still receives joint angles only.
    conda run --no-capture-output -n tv python scripts/teleop/run_r1_upstream_ik_stream.py --passthrough
  else
    cat
  fi \
| conda run --no-capture-output -n unitree_sim_env python scripts/teleop/run_r1_quest3_hardware_targets.py \
    --duration-s "$DURATION_S" \
    --control-hz 10 \
    $([[ "$HB_TELEOP_SOLVER" == "upstream" ]] && echo --upstream-joint-stream || echo --coupled-ik) \
| ssh -o BatchMode=yes "$ROBOT" \
    "cd /home/unitree/HB/teleop && HB_TELEOP_ALLOW_HIGH_LEVEL_TELEOP=1 PYTHONPATH=/home/unitree/HB/teleop/src python3 -m teleop.hardware.high_level_sidecar --interface eth10 --udp-host 127.0.0.1 --udp-port 5560 --confirm-suspended-with-estop --confirm-dev-mode --duration-s '$DURATION_S' --first-input-timeout-s 120 --input-timeout-s 0.75 --state-timeout-s 0.20 --send-hz 100 --max-offset-rad $HB_TELEOP_MAX_OFFSET_RAD --max-offset-rad-shoulders $HB_TELEOP_MAX_OFFSET_SHOULDER_RAD $([[ "$HB_TELEOP_HOME" == "1" ]] && echo --home-to-nominal) --log-dir /home/unitree/HB/teleop/logs" \
| tee "$RUN_DIR/robot_receiver.log"
