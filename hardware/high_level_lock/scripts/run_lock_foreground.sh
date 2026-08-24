#!/usr/bin/env bash
# Chạy bản cô lập THAY CHO hb_high_level, rồi trả service về chỗ cũ.
#
# hb_high_level là publisher rt/lowcmd duy nhất (D003). Hai bản chạy song song
# là đúng thứ D003 cấm, nên script này dừng service trước khi chạy và bật lại
# khi thoát — kể cả khi thoát vì Ctrl+C hay vì crash. Robot không được để lâu ở
# trạng thái không có ai làm chủ rt/lowcmd.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
LOCK_DIR="$(pwd)"
LOG_DIR="$LOCK_DIR/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$(date -u +%Y%m%dT%H%M%SZ)_run_lock.log"

[[ -x ./build/run_r1 ]] || { echo "[FAIL] Chưa build: chạy ./scripts/build.sh" >&2; exit 2; }

if [[ "${CONFIRM_SUSPENDED_WITH_ESTOP:-0}" != "1" ]]; then
    if [[ ! -t 0 ]]; then
        echo "[FAIL] Cần terminal tương tác hoặc CONFIRM_SUSPENDED_WITH_ESTOP=1." >&2
        exit 2
    fi
    echo "Bản này KHOÁ CỨNG chân và eo tại tư thế lúc bóp cò. Chỉ đúng khi robot đang treo."
    read -r -p "Robot đã treo/cố định và có người giữ E-stop? Nhập YES: " confirmation
    [[ "$confirmation" == "YES" ]] || { echo "[SAFE] Hủy; không đụng service." >&2; exit 2; }
fi

restore() {
    echo ""
    echo "[RESTORE] Bật lại hb_high_level..."
    sudo systemctl start hb_high_level || echo "[WARN] Bật lại THẤT BẠI — chạy tay: sudo systemctl start hb_high_level" >&2
    systemctl is-active hb_high_level || true
}
trap restore EXIT

echo "[STOP] Dừng hb_high_level để nhường quyền rt/lowcmd..."
sudo systemctl stop hb_high_level
sleep 1
if pgrep -f 'high_level_2/build/run_r1' >/dev/null; then
    echo "[FAIL] high_level_2 còn chạy; không khởi động bản thứ hai." >&2
    exit 3
fi

echo "[RUN] $LOCK_DIR/build/run_r1  (log: $LOG)"
echo "[RUN] Trên tay cầm R3: L2+R2 -> Dev Mode, giữ R1+R2 3 giây -> armed, L2+Y -> ZERO TORQUE."
echo "[RUN] Ctrl+C để dừng; service sẽ tự bật lại."
./build/run_r1 2>&1 | tee "$LOG"
