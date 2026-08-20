#!/usr/bin/env bash
# Kiểm tra tĩnh trước khi deploy. Chạy được cả ở máy dev lẫn trên robot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -r /etc/hb/teleop.env ]]; then
    set -a; source /etc/hb/teleop.env; set +a
fi

fail() { echo "[FAIL] $*" >&2; exit 1; }
ok()   { echo "[OK] $*"; }

[[ -d "$TELEOP_DIR/src/teleop" ]] || fail "Chưa có src/teleop. Chạy scripts/sync_from_workspace.sh trước."
[[ -f "$TELEOP_DIR/src/SOURCE.txt" ]] || fail "Thiếu src/SOURCE.txt: không truy được nguồn của bản sync."
ok "src/teleop có mặt ($(grep '^commit:' "$TELEOP_DIR/src/SOURCE.txt" | awk '{print $2}'))"

# Unit file starts this module.  Do not report a deploy as ready when only the
# simulation-owned IK package has been synced and the hardware adapter is still
# absent.
HARDWARE_ENTRY="$TELEOP_DIR/src/teleop/hardware/run_teleop.py"
[[ -f "$HARDWARE_ENTRY" ]] || fail \
    "Thiếu runtime hardware teleop.hardware.run_teleop; package hiện chỉ có logic mô phỏng và chưa thể chạy service trên robot."

PYTHON="${PYTHON:-python3}"
"$PYTHON" - "$TELEOP_DIR" <<'PY' || exit 1
import ast, pathlib, sys
root = pathlib.Path(sys.argv[1]) / "src" / "teleop"
bad = []
for path in sorted(root.rglob("*.py")):
    try:
        ast.parse(path.read_text(encoding="utf-8"))
    except SyntaxError as exc:
        bad.append(f"{path}: {exc}")
if bad:
    print("[FAIL] Lỗi cú pháp:\n  " + "\n  ".join(bad), file=sys.stderr)
    sys.exit(1)
print(f"[OK] {len(list(root.rglob('*.py')))} file Python parse được")
PY

# Cổng an toàn: mặc định fail-closed.
ALLOW="${HB_TELEOP_ALLOW_MOTOR_WRITE:-0}"
if [[ "$ALLOW" == "1" ]]; then
    echo "[WARN] HB_TELEOP_ALLOW_MOTOR_WRITE=1 -> teleop ĐƯỢC PHÉP ghi xuống motor." >&2
    echo "[WARN] Chỉ chạy khi có người giữ E-stop và đã đóng docs/hardware_gate.md." >&2
else
    ok "Cổng motor đang đóng (HB_TELEOP_ALLOW_MOTOR_WRITE=0)"
fi

"$PYTHON" -m pytest "$TELEOP_DIR/tests" -q 2>/dev/null || fail "tests/ không đạt"
ok "tests/ đạt"
ok "preflight hoàn tất"
