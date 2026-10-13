#!/usr/bin/env bash
# Kiểm tra tĩnh trước khi deploy. Chạy được cả ở máy dev lẫn trên robot.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export PYTHONPATH="$TELEOP_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
    GREEN=$'\033[32m'; RED=$'\033[31m'; YELLOW=$'\033[33m'; RESET=$'\033[0m'
else
    GREEN=; RED=; YELLOW=; RESET=
fi

if [[ -r /etc/hb/teleop.env ]]; then
    set -a; source /etc/hb/teleop.env; set +a
fi

fail() { printf '%s[FAIL]%s %s\n' "$RED" "$RESET" "$*" >&2; exit 1; }
ok()   { printf '%s[OK]%s %s\n' "$GREEN" "$RESET" "$*"; }

[[ -d "$TELEOP_DIR/src/teleop" ]] || fail "Chưa có src/teleop. Chạy scripts/sync_from_workspace.sh trước."
[[ -f "$TELEOP_DIR/src/SOURCE.txt" ]] || fail "Thiếu src/SOURCE.txt: không truy được nguồn của bản sync."
ok "src/teleop có mặt ($(grep '^commit:' "$TELEOP_DIR/src/SOURCE.txt" | awk '{print $2}'))"

# Unit file starts this module.  Do not report a deploy as ready when only the
# simulation-owned IK package has been synced and the hardware adapter is still
# absent.
HARDWARE_ENTRY="$TELEOP_DIR/src/teleop/hardware/run_teleop.py"
[[ -f "$HARDWARE_ENTRY" ]] || fail \
    "Thiếu runtime hardware teleop.hardware.run_teleop; package hiện chỉ có logic mô phỏng và chưa thể chạy service trên robot."

if [[ -z "${PYTHON:-}" ]]; then
    if [[ -x /usr/bin/python3 ]]; then
        PYTHON=/usr/bin/python3
    else
        PYTHON=python3
    fi
fi
export PYTHON
export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
"$PYTHON" - "$TELEOP_DIR" <<'PY' || exit 1
import ast, pathlib, sys
root = pathlib.Path(sys.argv[1]) / "src" / "teleop"
bad = []
paths = sorted(root.rglob("*.py")) + sorted((root.parents[1] / "scripts").rglob("*.py"))
for path in paths:
    try:
        ast.parse(path.read_text(encoding="utf-8"))
    except SyntaxError as exc:
        bad.append(f"{path}: {exc}")
if bad:
    print("[FAIL] Lỗi cú pháp:\n  " + "\n  ".join(bad), file=sys.stderr)
    sys.exit(1)
print(f"[OK] {len(paths)} file Python parse được")
PY

"$PYTHON" "$SCRIPT_DIR/load_teleop_config.py" --config "$TELEOP_DIR/config/teleop.yaml" >/dev/null
ok "teleop.yaml parse/ports/camera timing hợp lệ"

# Cổng an toàn: mặc định fail-closed.
ALLOW="${HB_TELEOP_ALLOW_MOTOR_WRITE:-0}"
if [[ "$ALLOW" == "1" ]]; then
    printf '%s[WARN]%s HB_TELEOP_ALLOW_MOTOR_WRITE=1 -> teleop ĐƯỢC PHÉP ghi xuống motor.\n' "$YELLOW" "$RESET" >&2
    printf '%s[WARN]%s Chỉ chạy khi có người giữ E-stop và đã đóng docs/hardware_gate.md.\n' "$YELLOW" "$RESET" >&2
else
    ok "Cổng motor đang đóng (HB_TELEOP_ALLOW_MOTOR_WRITE=0)"
fi

# Không nuốt stderr: "tests/ không đạt" mà không kèm lý do thì không sửa được.
# Thiếu pytest trong interpreter đang dùng là lỗi MÔI TRƯỜNG, không phải test hỏng.
if ! "$PYTHON" -c 'import pytest' 2>/dev/null; then
    fail "$PYTHON không có pytest. Chạy lại với PYTHON=/usr/bin/python3 ./scripts/preflight.sh
       hoặc cài: $PYTHON -m pip install pytest"
fi
if ! "$PYTHON" -m pytest "$TELEOP_DIR/tests" -q; then
    fail "tests/ không đạt (log pytest ở trên)"
fi
ok "tests/ đạt"
ok "preflight hoàn tất"
