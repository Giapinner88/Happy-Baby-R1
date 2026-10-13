#!/usr/bin/env bash
# Ported R1 mapping and hardware transport now differ deliberately from source.
# This command is a read-only comparison; apply selected changes by review.
set -euo pipefail

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--dry-run" ) ]]; then
    echo "Usage: $0 [--dry-run] (comparison only; no automatic source replacement)" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="${HB_WORKSPACE:-$(cd "$TELEOP_DIR/../.." && pwd)/Happy-Baby-R1}"

[[ -d "$WORKSPACE/teleop" ]] || {
    echo "[FAIL] Không thấy $WORKSPACE/teleop; đặt HB_WORKSPACE rõ ràng." >&2
    exit 1
}
[[ -d "$TELEOP_DIR/src/teleop" ]] || {
    echo "[FAIL] Thiếu bản HB teleop/src/teleop để đối chiếu." >&2
    exit 1
}

echo "[INFO] So sánh chỉ đọc $WORKSPACE/teleop -> $TELEOP_DIR/src/teleop"
rsync -ani --exclude '__pycache__/' --exclude '*.pyc' --exclude 'hardware/' \
    "$WORKSPACE/teleop/" "$TELEOP_DIR/src/teleop/"
echo "[INFO] Không thay đổi file nào. Hãy review từng phần port trước khi chép."
