#!/usr/bin/env bash
# Kéo mã teleop từ workspace Happy-Baby-R1 vào src/.
#
# Thuật toán IK/mapping có MỘT nguồn sự thật duy nhất là workspace. Package này
# chỉ chứa bản sao để deploy, không sửa trực tiếp. Sửa ở workspace rồi sync lại.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
# Package nằm NGAY TRONG workspace (hardware/teleop), nên nguồn suy ra từ vị trí
# của chính script này. HB_WORKSPACE chỉ dùng khi copy package ra ngoài repo.
WORKSPACE="${HB_WORKSPACE:-$(cd "$TELEOP_DIR/../.." && pwd)}"

[[ -d "$WORKSPACE/teleop" ]] || { echo "[FAIL] Không thấy $WORKSPACE/teleop" >&2; echo "       Đặt HB_WORKSPACE=/duong/dan/toi/Happy-Baby-R1" >&2; exit 1; }

mkdir -p "$TELEOP_DIR/src"
rsync -a --delete \
    --exclude '__pycache__/' --exclude '*.pyc' --exclude 'hardware/' \
    "$WORKSPACE/teleop/" "$TELEOP_DIR/src/teleop/"

# Ghi lại đúng commit đã sync: nếu không truy được nguồn thì bản deploy vô nghĩa.
COMMIT="$(git -C "$WORKSPACE" rev-parse HEAD 2>/dev/null || echo unknown)"
DIRTY="clean"
git -C "$WORKSPACE" diff --quiet 2>/dev/null || DIRTY="dirty"
cat > "$TELEOP_DIR/src/SOURCE.txt" <<META
workspace: $WORKSPACE
commit:    $COMMIT
worktree:  $DIRTY
synced_at: $(date -u +%Y-%m-%dT%H:%M:%SZ)
by:        $(id -un)@$(hostname)
META

echo "[OK] Đã sync teleop/ từ $WORKSPACE ($COMMIT, $DIRTY)"
[[ "$DIRTY" == "clean" ]] || echo "[WARN] Workspace đang có thay đổi chưa commit; bản sync này không tái tạo lại được." >&2
