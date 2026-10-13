#!/usr/bin/env bash
# Dance policy folders: list in-use vs unreferenced, archive unreferenced ones,
# restore an archived one. `./hb dance list|prune|restore` calls this.
# Prune is a dry run unless --apply is given.
set -euo pipefail

HB_ROOT="${HB_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
HIGH_DIR="$HB_ROOT/controller"
DANCE_DIR="$HIGH_DIR/policies/dance"
DANCE_ARCHIVE="$HB_ROOT/_archive/dance"

die() { echo "dance_assets: $*" >&2; exit 2; }

# Dance folders that must stay: named in config/dance.yaml, or pinned by a
# path in CMakeLists/tests/src (e.g. the ONNX contract test model).
dance_referenced() {
    local name="$1"
    grep -Eq "^dance_[0-9]+:[[:space:]]*${name}[[:space:]]*(#.*)?$" "$HIGH_DIR/config/dance.yaml" && return 0
    grep -rqF "policies/dance/$name/" "$HIGH_DIR/CMakeLists.txt" "$HIGH_DIR/tests" "$HIGH_DIR/src" 2>/dev/null
}

dance_unreferenced() {
    local dir name
    for dir in "$DANCE_DIR"/*/; do
        [[ -d "$dir" ]] || continue
        name="$(basename "$dir")"
        [[ "$name" == backup ]] && continue
        dance_referenced "$name" || printf '%s\n' "$name"
    done
}

sub="${1:-list}"
[[ $# -gt 0 ]] && shift
apply=0 count=0
case "$sub" in
    list)
        for dir in "$DANCE_DIR"/*/; do
            name="$(basename "$dir")"
            [[ "$name" == backup ]] && continue
            if dance_referenced "$name"; then
                printf 'in-use  %6s  %s\n' "$(du -sh "$dir" | cut -f1)" "$name"
            fi
        done
        echo "unreferenced: $(dance_unreferenced | wc -l) (./hb dance prune để xem chi tiết)"
        ;;
    prune)
        [[ "${1:-}" == "--apply" ]] && apply=1
        while IFS= read -r name; do
            count=$((count + 1))
            if (( apply )); then
                mkdir -p "$DANCE_ARCHIVE"
                [[ -e "$DANCE_ARCHIVE/$name" ]] && die "$DANCE_ARCHIVE/$name already exists"
                mv -- "$DANCE_DIR/$name" "$DANCE_ARCHIVE/$name"
                echo "archived  $name"
            else
                printf 'would archive  %6s  %s\n' "$(du -sh "$DANCE_DIR/$name" | cut -f1)" "$name"
            fi
        done < <(dance_unreferenced)
        if (( ! apply )); then
            echo "$count folder(s). Chạy lại với --apply để chuyển sang _archive/dance/."
            echo "Lưu ý: lần deploy dance/deploy kế tiếp mirror policies/dance với --delete,"
            echo "nên các folder này cũng bị xoá khỏi robot. Chỉ './hb deploy' (đầy đủ) tạo"
            echo "tarball ~/HB_backups trên robot trước khi sync."
        fi
        ;;
    restore)
        name="${1:-}"
        [[ -n "$name" && "$name" != */* ]] || die "usage: $0 restore <name>"
        [[ -d "$DANCE_ARCHIVE/$name" ]] || die "not archived: $name"
        [[ -e "$DANCE_DIR/$name" ]] && die "already present: $DANCE_DIR/$name"
        mv -- "$DANCE_ARCHIVE/$name" "$DANCE_DIR/$name"
        echo "restored  $name"
        ;;
    *)  die "usage: $0 [list|prune [--apply]|restore <name>]" ;;
esac
