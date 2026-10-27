#!/usr/bin/env bash
# Locomotion policy bundles: controller/policies/locomotion/<name>/{policy.onnx,policy.yaml}.
#   ./hb policy list
#   ./hb policy add <file.onnx> <name> [--contract C]
# `add` only creates the bundle; selecting it stays an explicit edit of
# walk_policy / teleop_policy in controller/config/locomotion.yaml.
set -euo pipefail

HB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HIGH_DIR="$HB_ROOT/controller"
LOCO_DIR="$HIGH_DIR/policies/locomotion"
CONFIG="$HIGH_DIR/config/tuning.yaml"
# shellcheck disable=SC1091
source "$HB_ROOT/integration/scripts/config_reader.sh"

usage() {
    cat <<'USAGE'
Usage:
  ./hb policy list                                       liệt kê bundle, đánh dấu walk/teleop đang chọn
  ./hb policy add <file.onnx> <tên> [--contract C]
                                                         tên hoặc nhóm/tên; tạo policies/locomotion/<tên>/ (mặc định
                                                         contract flat_plus_gait_h4_v1)
USAGE
}

cmd_list() {
    local walk teleop yaml dir name contract h4 mark
    walk="$(hb_config_get "$CONFIG" walk_policy)"
    teleop="$(hb_config_get "$CONFIG" teleop_policy)"
    [[ -n "$teleop" ]] || teleop="$walk"
    [[ -n "$walk" ]] || echo "(config dang dung route cu flat_model: $(hb_config_get "$CONFIG" flat_model))"
    printf '%-40s %-24s %-9s %s\n' "BUNDLE" "CONTRACT" "H4" "DANG CHON"
    while IFS= read -r -d '' yaml; do
        dir="${yaml%/policy.yaml}"
        name="${dir#"$LOCO_DIR/"}"
        contract="$(hb_config_get "$dir/policy.yaml" contract 2>/dev/null || echo '?')"
        # H4 theo contract (SupportsH4TeleopOverlay trong controller), không theo bundle.
        case "$contract" in
            flat_plus_gait_h4_v1|r1_wb_legs_v1_blind) h4=yes ;;
            *) h4=no ;;
        esac
        mark=""
        [[ "$name" == "$walk" ]] && mark+="walk(R2+A) "
        [[ "$name" == "$teleop" ]] && mark+="teleop(R2+X)"
        printf '%-40s %-24s %-9s %s\n' "$name" "${contract:-?}" "$h4" "$mark"
    done < <(find "$LOCO_DIR" -type d -name '_*' -prune -o -type f -name policy.yaml -print0 | sort -z)
}

cmd_add() {
    local src="" name="" contract="flat_plus_gait_h4_v1"
    while (( $# )); do
        case "$1" in
            --contract) contract="${2:?--contract needs a value}"; shift 2 ;;
            -*) usage >&2; exit 2 ;;
            *)
                if [[ -z "$src" ]]; then src="$1"
                elif [[ -z "$name" ]]; then name="$1"
                else usage >&2; exit 2; fi
                shift ;;
        esac
    done
    [[ -n "$src" && -n "$name" ]] || { usage >&2; exit 2; }
    [[ -s "$src" && "$src" == *.onnx ]] || { echo "khong phai file .onnx: $src" >&2; exit 1; }
    hb_valid_policy_bundle_name "$name" || {
        echo "ten bundle hoac nhom/ten: moi thanh phan bat dau bang chu/so, chi gom chu, so, . _ -: '$name'" >&2; exit 1; }
    case "$contract" in
        legacy_83|flat_plus_h4_v1|flat_plus_h5_v1|flat_plus_gait_h4_v1|r1_wb_legs_v1_blind) ;;
        *) echo "contract khong ho tro: $contract" >&2; exit 1 ;;
    esac
    local dest="$LOCO_DIR/$name"
    [[ ! -e "$dest" ]] || { echo "da ton tai: $dest" >&2; exit 1; }

    mkdir -p "$dest"
    cp -- "$src" "$dest/policy.onnx"
    local sha
    sha="$(sha256sum "$dest/policy.onnx" | awk '{print $1}')"
    cat >"$dest/policy.yaml" <<EOF
# Locomotion policy bundle — chọn bằng walk_policy / teleop_policy trong
# controller/config/locomotion.yaml. run_r1 kiểm sha256 và metadata ONNX lúc khởi động.
contract: $contract
sha256: $sha
# Nguồn: $(basename "$src"), thêm ngày $(date +%Y-%m-%d). Ghi provenance huấn luyện
# (nhánh, run, checkpoint, đánh giá) vào README.md cạnh file này.
EOF
    echo "Da tao $dest"
    echo "Tiep theo:"
    echo "  1. Ghi provenance vao $dest/README.md"
    echo "  2. Sua controller/config/locomotion.yaml: walk_policy: $name  (hoac teleop_policy: $name)"
    echo "  3. ./hb deploy   (may dev)"
}

case "${1:-help}" in
    list) shift; cmd_list "$@" ;;
    add)  shift; cmd_add "$@" ;;
    help|-h|--help) usage ;;
    *) usage >&2; exit 2 ;;
esac
