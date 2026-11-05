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

# run_r1 --policy-meta in ra metadata cần cho policy.yaml. Build tăng dần nếu
# chưa có binary; không build lại từ đầu trong /tmp.
hb_policy_onnx_meta() {
    local onnx="$1" binary="${HIGH_BIN:-$HIGH_DIR/build/run_r1}" ort arch
    if [[ ! -x "$binary" ]]; then
        [[ -f "$HIGH_DIR/build/CMakeCache.txt" ]] || \
            cmake -S "$HIGH_DIR" -B "$HIGH_DIR/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
        cmake --build "$HIGH_DIR/build" --target run_r1 --parallel "$(nproc)" >/dev/null
        binary="$HIGH_DIR/build/run_r1"
    fi
    arch="$(uname -m)"
    if [[ "$arch" == aarch64 || "$arch" == arm64 ]]; then
        ort="$HIGH_DIR/thirdparty/onnxruntime_aarch64/lib"
    else
        ort="$HIGH_DIR/thirdparty/onnxruntime/lib"
    fi
    LD_LIBRARY_PATH="$ort${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$binary" --policy-meta "$onnx"
}

usage() {
    cat <<'USAGE'
Usage:
  ./hb policy list                                       liệt kê bundle, đánh dấu walk/teleop đang chọn
  ./hb policy add <file.onnx> <tên> [--contract C]
                   [--gait-period S] [--settle-dwell S]
                                                         tên hoặc nhóm/tên; tạo policies/locomotion/<tên>/ (mặc định
                                                         contract flat_plus_gait_h4_v1). Chu kỳ và dwell đọc từ
                                                         metadata ONNX; chỉ khai tay khi không đọc được.
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
    local src="" name="" contract="flat_plus_gait_h4_v1" period="" settle=""
    while (( $# )); do
        case "$1" in
            --contract) contract="${2:?--contract needs a value}"; shift 2 ;;
            --gait-period) period="${2:?--gait-period needs a value}"; shift 2 ;;
            --settle-dwell) settle="${2:?--settle-dwell needs a value}"; shift 2 ;;
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
        legacy_83|flat_plus_h4_v1|flat_plus_h5_v1|flat_plus_gait_h4_v1|r1_wb_legs_v1_blind|r1_wb_legs_gait_h4_v1_blind) ;;
        *) echo "contract khong ho tro: $contract" >&2; exit 1 ;;
    esac
    local dest="$LOCO_DIR/$name"
    [[ ! -e "$dest" ]] || { echo "da ton tai: $dest" >&2; exit 1; }

    # Chu kỳ phase và dwell phải nằm trong bundle: locomotion.yaml không còn khai
    # chúng, nên bundle thiếu là run_r1 từ chối khởi động. Shell không đọc được
    # ONNX nên hỏi chính run_r1; --gait-period/--settle-dwell chỉ để vá khi
    # không đọc được. Giá trị sai không trôi được: runtime đối chiếu lại metadata.
    local meta
    if [[ -z "$period" ]] && meta="$(hb_policy_onnx_meta "$src" 2>/dev/null)"; then
        period="$(sed -n 's/^gait_period_s=//p' <<<"$meta")"
        [[ -n "$settle" ]] || settle="$(sed -n 's/^gait_t_settle_s=//p' <<<"$meta")"
    fi
    [[ -n "$period" ]] || {
        echo "khong lay duoc gait_period_s tu $src (metadata thieu hoac khong doc duoc)." \
             "Them --gait-period <giay> neu biet chac chu ky luc train." >&2; exit 1; }

    mkdir -p "$dest"
    cp -- "$src" "$dest/policy.onnx"
    local sha
    sha="$(sha256sum "$dest/policy.onnx" | awk '{print $1}')"
    cat >"$dest/policy.yaml" <<EOF
# Locomotion policy bundle — chọn bằng walk_policy / teleop_policy trong
# controller/config/locomotion.yaml. run_r1 kiểm sha256 và metadata ONNX lúc khởi động.
contract: $contract
sha256: $sha
# Bản sao metadata ONNX cùng tên, dập lúc tạo bundle. Runtime vẫn đối chiếu lại
# với ONNX, nên sửa tay ở đây là khởi động chết chứ không đổi được hành vi.
gait_period_s: $period
EOF
    [[ -z "$settle" || "$contract" != flat_plus_gait_h4_v1 ]] || \
        printf 'gait_settle_dwell_s: %s\n' "$settle" >>"$dest/policy.yaml"
    cat >>"$dest/policy.yaml" <<EOF
# Nguồn: $(basename "$src"), thêm ngày $(date +%Y-%m-%d). Ghi provenance huấn luyện
# (nhánh, run, checkpoint, đánh giá) vào README.md cạnh file này.
EOF
    echo "Da tao $dest (gait_period_s=$period${settle:+, gait_t_settle_s=$settle})"
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
