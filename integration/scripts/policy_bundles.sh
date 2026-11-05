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
  ./hb policy use <tên> [--teleop]                       đổi walk_policy (hoặc teleop_policy) rồi preflight
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
        r1_armfree0917_multigait_h4_v2) ;;
        *) echo "contract khong ho tro: $contract" >&2; exit 1 ;;
    esac
    # Chu ky cua multigait V2 do chinh policy phat ra moi buoc (action index
    # 24), khong co mot so train nao de ghi vao bundle. Dong bo voi
    # FlatPolicyProfileHasFixedGaitPeriod() trong controller.
    local fixed_period=1
    [[ "$contract" != r1_armfree0917_multigait_h4_v2 ]] || fixed_period=0
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
    (( fixed_period == 0 )) || [[ -n "$period" ]] || {
        echo "khong lay duoc gait_period_s tu $src (metadata thieu hoac khong doc duoc)." \
             "Them --gait-period <giay> neu biet chac chu ky luc train." >&2; exit 1; }
    (( fixed_period == 1 )) || [[ -z "$period" ]] || {
        echo "contract $contract khong co chu ky gait co dinh: bo --gait-period." >&2; exit 1; }

    mkdir -p "$dest"
    cp -- "$src" "$dest/policy.onnx"
    local sha
    sha="$(sha256sum "$dest/policy.onnx" | awk '{print $1}')"
    cat >"$dest/policy.yaml" <<EOF
# Locomotion policy bundle — chọn bằng walk_policy / teleop_policy trong
# controller/config/locomotion.yaml. run_r1 kiểm sha256 và metadata ONNX lúc khởi động.
contract: $contract
sha256: $sha
EOF
    if (( fixed_period == 1 )); then
        cat >>"$dest/policy.yaml" <<EOF
# Bản sao metadata ONNX cùng tên, dập lúc tạo bundle. Runtime vẫn đối chiếu lại
# với ONNX, nên sửa tay ở đây là khởi động chết chứ không đổi được hành vi.
gait_period_s: $period
EOF
    else
        cat >>"$dest/policy.yaml" <<EOF
# Contract này không có chu kỳ gait cố định: policy tự phát chu kỳ mỗi bước qua
# action index 24, nên không có gait_period_s để chép từ metadata.
EOF
    fi
    [[ -z "$settle" || "$contract" != flat_plus_gait_h4_v1 ]] || \
        printf 'gait_settle_dwell_s: %s\n' "$settle" >>"$dest/policy.yaml"
    # Nguon term phase: hai bundle cung gait_fsm_contract co the khac nhau o
    # day, nen controller doi bundle khai chu khong suy tu metadata.
    if [[ "$contract" == flat_plus_gait_h4_v1 ]]; then
        if grep -q '^gait_phase_contract=' <<<"${meta:-}"; then
            printf 'gait_phase_source: command_gated\n' >>"$dest/policy.yaml"
        else
            printf 'gait_phase_source: episode_clock\n' >>"$dest/policy.yaml"
        fi
    fi
    cat >>"$dest/policy.yaml" <<EOF
# Nguồn: $(basename "$src"), thêm ngày $(date +%Y-%m-%d). Ghi provenance huấn luyện
# (nhánh, run, checkpoint, đánh giá) vào README.md cạnh file này.
EOF
    if (( fixed_period == 1 )); then
        echo "Da tao $dest (gait_period_s=$period${settle:+, gait_t_settle_s=$settle})"
    else
        echo "Da tao $dest (contract $contract: chu ky do policy phat, khong ghi gait_period_s)"
    fi
    echo "Tiep theo:"
    echo "  1. Ghi provenance vao $dest/README.md"
    echo "  2. Sua controller/config/locomotion.yaml: walk_policy: $name  (hoac teleop_policy: $name)"
    echo "  3. ./hb deploy   (may dev)"
}

# Đổi policy đang chọn. Chu kỳ, dwell và manifest đi theo bundle, nên đây là
# toàn bộ thao tác: sửa một dòng rồi chứng minh bằng preflight thật.
cmd_use() {
    local name="" role=walk_policy
    while (($#)); do
        case "$1" in
            --teleop) role=teleop_policy; shift ;;
            --walk)   role=walk_policy; shift ;;
            -*) usage >&2; exit 2 ;;
            *) [[ -z "$name" ]] || { usage >&2; exit 2; }; name="$1"; shift ;;
        esac
    done
    [[ -n "$name" ]] || { usage >&2; exit 2; }
    hb_valid_policy_bundle_name "$name" || {
        echo "ten bundle khong hop le: '$name'" >&2; exit 1; }
    local dir="$LOCO_DIR/$name"
    [[ -f "$dir/policy.onnx" && -f "$dir/policy.yaml" ]] || {
        echo "khong co bundle '$name' ($dir thieu policy.onnx hoac policy.yaml)" >&2
        echo "xem danh sach: ./hb policy list" >&2; exit 1; }
    local contract period actual expected
    contract="$(hb_config_get "$dir/policy.yaml" contract)"
    period="$(hb_config_get "$dir/policy.yaml" gait_period_s)"
    expected="$(hb_config_get "$dir/policy.yaml" sha256)"
    [[ -n "$contract" ]] || { echo "bundle '$name' thieu contract" >&2; exit 1; }
    # Dong bo voi FlatPolicyProfileHasFixedGaitPeriod() va cmd_add: contract ma
    # policy tu phat chu ky moi buoc khong co so nao de khai.
    if [[ "$contract" == r1_armfree0917_multigait_h4_v2 ]]; then
        [[ -z "$period" ]] || {
            echo "contract $contract khong co chu ky gait co dinh nhung policy.yaml khai" \
                 "gait_period_s: $period" >&2; exit 1; }
        period="(do policy phat)"
    else
        [[ -n "$period" ]] || {
            echo "bundle '$name' thieu gait_period_s trong policy.yaml; runtime se tu choi." \
                 "Them bang: ./hb policy add ... --gait-period <giay>" >&2; exit 1; }
    fi
    actual="$(sha256sum "$dir/policy.onnx" | awk '{print $1}')"
    [[ "$actual" == "$expected" ]] || {
        echo "bundle '$name': sha256 trong policy.yaml ($expected) khac file onnx ($actual)" >&2
        exit 1; }

    local config="$HIGH_DIR/config/locomotion.yaml" previous
    previous="$(hb_config_get "$config" "$role")"
    if grep -qE "^${role}:" "$config"; then
        sed -i "s|^${role}:.*|${role}: ${name}|" "$config"
    else
        printf '%s: %s\n' "$role" "$name" >>"$config"
    fi
    echo "$role: ${previous:-<trong>} -> $name  (contract $contract, gait_period_s $period)"

    # Manifest sinh lai tu runtime, khong suy ra o day. ./hb deploy cung lam
    # buoc nay, chay truoc de loi hien ra ngay luc doi chu khong luc deploy.
    if ! bash "$HB_ROOT/integration/scripts/update_model_manifest.sh" --accept; then
        echo "preflight that bai; tra lai $role: ${previous:-<trong>}" >&2
        if [[ -n "$previous" ]]; then
            sed -i "s|^${role}:.*|${role}: ${previous}|" "$config"
        else
            sed -i "/^${role}:/d" "$config"
        fi
        exit 1
    fi
    echo "Tiep theo: ./hb test cpp, roi ./hb deploy (may dev)"
}

case "${1:-help}" in
    list) shift; cmd_list "$@" ;;
    add)  shift; cmd_add "$@" ;;
    use)  shift; cmd_use "$@" ;;
    help|-h|--help) usage ;;
    *) usage >&2; exit 2 ;;
esac
