#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HIGH_DIR="$HB_ROOT/controller"
MANIFEST="$HB_ROOT/integration/config/model_manifest.conf"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/config_reader.sh"
source "$SCRIPT_DIR/status_log.sh"
SELECTED="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_model)"
CONTRACT="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_policy_contract)"
[[ -n "$SELECTED" && "$SELECTED" != */* && "$SELECTED" != "." && "$SELECTED" != ".." ]] || {
    echo "flat_model must name a file, not a path: '$SELECTED'" >&2
    exit 1
}
HIGH_BIN="${HIGH_BIN:-}"
CHECK_BUILD=""
PREFLIGHT_LOG=""
KEEP_PREFLIGHT_LOG=0

cleanup() {
    if [[ -n "$PREFLIGHT_LOG" && "$KEEP_PREFLIGHT_LOG" != "1" ]]; then
        rm -f -- "$PREFLIGHT_LOG"
    fi
    if [[ -n "$CHECK_BUILD" && -d "$CHECK_BUILD" && \
          "$CHECK_BUILD" == /tmp/hb-model-check.* ]]; then
        rm -rf -- "$CHECK_BUILD"
    fi
}
trap cleanup EXIT


if [[ -z "$HIGH_BIN" ]]; then
    CHECK_BUILD="$(mktemp -d /tmp/hb-model-check.XXXXXX)"
    cmake -S "$HIGH_DIR" -B "$CHECK_BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$CHECK_BUILD" --target run_r1 --parallel "$(nproc)" >/dev/null
    HIGH_BIN="$CHECK_BUILD/run_r1"
fi

ARCH="$(uname -m)"
if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
    ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime_aarch64/lib"
else
    ORT_LIB="$HIGH_DIR/thirdparty/onnxruntime/lib"
fi
export LD_LIBRARY_PATH="$ORT_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Đường dẫn model và số chiều obs lấy TỪ RUNTIME, không suy ra ở đây. Bảng
# contract cũ trong script này (và trong preflight.sh) đã từng lệch với
# FlatPolicyProfile.hpp cả hai chiều, và cả hai đều ép model nằm dưới
# policies/flat trong khi non-legacy nạp từ policies/locomotion/flat_plus.
PREFLIGHT_LOG="$(mktemp /tmp/hb-model-preflight.XXXXXX)"
HB_PROJECT_DIR="$HIGH_DIR" "$HIGH_BIN" --preflight >"$PREFLIGHT_LOG" 2>&1 || {
    KEEP_PREFLIGHT_LOG=1
    hb_fail "run_r1 preflight failed; details: $PREFLIGHT_LOG; manifest unchanged"
    exit 1
}
MODEL_LINE="$(grep -m1 '^\[Preflight\] MODEL ' "$PREFLIGHT_LOG" || true)"
[[ -n "$MODEL_LINE" ]] || {
    echo "run_r1 --preflight khong in dong '[Preflight] MODEL ...' (binary cu hon script?)" >&2
    exit 1
}
MODEL_REL="$(sed -n 's/.* rel=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
MODEL_INPUT="$(sed -n 's/.* input=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
MODEL_OUTPUT="$(sed -n 's/.* output=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
RUNTIME_CONTRACT="$(sed -n 's/.* contract=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
[[ "$RUNTIME_CONTRACT" == "$CONTRACT" ]] || {
    echo "runtime nap contract=$RUNTIME_CONTRACT nhung tuning chon $CONTRACT" >&2
    exit 1
}
[[ "$MODEL_REL" == */"$SELECTED" ]] || {
    echo "runtime nap $MODEL_REL nhung flat_model=$SELECTED" >&2
    exit 1
}
MODEL="$HIGH_DIR/$MODEL_REL"
[[ -f "$MODEL" && -s "$MODEL" ]] || { echo "Model not found or not a regular file: $MODEL" >&2; exit 1; }
SHA="$(sha256sum "$MODEL" | awk '{print $1}')"

if [[ "${1:-}" != "--accept" ]]; then
    echo "Model passed run_r1 preflight. Review then run: $0 --accept"
    echo "MODEL_REL=$MODEL_REL"
    echo "MODEL_SHA256=$SHA"
    echo "MODEL_CONTRACT=$CONTRACT"
    echo "MODEL_INPUT=$MODEL_INPUT"
    echo "MODEL_OUTPUT=$MODEL_OUTPUT"
    exit 0
fi

TMP="$(mktemp /tmp/hb-model-manifest.XXXXXX)"
{
    echo "# Walking policy accepted by the current controller runner."
    echo "MODEL_REL=$MODEL_REL"
    echo "MODEL_SHA256=$SHA"
    echo "MODEL_CONTRACT=$CONTRACT"
    echo "MODEL_INPUT=$MODEL_INPUT"
    echo "MODEL_OUTPUT=$MODEL_OUTPUT"
} >"$TMP"
mv "$TMP" "$MANIFEST"
hb_ok "locomotion model manifest updated"
