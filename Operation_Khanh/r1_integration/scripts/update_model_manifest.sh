#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
HIGH_DIR="$HB_ROOT/high_level_2"
MANIFEST="$HB_ROOT/r1_integration/config/model_manifest.conf"
SELECTED="$(awk -F: '$1 ~ /^[[:space:]]*flat_model[[:space:]]*$/ {v=$2; sub(/#.*/, "", v); gsub(/[[:space:]\"]/, "", v); print v; exit}' "$HIGH_DIR/config/tuning.yaml")"
MODEL="$HIGH_DIR/policies/flat/$SELECTED"
HIGH_BIN="${HIGH_BIN:-}"
CHECK_BUILD=""

cleanup() {
    if [[ -n "$CHECK_BUILD" && -d "$CHECK_BUILD" && \
          "$CHECK_BUILD" == /tmp/hb-model-check.* ]]; then
        rm -rf -- "$CHECK_BUILD"
    fi
}
trap cleanup EXIT

[[ -s "$MODEL" ]] || { echo "Model not found: $MODEL" >&2; exit 1; }
SHA="$(sha256sum "$MODEL" | awk '{print $1}')"

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
HB_PROJECT_DIR="$HIGH_DIR" "$HIGH_BIN" --preflight >/dev/null

if [[ "${1:-}" != "--accept" ]]; then
    echo "Model passed run_r1 preflight. Review then run: $0 --accept"
    echo "MODEL_REL=policies/flat/$SELECTED"
    echo "MODEL_SHA256=$SHA"
    exit 0
fi

TMP="$(mktemp /tmp/hb-model-manifest.XXXXXX)"
{
    echo "# Walking policy accepted by the current high_level_2 runner."
    echo "MODEL_REL=policies/flat/$SELECTED"
    echo "MODEL_SHA256=$SHA"
    echo "MODEL_INPUT=83"
    echo "MODEL_OUTPUT=24"
} >"$TMP"
mv "$TMP" "$MANIFEST"
echo "Updated $MANIFEST"
