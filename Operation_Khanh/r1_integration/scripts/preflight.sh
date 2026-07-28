#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-all}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
HIGH_DIR="$HB_ROOT/high_level_2"
VOICE_DIR="$HB_ROOT/voice_r1"
INTEGRATION_DIR="$HB_ROOT/r1_integration"

if [[ -r /etc/hb/stack.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source /etc/hb/stack.env
    set +a
fi

fail() { echo "[FAIL] $*" >&2; exit 1; }
ok() { echo "[OK] $*"; }
includes() { [[ "$MODE" == "all" || "$MODE" == "$1" ]]; }
TEMP_FILES=()
cleanup() {
    if ((${#TEMP_FILES[@]})); then
        rm -f "${TEMP_FILES[@]}"
    fi
}
trap cleanup EXIT

ARCH="$(uname -m)"
if [[ "${HB_ALLOW_NON_ARM64:-0}" != "1" ]]; then
    [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]] || fail "runtime architecture is $ARCH, expected ARM64"
fi
ok "architecture=$ARCH"

IFACE="${UNITREE_NETWORK_INTERFACE:-eth10}"
ip link show "$IFACE" >/dev/null 2>&1 || fail "network interface not found: $IFACE"
ok "network interface=$IFACE"

if includes high; then
    BIN="$HIGH_DIR/build/run_r1"
    [[ -x "$BIN" ]] || fail "missing executable: $BIN"
    if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
        file "$BIN" | grep -Eq 'ARM aarch64|ARM64' || fail "run_r1 is not ARM64"
    fi
    ! ldd "$BIN" 2>&1 | grep -q 'not found' || fail "run_r1 has unresolved shared libraries"

    # shellcheck disable=SC1091
    source "$INTEGRATION_DIR/config/model_manifest.conf"
    SELECTED="$(awk -F: '$1 ~ /^[[:space:]]*flat_model[[:space:]]*$/ {v=$2; sub(/#.*/, "", v); gsub(/[[:space:]\"]/, "", v); print v; exit}' "$HIGH_DIR/config/tuning.yaml")"
    [[ -n "$SELECTED" ]] || fail "flat_model is missing from tuning.yaml"
    [[ "$MODEL_REL" == "policies/flat/$SELECTED" ]] || fail "manifest=$MODEL_REL but tuning selects $SELECTED"
    MODEL="$HIGH_DIR/$MODEL_REL"
    [[ -s "$MODEL" ]] || fail "model missing: $MODEL"
    ACTUAL_SHA="$(sha256sum "$MODEL" | awk '{print $1}')"
    [[ "$ACTUAL_SHA" == "$MODEL_SHA256" ]] || fail "model SHA-256 differs from manifest"
    HIGH_PREFLIGHT_LOG="$(mktemp /tmp/hb-high-preflight.XXXXXX)"
    TEMP_FILES+=("$HIGH_PREFLIGHT_LOG")
    "$BIN" --preflight >"$HIGH_PREFLIGHT_LOG" 2>&1 || {
        tail -40 "$HIGH_PREFLIGHT_LOG" >&2
        fail "run_r1 model/asset preflight failed"
    }
    if grep -E '^[[:space:]]*voice_[^:]*:[[:space:]]*"?/home/' "$HIGH_DIR/config/tuning.yaml" >/dev/null; then
        fail "high-level voice paths still contain absolute /home paths"
    fi
    ok "high-level model manifest, assets and libraries"
fi

if includes integration; then
    BIN="$INTEGRATION_DIR/build/hb_integration"
    [[ -x "$BIN" ]] || fail "missing executable: $BIN"
    "$BIN" --self-test >/dev/null || fail "hb_integration self-test failed"
    FORBIDDEN_LOG="$(mktemp /tmp/hb-integration-forbidden.XXXXXX)"
    TEMP_FILES+=("$FORBIDDEN_LOG")
    if grep -R -n -E 'LowCmd|LocoClient|ChannelPublisher|robot_action' \
        --include='*.cpp' --include='*.hpp' --include='*.py' \
        "$INTEGRATION_DIR/src" >"$FORBIDDEN_LOG"; then
        cat "$FORBIDDEN_LOG" >&2
        fail "integration source contains a motor-control API"
    fi
    ok "integration is read-only and self-test passed"
fi

if includes voice; then
    PYTHON="$VOICE_DIR/.venv/bin/python"
    BRIDGE="${UNITREE_BRIDGE_PATH:-unitree_bridge/build/r1_bridge}"
    [[ "$BRIDGE" = /* ]] || BRIDGE="$VOICE_DIR/$BRIDGE"
    [[ -x "$PYTHON" ]] || fail "voice virtualenv missing: $PYTHON"
    [[ -x "$BRIDGE" ]] || fail "voice bridge missing/not executable: $BRIDGE"
    if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
        file "$BRIDGE" | grep -Eq 'ARM aarch64|ARM64' || fail "r1_bridge is not ARM64"
    fi
    ! ldd "$BRIDGE" 2>&1 | grep -q 'not found' || fail "r1_bridge has unresolved shared libraries"
    command -v ffmpeg >/dev/null || fail "ffmpeg is missing"

    [[ -n "${OPENAI_API_KEY:-}" && "${OPENAI_API_KEY}" != "sk-your-openai-key-here" ]] || fail "OPENAI_API_KEY is missing/placeholder"

    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" \
        "$PYTHON" -m hb_voice --check-config
    MIC_SOURCE="$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
        'from hb_voice.config import VoiceConfig; print(VoiceConfig.load().mic_source)')"
    if [[ "$MIC_SOURCE" == "alsa_usb" ]]; then
        command -v arecord >/dev/null || fail "arecord is required for input.source=alsa_usb"
        [[ -n "${ALSA_DEVICE:-}" ]] || fail "ALSA_DEVICE is required"
        [[ "${ALSA_DEVICE}" != hw:[0-9]* ]] || fail "numeric ALSA card index is not stable"
    elif [[ "$MIC_SOURCE" != "r1_multicast" ]]; then
        fail "unsupported input.source=$MIC_SOURCE"
    fi

    if [[ -e /etc/hb/stack.env ]]; then
        [[ "$(stat -c %a /etc/hb/stack.env)" == "600" ]] || fail "/etc/hb/stack.env must have mode 600"
    fi
    timeout 2 getent hosts api.openai.com >/dev/null 2>&1 || echo "[WARN] api.openai.com is not currently resolvable; supervisor will retry"
    ok "voice package, tuning, secret policy, PTT and mic source=$MIC_SOURCE"
fi

echo "PREFLIGHT_OK mode=$MODE"
