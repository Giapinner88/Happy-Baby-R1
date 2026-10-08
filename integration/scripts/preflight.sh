#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-all}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
HIGH_DIR="$HB_ROOT/controller"
VOICE_DIR="$HB_ROOT/voice"
PRESETS_DIR="$HB_ROOT/voice_presets"
INTEGRATION_DIR="$HB_ROOT/integration"
# shellcheck disable=SC1091
source "$INTEGRATION_DIR/scripts/config_reader.sh"
source "$INTEGRATION_DIR/scripts/status_log.sh"
source "$INTEGRATION_DIR/scripts/dance_preflight_summary.sh"

if [[ -r /etc/hb/stack.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source /etc/hb/stack.env
    set +a
fi

fail() { hb_fail "$*"; exit 1; }
ok() { hb_ok "$*"; }
includes() { [[ "$MODE" == "all" || "$MODE" == "$1" ]]; }
TEMP_FILES=()
HIGH_PREFLIGHT_LOG=""
PRESERVE_HIGH_PREFLIGHT_LOG=0
cleanup() {
    if ((${#TEMP_FILES[@]})); then
        local file
        for file in "${TEMP_FILES[@]}"; do
            [[ "$PRESERVE_HIGH_PREFLIGHT_LOG" == "1" && "$file" == "$HIGH_PREFLIGHT_LOG" ]] || rm -f -- "$file"
        done
    fi
}
trap cleanup EXIT

check_ldd() {
    local label="$1" binary="$2" log
    log="$(mktemp /tmp/hb-ldd.XXXXXX)"
    TEMP_FILES+=("$log")
    if ! ldd "$binary" >"$log" 2>&1; then
        cat "$log" >&2
        fail "$label dependency inspection failed"
    fi
    if grep -q 'not found' "$log"; then
        grep 'not found' "$log" >&2
        fail "$label has unresolved shared libraries"
    fi
}

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
    check_ldd "run_r1" "$BIN"

    # shellcheck disable=SC1091
    source "$INTEGRATION_DIR/config/model_manifest.conf"
    SELECTED="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_model)"
    [[ -n "$SELECTED" && "$SELECTED" != */* && "$SELECTED" != "." && "$SELECTED" != ".." ]] || fail "flat_model must name a file, not a path"
    # Contract phải khớp cả ba nơi: config, manifest và (với route history) cả
    # metadata trong ONNX. Hash trùng chỉ chứng minh đúng file, không chứng minh
    # runtime sẽ diễn giải nó đúng cách.
    SELECTED_CONTRACT="$(hb_config_get "$HIGH_DIR/config/tuning.yaml" flat_policy_contract)"
    [[ -n "${MODEL_CONTRACT:-}" ]] || fail "manifest has no MODEL_CONTRACT; re-run update_model_manifest.sh"
    [[ "$MODEL_CONTRACT" == "$SELECTED_CONTRACT" ]] || \
        fail "manifest contract=$MODEL_CONTRACT but tuning selects $SELECTED_CONTRACT"

    # Chạy binary TRƯỚC rồi mới đối chiếu manifest: run_r1 là nguồn sự thật duy
    # nhất về đường dẫn model và số chiều obs của từng contract. Script này từng
    # giữ bảng contract riêng và ép mọi model nằm dưới policies/flat — cả hai đều
    # lệch với runtime (non-legacy nạp từ policies/locomotion/flat_plus).
    HIGH_PREFLIGHT_LOG="$(mktemp /tmp/hb-high-preflight.XXXXXX)"
    TEMP_FILES+=("$HIGH_PREFLIGHT_LOG")
    "$BIN" --preflight >"$HIGH_PREFLIGHT_LOG" 2>&1 || {
        PRESERVE_HIGH_PREFLIGHT_LOG=1
        fail "run_r1 preflight failed; details: $HIGH_PREFLIGHT_LOG"
    }
    MODEL_LINE="$(grep -m1 '^\[Preflight\] MODEL ' "$HIGH_PREFLIGHT_LOG" || true)"
    [[ -n "$MODEL_LINE" ]] || \
        fail "run_r1 --preflight khong in dong '[Preflight] MODEL ...' (binary cu hon script?)"
    RUNTIME_CONTRACT="$(sed -n 's/.* contract=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
    RUNTIME_REL="$(sed -n 's/.* rel=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
    RUNTIME_INPUT="$(sed -n 's/.* input=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
    RUNTIME_OUTPUT="$(sed -n 's/.* output=\([^ ]*\).*/\1/p' <<<"$MODEL_LINE")"
    [[ "$RUNTIME_CONTRACT" == "$SELECTED_CONTRACT" ]] || \
        fail "runtime nap contract=$RUNTIME_CONTRACT nhung tuning chon $SELECTED_CONTRACT"
    [[ "$RUNTIME_REL" == */"$SELECTED" ]] || \
        fail "runtime nap $RUNTIME_REL nhung flat_model=$SELECTED"
    [[ "$MODEL_REL" == "$RUNTIME_REL" ]] || \
        fail "manifest MODEL_REL=$MODEL_REL nhung runtime nap $RUNTIME_REL"
    [[ "${MODEL_INPUT:-}" == "$RUNTIME_INPUT" ]] || \
        fail "manifest MODEL_INPUT=${MODEL_INPUT:-<unset>} nhung $SELECTED_CONTRACT dung $RUNTIME_INPUT"
    [[ "${MODEL_OUTPUT:-$RUNTIME_OUTPUT}" == "$RUNTIME_OUTPUT" ]] || \
        fail "manifest MODEL_OUTPUT=${MODEL_OUTPUT} nhung runtime dung $RUNTIME_OUTPUT"
    hb_summarize_dance_preflight "$HIGH_DIR/config/dance.yaml" "$HIGH_PREFLIGHT_LOG"
    MODEL="$HIGH_DIR/$MODEL_REL"
    [[ -f "$MODEL" && -s "$MODEL" ]] || fail "model missing or not a regular file: $MODEL"
    ACTUAL_SHA="$(sha256sum "$MODEL" | awk '{print $1}')"
    [[ "$ACTUAL_SHA" == "$MODEL_SHA256" ]] || fail "model SHA-256 differs from manifest"
    if grep -R -E '^[[:space:]]*voice_[^:]*:[[:space:]]*"?/home/' "$HIGH_DIR/config" >/dev/null; then
        fail "high-level voice paths still contain absolute /home paths"
    fi
    ok "high-level locomotion model, manifest and libraries"
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
    check_ldd "r1_bridge" "$BRIDGE"
    command -v ffmpeg >/dev/null || fail "ffmpeg is missing"

    [[ -n "${OPENAI_API_KEY:-}" && "${OPENAI_API_KEY}" != "sk-your-openai-key-here" ]] || fail "OPENAI_API_KEY is missing/placeholder"

    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" \
        "$PYTHON" -m hb_voice --check-config
    TTS_PROVIDER="$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
        'from hb_voice.config import VoiceConfig; print(VoiceConfig.load().tts_provider)')"
    MIC_SOURCE="$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
        'from hb_voice.config import VoiceConfig; print(VoiceConfig.load().mic_source)')"
    MIC_FALLBACK="$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
        'from hb_voice.config import VoiceConfig; print(int(VoiceConfig.load().mic_fallback_enabled))')"
    if [[ "$TTS_PROVIDER" == "elevenlabs" ]]; then
        PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
            'from pipecat.services.elevenlabs.tts import ElevenLabsTTSService' \
            || fail "Pipecat ElevenLabs TTS integration is unavailable"
        timeout 2 getent hosts api.elevenlabs.io >/dev/null 2>&1 \
            || echo "[WARN] api.elevenlabs.io is not currently resolvable; supervisor will retry"
    elif [[ "$TTS_PROVIDER" != "openai" ]]; then
        fail "unsupported tts.provider=$TTS_PROVIDER"
    fi
    if [[ "$MIC_SOURCE" == "alsa_usb" ]]; then
        command -v arecord >/dev/null || fail "arecord is required for input.source=alsa_usb"
        # A missing external mic is only fatal without a standby.  With
        # input.fallback_to_robot_mic the runtime is expected to start on the
        # PC1 mic and pick the external one up as soon as it is plugged in;
        # blocking the whole stack here would defeat that.
        if [[ "$MIC_FALLBACK" == "1" ]]; then
            mic_missing() { echo "[WARN] $1; voice will start on the robot mic standby"; }
        else
            mic_missing() { fail "$1"; }
        fi
        if [[ -z "${ALSA_DEVICE:-}" ]]; then
            # Unset is the normal setup: the card name belongs to the mic model,
            # so the runtime discovers it at spawn time and a new mic needs no
            # file edited.  Ask the runtime's own scanner rather than repeating
            # it here, or the two would drift apart.
            USB_CARDS="$(PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$VOICE_DIR" "$PYTHON" -c \
                'from hb_voice.input import usb_capture_cards
print(" ".join(name for _, name in usb_capture_cards()))')" \
                || fail "USB capture card detection failed"
            if [[ -n "$USB_CARDS" ]]; then
                ok "external mic auto-detected: $USB_CARDS"
            else
                mic_missing "no USB capture card is present on PC2"
            fi
        else
            [[ "${ALSA_DEVICE}" != hw:[0-9]* ]] || fail "numeric ALSA card index is not stable"
            if [[ "$ALSA_DEVICE" =~ CARD=([^,]+),DEV=([0-9]+) ]]; then
                ALSA_CARD="${BASH_REMATCH[1]}"
                ALSA_CAPTURE_DEVICE="${BASH_REMATCH[2]}"
            else
                fail "ALSA_DEVICE must contain a stable CARD name and DEV number"
            fi
            [[ "$ALSA_CARD" != "APE" ]] \
                || fail "Jetson APE is a virtual card, not the external microphone"
            ARECORD_DEVICES="$(LC_ALL=C arecord -l 2>/dev/null)" \
                || mic_missing "PC2 has no available ALSA capture device"
            awk -v card="$ALSA_CARD" -v device="$ALSA_CAPTURE_DEVICE" '
                $1 == "card" {
                    card_name = $3
                    device_number = ""
                    for (i = 1; i <= NF; i++) {
                        if ($i == "device") {
                            device_number = $(i + 1)
                            sub(/:$/, "", device_number)
                            break
                        }
                    }
                    if (card_name == card && device_number == device) found = 1
                }
                END { exit !found }
            ' <<<"$ARECORD_DEVICES" \
                || mic_missing "ALSA capture device $ALSA_DEVICE is not present on PC2"
        fi
    elif [[ "$MIC_SOURCE" != "r1_multicast" ]]; then
        fail "unsupported input.source=$MIC_SOURCE"
    fi

    if [[ -e /etc/hb/stack.env ]]; then
        [[ "$(stat -c %a /etc/hb/stack.env)" == "600" ]] || fail "/etc/hb/stack.env must have mode 600"
    fi
    timeout 2 getent hosts api.openai.com >/dev/null 2>&1 || echo "[WARN] api.openai.com is not currently resolvable; supervisor will retry"
    ok "voice package, tuning, secret policy, PTT, tts=$TTS_PROVIDER and mic source=$MIC_SOURCE"
fi

if includes presets; then
    PYTHON="$VOICE_DIR/.venv/bin/python"
    BRIDGE="${UNITREE_BRIDGE_PATH:-voice/unitree_bridge/build/r1_bridge}"
    [[ "$BRIDGE" = /* ]] || BRIDGE="$HB_ROOT/$BRIDGE"
    [[ -d "$PRESETS_DIR" ]] || fail "voice_presets package is missing: $PRESETS_DIR"
    [[ -x "$PYTHON" ]] || fail "voice virtualenv missing: $PYTHON"
    [[ -x "$BRIDGE" ]] || fail "r1_bridge missing/not executable: $BRIDGE"
    if [[ "$ARCH" == "aarch64" || "$ARCH" == "arm64" ]]; then
        file "$BRIDGE" | grep -Eq 'ARM aarch64|ARM64' || fail "r1_bridge is not ARM64"
    fi
    check_ldd "r1_bridge" "$BRIDGE"
    command -v ffmpeg >/dev/null || fail "ffmpeg is missing"
    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$HB_ROOT" \
        "$PYTHON" -m voice_presets --check-config
    ok "offline voice presets, local assets and speaker bridge"
fi

hb_ok "PREFLIGHT_OK mode=$MODE"
