#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/status_log.sh"
WAIT_VOICE=0
PRESETS_ONLY=0
QUIET=0
for arg in "$@"; do
    case "$arg" in
        --wait-voice) WAIT_VOICE=35 ;;
        --presets) PRESETS_ONLY=1 ;;
        --quiet) QUIET=1 ;;
    esac
done

show() { (( QUIET )) || printf '%s\n' "$*"; }
mark_failed() {
    FAILED=1
    if (( QUIET )); then hb_fail "$*"; else printf '%s\n' "$*" >&2; fi
    return 0
}

if (( PRESETS_ONLY )); then
    FAILED=0
    for unit in hb_integration.service hb_voice_presets.service; do
        ACTIVE="$(systemctl is-active "$unit" 2>/dev/null || true)"
        RESTARTS="$(systemctl show "$unit" -p NRestarts --value 2>/dev/null || echo '?')"
        show "$unit active=$ACTIVE restarts=$RESTARTS"
        [[ "$ACTIVE" == "active" ]] || mark_failed "$unit inactive ($ACTIVE)"
    done
    if [[ -r /run/hb/voice_presets_status.env ]]; then
        show "--- voice presets ---"
        show "$(grep -E '^(state|mode_enabled|active|updated_monotonic_ms)=' /run/hb/voice_presets_status.env)"
    else
        mark_failed "voice preset status is missing"
    fi
    if (( QUIET )); then
        if (( FAILED == 0 )); then hb_ok "preset health check"; else hb_fail "preset health check failed"; fi
    fi
    exit "$FAILED"
fi

FAILED=0
for unit in hb_integration.service hb_high_level.service hb_voice.service hb_voice_presets.service; do
    ACTIVE="$(systemctl is-active "$unit" 2>/dev/null || true)"
    RESTARTS="$(systemctl show "$unit" -p NRestarts --value 2>/dev/null || echo '?')"
    show "$unit active=$ACTIVE restarts=$RESTARTS"
    [[ "$ACTIVE" == "active" ]] || mark_failed "$unit inactive ($ACTIVE)"
done

if [[ -r /run/hb/status.env ]]; then
    show "--- coordinator ---"
    show "$(grep -E '^(ready|high_alive|high_busy|high_armed|high_state|remote_alive|ptt|conv_mode|ptt_rearm_required|preset_busy|mic_allowed|speaker_allowed)=' /run/hb/status.env)"
    grep -q '^high_alive=1$' /run/hb/status.env || {
        mark_failed "high-level heartbeat is missing"
    }
    if ! grep -q '^remote_alive=1$' /run/hb/status.env; then
        echo "[WARN] R3-1 is not currently detected; microphone remains fail-closed" >&2
    fi
else
    mark_failed "coordinator status is missing"
fi

pgrep -af 'python.*-m hb_voice' >/dev/null || {
    mark_failed "voice runtime process is missing"
}

if (( WAIT_VOICE > 0 )); then
    for _ in $(seq 1 "$WAIT_VOICE"); do
        if [[ -r /run/hb/voice_status.env ]] && \
           grep -q '^openai_ready=1$' /run/hb/voice_status.env && \
           grep -q '^mic_ready=1$' /run/hb/voice_status.env; then
            break
        fi
        sleep 1
    done
fi

if [[ -r /run/hb/voice_status.env ]]; then
    show "--- voice runtime ---"
    show "$(grep -E '^(state|openai_ready|mic_ready|attempt|last_reason|updated_unix)=' \
        /run/hb/voice_status.env)"
    grep -q '^openai_ready=1$' /run/hb/voice_status.env || {
        mark_failed "OpenAI Realtime session is not ready"
    }
    grep -q '^mic_ready=1$' /run/hb/voice_status.env || {
        mark_failed "microphone stream is not ready"
    }
    UPDATED="$(sed -n 's/^updated_unix=//p' /run/hb/voice_status.env)"
    NOW="$(date +%s)"
    if ! [[ "$UPDATED" =~ ^[0-9]+$ ]]; then
        mark_failed "voice runtime status timestamp is invalid"
    elif (( NOW - UPDATED > 5 )); then
        mark_failed "voice runtime status is stale"
    fi
else
    mark_failed "voice runtime status is missing"
fi

if [[ -r /run/hb/voice_presets_status.env ]]; then
    show "--- voice presets ---"
    show "$(grep -E '^(state|mode_enabled|active|updated_monotonic_ms)=' /run/hb/voice_presets_status.env)"
else
    mark_failed "voice preset status is missing"
fi

if (( QUIET )); then
    if (( FAILED == 0 )); then hb_ok "runtime health check"; else hb_fail "runtime health check failed"; fi
fi
exit "$FAILED"
