#!/usr/bin/env bash
set -euo pipefail

WAIT_VOICE=0
[[ "${1:-}" == "--wait-voice" ]] && WAIT_VOICE=35

FAILED=0
for unit in hb_integration.service hb_high_level.service hb_voice.service; do
    ACTIVE="$(systemctl is-active "$unit" 2>/dev/null || true)"
    RESTARTS="$(systemctl show "$unit" -p NRestarts --value 2>/dev/null || echo '?')"
    echo "$unit active=$ACTIVE restarts=$RESTARTS"
    [[ "$ACTIVE" == "active" ]] || FAILED=1
done

if [[ -r /run/hb/status.env ]]; then
    echo "--- coordinator ---"
    grep -E '^(ready|high_alive|high_busy|high_armed|high_state|remote_alive|ptt|ptt_rearm_required|mic_allowed|speaker_allowed)=' /run/hb/status.env
    grep -q '^high_alive=1$' /run/hb/status.env || {
        echo "high-level heartbeat is missing" >&2
        FAILED=1
    }
    if ! grep -q '^remote_alive=1$' /run/hb/status.env; then
        echo "[WARN] R3-1 is not currently detected; microphone remains fail-closed" >&2
    fi
else
    echo "coordinator status is missing" >&2
    FAILED=1
fi

pgrep -af 'python.*-m hb_voice' >/dev/null || {
    echo "voice runtime process is missing" >&2
    FAILED=1
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
    echo "--- voice runtime ---"
    grep -E '^(state|openai_ready|mic_ready|attempt|last_reason|updated_unix)=' \
        /run/hb/voice_status.env
    grep -q '^openai_ready=1$' /run/hb/voice_status.env || {
        echo "OpenAI Realtime session is not ready" >&2
        FAILED=1
    }
    grep -q '^mic_ready=1$' /run/hb/voice_status.env || {
        echo "microphone stream is not ready" >&2
        FAILED=1
    }
    UPDATED="$(sed -n 's/^updated_unix=//p' /run/hb/voice_status.env)"
    NOW="$(date +%s)"
    if ! [[ "$UPDATED" =~ ^[0-9]+$ ]]; then
        echo "voice runtime status timestamp is invalid" >&2
        FAILED=1
    elif (( NOW - UPDATED > 5 )); then
        echo "voice runtime status is stale" >&2
        FAILED=1
    fi
else
    echo "voice runtime status is missing" >&2
    FAILED=1
fi

exit "$FAILED"
