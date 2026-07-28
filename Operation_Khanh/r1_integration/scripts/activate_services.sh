#!/usr/bin/env bash
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }
NO_RESTART=0
[[ "${1:-}" == "--no-restart" ]] && NO_RESTART=1

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
SAFE_HIGH=0

if [[ "$NO_RESTART" == "0" ]]; then
    if ! systemctl is-active --quiet hb_high_level.service; then
        SAFE_HIGH=1
    elif [[ -r /run/hb/status.env ]] && \
         grep -q '^high_alive=1$' /run/hb/status.env && \
         grep -q '^high_armed=0$' /run/hb/status.env && \
         grep -q '^high_state=DISARMED' /run/hb/status.env; then
        SAFE_HIGH=1
    else
        # Lần cài đầu chưa có coordinator/heartbeat. Chỉ chấp nhận log sống rõ ràng:
        # chờ một batch stdout mới (C++ có thể buffer journal khoảng 30–40 giây), rồi
        # yêu cầu ít nhất 3 dòng DISARMED và 3 dòng cuối đều DISARMED trong 5 giây.
        for _ in $(seq 1 45); do
            RECENT_HIGH="$(journalctl -u hb_high_level.service --since '-5 seconds' \
                --no-pager -o cat 2>/dev/null || true)"
            DISARMED_COUNT="$(printf '%s\n' "$RECENT_HIGH" | grep -c '\[kDisarmed\]' || true)"
            LAST_THREE="$(printf '%s\n' "$RECENT_HIGH" | tail -n 3)"
            LAST_THREE_COUNT="$(printf '%s\n' "$LAST_THREE" | grep -c '\[kDisarmed\]' || true)"
            if (( DISARMED_COUNT >= 3 && LAST_THREE_COUNT == 3 )); then
                SAFE_HIGH=1
                break
            fi
            sleep 2
        done
    fi
fi

HB_ROOT="$HB_ROOT" bash "$SCRIPT_DIR/install_services.sh"

set -a
# shellcheck disable=SC1091
source /etc/hb/stack.env
set +a
export HB_ROOT
bash "$SCRIPT_DIR/preflight.sh" all

if [[ "$NO_RESTART" == "1" ]]; then
    echo "DEPLOYED_NO_RESTART"
    exit 0
fi

systemctl restart hb_integration.service
if [[ "$SAFE_HIGH" == "1" ]]; then
    systemctl restart hb_high_level.service
else
    systemctl restart hb_voice.service || true
    echo "PENDING_RESTART: high-level was not proven DISARMED; new high binary was not activated" >&2
    exit 3
fi

sleep 6
systemctl restart hb_voice.service
sleep 10
bash "$SCRIPT_DIR/health_check.sh" --wait-voice
echo "DEPLOY_OK"
