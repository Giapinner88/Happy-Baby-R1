#!/usr/bin/env bash
# Robot-local service control. `make status|stop-*|start-*|restart-all` call this
# on the robot; deploy_stack.sh calls the same script over SSH from the dev machine.
# hb_high_level owns rt/lowcmd: it is stopped or restarted only when run_r1 reports
# DISARMED, otherwise the robot would lose its controller mid-motion.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATUS=/run/hb/status.env
ALL=(hb_high_level hb_integration hb_voice hb_voice_presets)

disarmed() {
    grep -qx "high_armed=0" "$STATUS" 2>/dev/null &&
        grep -q "^high_state=DISARMED" "$STATUS" 2>/dev/null
}

refuse_armed() {
    echo "!! CHƯA DISARMED (hoặc mất $STATUS) — KHÔNG thực hiện." >&2
    grep "^high_state=" "$STATUS" 2>/dev/null >&2 || true
    echo "   Đỡ/treo robot, vào Mode Z, nhả Quest deadman và các nút; giữ R1+R2 khi đứng yên để DISARMED, rồi chạy lại." >&2
    exit 1
}

show() { echo "=== is-active ==="; systemctl is-active "$@" || true; }

case "${1:-}" in
    status)
        bash "$SCRIPT_DIR/health_check.sh" || true
        systemctl show "${ALL[@]}" hb_teleop_cert hb_teleop_runtime \
            -p Id -p ActiveState -p SubState -p NRestarts --no-pager
        ;;
    stop-all)
        disarmed || refuse_armed
        sudo systemctl stop hb_high_level.service hb_integration.service \
            hb_voice.service hb_voice_presets.service
        show "${ALL[@]}" hb-stack.target
        ;;
    stop-high)
        # Dùng trước auto-calib: run_r1 zero-torque đè rt/lowcmd -> built-in báo overlimit.
        disarmed || refuse_armed
        sudo systemctl stop hb_high_level.service
        show hb_high_level hb_integration hb_voice
        ;;
    restart-all)
        grep -qx "high_alive=1" "$STATUS" 2>/dev/null && disarmed || refuse_armed
        sudo systemctl restart hb_integration.service hb_voice.service hb_voice_presets.service
        sudo systemctl restart hb_high_level.service
        show "${ALL[@]}"
        ;;
    stop-integration|stop-voice|stop-presets)
        # Không đụng motor: integration/voice/presets không ghi rt/lowcmd.
        unit="hb_${1#stop-}"; [[ "$unit" == hb_presets ]] && unit=hb_voice_presets
        sudo systemctl stop "$unit.service"
        show "${ALL[@]}"
        ;;
    start-high|start-integration|start-voice|start-presets)
        unit="hb_${1#start-}"
        [[ "$unit" == hb_high ]] && unit=hb_high_level
        [[ "$unit" == hb_presets ]] && unit=hb_voice_presets
        sudo systemctl start "$unit.service"
        show "$unit"
        ;;
    start-all)
        sudo systemctl start hb_voice.service hb_voice_presets.service \
            hb_integration.service hb_high_level.service
        show "${ALL[@]}"
        ;;
    *)
        echo "Usage: $0 status|stop-all|stop-high|stop-integration|stop-voice|stop-presets|restart-all|start-high|start-integration|start-voice|start-presets|start-all" >&2
        exit 2
        ;;
esac
