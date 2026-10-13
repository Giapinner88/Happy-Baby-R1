#!/usr/bin/env bash
# Robot-local service control. `./hb start|stop|restart|status|logs` call this on
# the robot; from the dev machine the same commands run it over SSH through
# deploy_stack.sh.
# hb_high_level owns rt/lowcmd: it is stopped or restarted only when run_r1 reports
# DISARMED, otherwise the robot would lose its controller mid-motion. Teleop may
# start/stop/restart at any time: it never writes rt/lowcmd, and its loopback
# stream ending is the fail-closed path in run_r1 (same as a deadman release).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATUS="${HB_STATUS_FILE:-/run/hb/status.env}"
ALL=(hb_high_level hb_integration hb_voice hb_voice_presets)
TELEOP=(hb_teleop_cert hb_teleop_runtime)

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

# The high-level guard only matters while run_r1 is running.
high_active() { systemctl is-active --quiet hb_high_level.service; }
# Teleop units exist only after `./hb install teleop` with robot_runtime.enabled.
teleop_installed() { systemctl is-enabled --quiet hb_teleop_runtime.service 2>/dev/null; }

show() { echo "=== is-active ==="; systemctl is-active "$@" || true; }

unit_of() {
    case "$1" in
        high) echo hb_high_level ;;
        integration) echo hb_integration ;;
        voice) echo hb_voice ;;
        presets) echo hb_voice_presets ;;
        *) return 1 ;;
    esac
}

case "${1:-}" in
    status)
        bash "$SCRIPT_DIR/health_check.sh" || true
        systemctl show "${ALL[@]}" "${TELEOP[@]}" \
            -p Id -p ActiveState -p SubState -p NRestarts --no-pager
        ;;
    stop-all)
        ! high_active || disarmed || refuse_armed
        units=(hb_high_level.service hb_integration.service hb_voice.service hb_voice_presets.service)
        teleop_installed && units=(hb_teleop_runtime.service "${units[@]}")
        sudo systemctl stop "${units[@]}"
        show "${ALL[@]}" hb_teleop_runtime hb-stack.target
        ;;
    stop-high)
        # Dùng trước auto-calib: run_r1 zero-torque đè rt/lowcmd -> built-in báo overlimit.
        ! high_active || disarmed || refuse_armed
        sudo systemctl stop hb_high_level.service
        show hb_high_level hb_integration hb_voice
        ;;
    restart-all)
        ! high_active || { grep -qx "high_alive=1" "$STATUS" 2>/dev/null && disarmed; } || refuse_armed
        sudo systemctl restart hb_integration.service hb_voice.service hb_voice_presets.service
        sudo systemctl restart hb_high_level.service
        if teleop_installed; then
            sudo systemctl restart hb_teleop_cert.service hb_teleop_runtime.service
        fi
        show "${ALL[@]}" hb_teleop_runtime
        ;;
    restart-high)
        ! high_active || { grep -qx "high_alive=1" "$STATUS" 2>/dev/null && disarmed; } || refuse_armed
        sudo systemctl restart hb_high_level.service
        show hb_high_level
        ;;
    stop-integration|stop-voice|stop-presets|restart-integration|restart-voice|restart-presets)
        # Không đụng motor: integration/voice/presets không ghi rt/lowcmd.
        # ignore-dependencies: không kéo hb_high_level lên qua Wants=.
        unit="$(unit_of "${1#*-}")"
        sudo systemctl --job-mode=ignore-dependencies "${1%%-*}" "$unit.service"
        show "${ALL[@]}"
        ;;
    start-high|start-integration|start-voice|start-presets)
        unit="$(unit_of "${1#start-}")"
        sudo systemctl start "$unit.service"
        show "$unit"
        ;;
    start-all)
        sudo systemctl start hb_voice.service hb_voice_presets.service \
            hb_integration.service hb_high_level.service
        # Teleop cần R3 + START + deadman mới cử động.
        if teleop_installed; then
            sudo systemctl start hb_teleop_cert.service hb_teleop_runtime.service
        fi
        show "${ALL[@]}" hb_teleop_runtime
        ;;
    stop-teleop)
        sudo systemctl stop hb_teleop_runtime.service
        show hb_teleop_runtime
        ;;
    start-teleop|restart-teleop)
        teleop_installed || { echo "!! Teleop runtime chưa cài (./hb install teleop)." >&2; exit 1; }
        if high_active && ! disarmed; then
            echo ">> high-level đang ARMED: tay về tư thế nền như khi nhả deadman; bóp lại cò sau khi runtime lên." >&2
        fi
        sudo systemctl start hb_teleop_cert.service
        sudo systemctl "${1%%-*}" hb_teleop_runtime.service
        show hb_teleop_cert hb_teleop_runtime
        ;;
    logs-all)
        journalctl -f -n 50 -o short-iso -u hb_high_level -u hb_integration -u hb_voice \
            -u hb_voice_presets -u hb_teleop_runtime
        ;;
    logs-high|logs-integration|logs-voice|logs-presets|logs-teleop)
        target="${1#logs-}"
        if [[ "$target" == teleop ]]; then unit=hb_teleop_runtime; else unit="$(unit_of "$target")"; fi
        journalctl -f -n 100 -o short-iso -u "$unit"
        ;;
    *)
        echo "Usage: $0 status | {start|stop|restart}-{all|high|integration|voice|presets|teleop} | logs-{all|high|integration|voice|presets|teleop}" >&2
        exit 2
        ;;
esac
