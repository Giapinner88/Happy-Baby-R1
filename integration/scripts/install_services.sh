#!/usr/bin/env bash
set -euo pipefail

[[ "${EUID}" -eq 0 ]] || { echo "Run with sudo" >&2; exit 1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
source "$SCRIPT_DIR/status_log.sh"
RUN_USER="${HB_RUN_USER:-${SUDO_USER:-$(stat -c %U "$HB_ROOT")}}"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/config_reader.sh"
INTERFACE="$(hb_config_get "$HB_ROOT/controller/config/tuning.yaml" network_interface)"
INTERFACE="${INTERFACE:-eth10}"

install -d -m 0755 /etc/hb
if [[ ! -e /etc/hb/stack.env ]]; then
    TEMPLATE="$HB_ROOT/integration/config/stack.env.example"
    install -o root -g root -m 0600 "$TEMPLATE" /etc/hb/stack.env
fi
chown root:root /etc/hb/stack.env
chmod 0600 /etc/hb/stack.env
# File này chứa secret riêng của robot. Deploy chỉ tạo khi chưa tồn tại và
# sửa quyền truy cập; tuyệt đối không ghi lại nội dung file đang có.

render() {
    local input="$1" output="$2"
    sed -e "s|@HB_ROOT@|$HB_ROOT|g" \
        -e "s|@RUN_USER@|$RUN_USER|g" \
        -e "s|@INTERFACE@|$INTERFACE|g" \
        "$input" >"$output"
    chmod 0644 "$output"
}

render "$HB_ROOT/integration/systemd/hb_integration.service.in" /etc/systemd/system/hb_integration.service
render "$HB_ROOT/integration/systemd/hb_high_level.service.in" /etc/systemd/system/hb_high_level.service
render "$HB_ROOT/integration/systemd/hb_voice.service.in" /etc/systemd/system/hb_voice.service
render "$HB_ROOT/integration/systemd/hb_voice_presets.service.in" /etc/systemd/system/hb_voice_presets.service
render "$HB_ROOT/integration/systemd/hb-stack.target.in" /etc/systemd/system/hb-stack.target

systemctl daemon-reload >/dev/null
systemctl enable hb-stack.target hb_integration.service hb_high_level.service hb_voice.service hb_voice_presets.service >/dev/null
hb_ok "HB service units installed"
