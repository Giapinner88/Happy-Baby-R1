#!/usr/bin/env bash
# Load the single operator-editable YAML config from inside HB.
#
# The shell launcher remains responsible for joining isolated Python
# environments; load_teleop_config.py is the only YAML parser it needs.

hb_load_network_config() {
    if [[ "${HB_TELEOP_CONFIG_LOADED:-0}" == "1" ]]; then
        return 0
    fi

    local scripts_dir config loader
    scripts_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    config="${HB_TELEOP_CONFIG:-$scripts_dir/../config/teleop.yaml}"
    loader="$scripts_dir/load_teleop_config.py"
    if [[ ! -r "$config" ]]; then
        echo "[FAIL] Không thấy teleop YAML: $config" >&2
        return 1
    fi
    if [[ ! -r "$loader" ]]; then
        echo "[FAIL] Không thấy YAML loader: $loader" >&2
        return 1
    fi

    # YAML is authoritative. Explicit one-shot overrides use namespaced
    # *_OVERRIDE variables and cannot be accidentally inherited from an old
    # shell's generic ROBOT/HOST_IP variables.
    local assignments
    if ! assignments="$(python3 "$loader" --config "$config" --shell)"; then
        return 1
    fi
    eval "$assignments" || return 1
    [[ -n "${HB_TELEOP_ROBOT_OVERRIDE:-}" ]] && ROBOT="$HB_TELEOP_ROBOT_OVERRIDE"
    [[ -n "${HB_TELEOP_HOST_IP_OVERRIDE:-}" ]] && HB_TELEOP_HOST_IP="$HB_TELEOP_HOST_IP_OVERRIDE"
    [[ -n "${HB_TELEOP_HOST_INTERFACE_OVERRIDE:-}" ]] && HB_TELEOP_HOST_INTERFACE="$HB_TELEOP_HOST_INTERFACE_OVERRIDE"
    [[ -n "${HB_TELEOP_REQUIRE_LOCOMOTION_OVERRIDE:-}" ]] && HB_TELEOP_REQUIRE_LOCOMOTION="$HB_TELEOP_REQUIRE_LOCOMOTION_OVERRIDE"
    [[ -n "${HB_TELEOP_LOCOMOTION_WAIT_S_OVERRIDE:-}" ]] && HB_TELEOP_LOCOMOTION_WAIT_S="$HB_TELEOP_LOCOMOTION_WAIT_S_OVERRIDE"
    export ROBOT HB_TELEOP_HOST_IP HB_TELEOP_HOST_INTERFACE
    export HB_TELEOP_CONFIG="$config"
    export HB_TELEOP_CONFIG_LOADED=1
}

hb_resolve_host_ip() {
    local requested="${1:-${HB_TELEOP_HOST_IP:-auto}}"
    if [[ -n "$requested" && "$requested" != "auto" ]]; then
        printf '%s\n' "$requested"
        return 0
    fi

    local iface="${HB_TELEOP_HOST_INTERFACE:-}"
    local resolved=""
    if [[ -n "$iface" && "$iface" != "auto" ]]; then
        resolved="$(ip -4 -o addr show dev "$iface" scope global 2>/dev/null \
            | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
    else
        local route_iface
        route_iface="$(ip -4 route get 1.1.1.1 2>/dev/null \
            | awk '{for (i = 1; i <= NF; i++) if ($i == "dev") {print $(i+1); exit}}')"
        if [[ -n "$route_iface" ]]; then
            resolved="$(ip -4 -o addr show dev "$route_iface" scope global 2>/dev/null \
                | awk 'NR == 1 {split($4, a, "/"); print a[1]}')"
        fi
    fi
    if [[ -z "$resolved" ]]; then
        resolved="$(ip -4 -o addr show scope global 2>/dev/null \
            | awk '$2 != "lo" {split($4, a, "/"); print a[1]; exit}')"
    fi
    if [[ -z "$resolved" ]]; then
        echo "[FAIL] Không tự xác định được IPv4 của laptop; sửa network.host_ip trong teleop.yaml." >&2
        return 1
    fi
    printf '%s\n' "$resolved"
}
