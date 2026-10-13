#!/usr/bin/env bash
# Find a deploy transport for R1. Quest networking is independent: this only
# selects an SSH/rsync route and can use Ethernet, local Wi-Fi, or Tailscale.

if [[ "${HB_TELEOP_NETWORK_CONFIG_LOADED:-0}" != "1" ]]; then
    _FIND_ROBOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    # shellcheck disable=SC1091
    source "$_FIND_ROBOT_DIR/load_network_config.sh"
    hb_load_network_config
fi

# Local links come first. Tailscale remains last so it cannot hide a faster
# direct path. Override with a space-separated HB_ROBOT_CANDIDATES.
_teleop_robot_candidates() {
    if [[ -n "${HB_ROBOT_CANDIDATES:-}" ]]; then
        printf '%s\n' $HB_ROBOT_CANDIDATES
        return
    fi
    printf '%s\n' \
        10.42.0.33 \
        192.168.123.164 \
        192.168.145.209 \
        192.168.12.2 \
        192.168.1.33 \
        192.168.1.104 \
        192.168.1.108 \
        unitree-r1 \
        "${HB_ROBOT_TAILSCALE:-100.82.165.36}"
}

_robot_ssh_identity_ok() {
    local target="$1"
    local probe_timeout="${HB_ROBOT_PROBE_TIMEOUT:-3}"
    local -a multiplex_opts=()
    [[ "$probe_timeout" =~ ^[0-9]+([.][0-9]+)?$ ]] || probe_timeout=3
    if [[ -n "${HB_SSH_CONTROL_PATH:-}" ]]; then
        multiplex_opts=(-o ControlMaster=auto -o "ControlPath=$HB_SSH_CONTROL_PATH")
    else
        multiplex_opts=(-o ControlMaster=no -o ControlPath=none)
    fi
    timeout --signal=TERM --kill-after=1s "${probe_timeout}s" \
        ssh -o BatchMode=yes \
            "${multiplex_opts[@]}" \
            -o ConnectionAttempts=1 \
            -o ConnectTimeout="$probe_timeout" \
            -o StrictHostKeyChecking=accept-new \
            "$target" '[ -d /home/unitree/Foundation_Unitree ] && [ -d /home/unitree/HB ]' \
        2>/dev/null
}

_robot_direct_link() {
    local address="$1" route=""
    [[ "$address" == *.*.*.* ]] || return 1
    route="$(ip -4 route get "$address" 2>/dev/null || true)"
    [[ -n "$route" && "$route" == *' dev '* && "$route" != *' via '* ]]
}

_teleop_open_interactive_master() {
    local target="$1"
    local control_path="${HB_SSH_CONTROL_PATH:-/tmp/hb-ssh-${BASHPID}-%C}"
    local connect_timeout="${HB_DEPLOY_CONNECT_TIMEOUT:-10}"
    [[ "$connect_timeout" =~ ^[0-9]+$ ]] || connect_timeout=10

    echo "[AUTH] SSH key/password cần xác thực cho $target..."
    if ssh -o ControlMaster=yes -o ControlPersist=900 -o "ControlPath=$control_path" \
        -o ConnectionAttempts=1 -o "ConnectTimeout=$connect_timeout" \
        -o ServerAliveInterval=5 -o ServerAliveCountMax=2 -MNf "$target"; then
        HB_SSH_CONTROL_PATH="$control_path"
        export HB_SSH_CONTROL_PATH
        return 0
    fi
    return 1
}

_scan_robot_candidates() {
    local address target index pid scan_dir
    local -a direct=() other=() ordered=() pids=()
    declare -A seen=()

    while IFS= read -r address; do
        [[ -n "$address" && -z "${seen[$address]+x}" ]] || continue
        seen[$address]=1
        if _robot_direct_link "$address"; then direct+=("$address"); else other+=("$address"); fi
    done < <(_teleop_robot_candidates)
    ordered=("${direct[@]}" "${other[@]}")
    scan_dir="$(mktemp -d "${TMPDIR:-/tmp}/hb-teleop-robot-scan.XXXXXX")" || return 1

    index=0
    for address in "${ordered[@]}"; do
        target="unitree@$address"
        (_robot_ssh_identity_ok "$target" && : >"$scan_dir/$index.ok") &
        pids+=("$!")
        index=$((index + 1))
    done
    for pid in "${pids[@]}"; do wait "$pid" || true; done

    index=0
    for address in "${ordered[@]}"; do
        if [[ -f "$scan_dir/$index.ok" ]]; then
            rm -rf "$scan_dir"
            ROBOT="unitree@$address"
            case "$address" in
                100.*|unitree-r1) echo "✅ R1 qua Tailscale: $ROBOT" ;;
                *) echo "✅ R1 qua SSH: $ROBOT" ;;
            esac
            return 0
        fi
        index=$((index + 1))
    done
    rm -rf "$scan_dir"
    return 1
}

find_robot() {
    local preferred="${ROBOT:-}"
    local round=0
    local interval="${HB_ROBOT_SCAN_INTERVAL:-1}"
    local max_rounds="${HB_ROBOT_SCAN_MAX_ROUNDS:-0}"
    [[ "$interval" =~ ^[0-9]+$ ]] || interval=1
    [[ "$max_rounds" =~ ^[0-9]+$ ]] || max_rounds=0

    while :; do
        round=$((round + 1))
        if [[ -n "$preferred" ]] && _robot_ssh_identity_ok "$preferred"; then
            ROBOT="$preferred"
            echo "✅ R1 từ cấu hình/override: $ROBOT"
            return 0
        fi
        if [[ -n "$preferred" && "$round" -eq 1 && "${HB_DEPLOY_INTERACTIVE_SSH:-1}" == "1" ]] && \
           _teleop_open_interactive_master "$preferred"; then
            ROBOT="$preferred"
            echo "✅ R1 từ cấu hình/override: $ROBOT"
            return 0
        fi
        [[ -n "$preferred" ]] && echo "⚠️  $preferred không tới được hoặc không xác minh là R1; quét mọi đường SSH..."
        _scan_robot_candidates && return 0

        if [[ "${HB_ROBOT_SCAN_ONCE:-0}" == "1" || ( "$max_rounds" -gt 0 && "$round" -ge "$max_rounds" ) ]]; then
            echo "❌ Không thấy R1 qua Ethernet, Wi-Fi hay Tailscale." >&2
            return 1
        fi
        echo "⏳ Chưa thấy R1 ở vòng $round; quét lại sau ${interval}s (Ctrl+C để dừng)."
        sleep "$interval"
    done
}
