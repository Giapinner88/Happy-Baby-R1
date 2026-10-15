#!/usr/bin/env bash
# Shared SSH and sudo authentication for HB deploy entry points.
#
# Deploy commands open one SSH master immediately.  Privileged commands then
# use a short, user-scoped sudo timestamp shared by the SSH channels.  This
# prevents a build or rsync from reaching a late password prompt.

hb_deploy_auth_fail() {
    local red= reset=
    if [[ -t 2 && -z "${NO_COLOR:-}" ]]; then
        red=$'\033[31m'; reset=$'\033[0m'
    fi
    printf '%s[FAIL] deploy authentication:%s %s\n' "$red" "$reset" "$*" >&2
    return 1
}

hb_deploy_auth_open_ssh() {
    local robot="$1" control_path="$2" connect_timeout="${3:-10}"
    local persist="${HB_DEPLOY_CONTROL_PERSIST:-900}"

    [[ "$connect_timeout" =~ ^[0-9]+$ ]] || connect_timeout=10
    [[ "$persist" =~ ^[0-9]+$ ]] || persist=900

    if command ssh -o "ControlPath=$control_path" -O check "$robot" >/dev/null 2>&1; then
        return 0
    fi

    printf '[AUTH] Xác thực SSH tới %s trước khi deploy...\n' "$robot"
    command ssh \
        -o ControlMaster=yes \
        -o "ControlPersist=$persist" \
        -o "ControlPath=$control_path" \
        -o ConnectionAttempts=1 \
        -o "ConnectTimeout=$connect_timeout" \
        -o ServerAliveInterval=5 \
        -o ServerAliveCountMax=2 \
        -f "$robot" true || {
            hb_deploy_auth_fail "không mở được SSH tới $robot; chưa bắt đầu deploy"
            return 1
        }

    export HB_DEPLOY_AUTH_CONTROL_PATH="$control_path"
}

hb_deploy_auth_enable_sudo() {
    local robot="$1" control_path="$2" connect_timeout="${3:-10}"
    local keepalive_pid="${HB_DEPLOY_SUDO_KEEPALIVE_PID:-}"

    hb_deploy_auth_open_ssh "$robot" "$control_path" "$connect_timeout" || return 1

    if [[ "${HB_DEPLOY_SUDO_READY:-0}" == "1" ]]; then
        if command ssh -o ControlMaster=auto -o "ControlPath=$control_path" "$robot" 'sudo -n -v'; then
            if [[ -n "$keepalive_pid" ]] && kill -0 "$keepalive_pid" 2>/dev/null; then
                return 0
            fi
        fi
        # A child process can inherit these exported markers after the SSH
        # master or sudo timestamp has gone away.  Re-authenticate now, while
        # we are still at the start of the command, rather than fail or wait
        # for a password after the build has begun.
        unset HB_DEPLOY_SUDO_READY HB_DEPLOY_SUDO_KEEPALIVE_PID
        keepalive_pid=""
    fi

    printf '[AUTH] Xác thực sudo trên robot trước khi deploy...\n'
    # sudo normally caches credentials per terminal.  Deploy crosses several
    # SSH PTYs, so install one narrow Defaults entry for the logged-in robot
    # user.  It grants no command permission and keeps the normal timeout;
    # only the timestamp key becomes shared by this user during the deploy.
    command ssh \
        -tt \
        -o ControlMaster=auto \
        -o "ControlPath=$control_path" \
        -o ConnectionAttempts=1 \
        -o "ConnectTimeout=$connect_timeout" \
        "$robot" '
set -euo pipefail
# Changing timestamp_type changes the cache lookup immediately. Install and
# validate the rule inside one authenticated root process, without nested sudo.
sudo bash -s <<\HB_DEPLOY_ROOT
set -euo pipefail
user="${SUDO_USER:?missing original sudo user}"
tmp="$(mktemp /tmp/hb-deploy-sudoers.XXXXXX)"
cleanup() { rm -f "$tmp"; }
trap cleanup EXIT
printf "%s\n" \
  "# Managed by HB deploy: share only unitree sudo timestamps across SSH PTYs." \
  "Defaults:${user} timestamp_type=global" >"$tmp"
visudo -cf "$tmp"
install -d -o root -g root -m 0750 /etc/sudoers.d
install -o root -g root -m 0440 "$tmp" /etc/sudoers.d/90-hb-deploy-auth
visudo -c
HB_DEPLOY_ROOT
if ! sudo -n -v 2>/dev/null; then
    printf "[AUTH] Lần thiết lập đầu: xác thực lại cache sudo mới trong cùng lệnh deploy.\n"
    sudo -v
fi
sudo -n -v
' || {
        hb_deploy_auth_fail "xác thực/cấu hình sudo thất bại; xem lỗi ở trên, chưa đồng bộ hoặc restart runtime"
        return 1
    }

    if ! command ssh -o ControlMaster=auto -o "ControlPath=$control_path" \
        -o "ConnectTimeout=$connect_timeout" "$robot" 'sudo -n -v'; then
        hb_deploy_auth_fail "không dùng được sudo cache qua SSH channel mới; deploy chưa bắt đầu"
        return 1
    fi

    export HB_DEPLOY_SUDO_READY=1
    hb_deploy_auth_start_keepalive "$robot" "$control_path" "$connect_timeout"
}

hb_deploy_auth_start_keepalive() {
    local robot="$1" control_path="$2" connect_timeout="${3:-10}"
    local interval="${HB_DEPLOY_SUDO_KEEPALIVE_SECONDS:-60}"
    local existing="${HB_DEPLOY_SUDO_KEEPALIVE_PID:-}"

    [[ "$interval" =~ ^[0-9]+$ ]] && (( interval >= 10 )) || interval=60
    if [[ -n "$existing" ]] && kill -0 "$existing" 2>/dev/null; then
        return 0
    fi

    (
        sleep_pid=
        trap 'if [[ -n "$sleep_pid" ]]; then kill "$sleep_pid" 2>/dev/null || true; fi; exit 0' TERM INT
        while true; do
            sleep "$interval" &
            sleep_pid=$!
            wait "$sleep_pid" || exit 0
            sleep_pid=
            if ! command ssh \
                -o ControlMaster=auto \
                -o "ControlPath=$control_path" \
                -o ConnectionAttempts=1 \
                -o "ConnectTimeout=$connect_timeout" \
                "$robot" 'sudo -n -v' >/dev/null; then
                printf '[FAIL] deploy authentication: sudo ticket trên robot đã mất; lệnh sudo tiếp theo sẽ dừng.\n' >&2
                exit 1
            fi
        done
    ) &
    HB_DEPLOY_SUDO_KEEPALIVE_PID=$!
    export HB_DEPLOY_SUDO_KEEPALIVE_PID
}

hb_deploy_auth_cleanup() {
    local control_path="${1:-${HB_DEPLOY_AUTH_CONTROL_PATH:-}}"
    local robot="${2:-${ROBOT:-}}"
    local keepalive_pid="${HB_DEPLOY_SUDO_KEEPALIVE_PID:-}"

    if [[ -n "$keepalive_pid" ]] && kill -0 "$keepalive_pid" 2>/dev/null; then
        kill "$keepalive_pid" 2>/dev/null || true
        wait "$keepalive_pid" 2>/dev/null || true
    fi
    [[ -n "$control_path" && -n "$robot" ]] || return 0
    command ssh -o "ControlPath=$control_path" -O exit "$robot" >/dev/null 2>&1 || true
}
