#!/usr/bin/env bash

hb_status_init() {
    if [[ "${HB_COLOR:-0}" == "1" || ( -t 1 && -z "${NO_COLOR:-}" ) ]]; then
        HB_STATUS_GREEN=$'\033[32m'
        HB_STATUS_RED=$'\033[31m'
        HB_STATUS_RESET=$'\033[0m'
    else
        HB_STATUS_GREEN=""
        HB_STATUS_RED=""
        HB_STATUS_RESET=""
    fi
}

hb_ok()   { printf '%s✓%s %s\n' "$HB_STATUS_GREEN" "$HB_STATUS_RESET" "$*"; }
hb_fail() { printf '%s✗%s %s\n' "$HB_STATUS_RED" "$HB_STATUS_RESET" "$*" >&2; }
hb_info() { printf '%s\n' "$*"; }

hb_status_init
