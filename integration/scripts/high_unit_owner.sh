#!/usr/bin/env bash
# hb_high_level là unit DUY NHẤT ghi rt/lowcmd. Preflight chỉ có nghĩa khi unit
# chạy đúng cây đang được kiểm: một drop-in (vd. p5.conf ngày 2026-10-28) từng đổi
# ExecStart + HB_PROJECT_DIR sang cây khác, nên deploy "thành công" mà robot chạy
# code/config khác, và preflight trong ExecStartPre đối chiếu config của hai cây.
#
# hb_check_high_unit_owner <high_dir>: in lý do và trả 1 nếu unit lệch.
# HB_SYSTEMCTL ghi đè lệnh systemctl (test).

hb_check_high_unit_owner() {
    local high_dir="$1" unit="hb_high_level.service"
    local systemctl_bin="${HB_SYSTEMCTL:-systemctl}"
    local expect_bin="$high_dir/build/run_r1" expect_wd="$high_dir/build"
    local show load_state="" dropins="" workdir="" env_line="" exec_paths=() bad=0
    local line key value path

    # Lệnh process hiện tại (ExecStartPre thừa hưởng Environment của unit,
    # preflight.sh đã source /etc/hb/stack.env).
    if [[ -n "${HB_PROJECT_DIR:-}" && "${HB_PROJECT_DIR%/}" != "${high_dir%/}" ]]; then
        echo "HB_PROJECT_DIR=$HB_PROJECT_DIR trong moi truong hien tai, khac $high_dir"
        bad=1
    fi

    if ! command -v "$systemctl_bin" >/dev/null 2>&1; then
        echo "INFO khong co systemctl; bo qua kiem tra unit $unit"
        return "$bad"
    fi
    show="$("$systemctl_bin" show "$unit" --no-pager \
        -p LoadState -p DropInPaths -p WorkingDirectory -p Environment -p ExecStart 2>/dev/null)" || {
        echo "INFO systemctl show $unit loi; bo qua kiem tra unit"
        return "$bad"
    }
    while IFS= read -r line; do
        key="${line%%=*}"
        value="${line#*=}"
        case "$key" in
            LoadState) load_state="$value" ;;
            DropInPaths) dropins="$value" ;;
            WorkingDirectory) workdir="$value" ;;
            Environment) env_line="$value" ;;
            ExecStart)
                path="$(sed -n 's/.*path=\([^ ;]*\).*/\1/p' <<<"$value")"
                [[ -n "$path" ]] && exec_paths+=("$path")
                ;;
        esac
    done <<<"$show"

    if [[ "$load_state" != "loaded" ]]; then
        echo "INFO $unit chua cai (LoadState=${load_state:-?}); bo qua kiem tra unit"
        return "$bad"
    fi
    if [[ -n "$dropins" ]]; then
        echo "$unit co drop-in ghi de unit HB: $dropins (chuyen ra khoi /etc/systemd/system/$unit.d roi daemon-reload)"
        bad=1
    fi
    if ((${#exec_paths[@]} != 1)) || [[ "${exec_paths[0]}" != "$expect_bin" ]]; then
        echo "$unit ExecStart=${exec_paths[*]:-<trong>}, ky vong $expect_bin"
        bad=1
    fi
    if [[ -n "$workdir" && "${workdir%/}" != "$expect_wd" ]]; then
        echo "$unit WorkingDirectory=$workdir, ky vong $expect_wd"
        bad=1
    fi
    local item
    for item in $env_line; do
        if [[ "$item" == HB_PROJECT_DIR=* && "${item#HB_PROJECT_DIR=}" != "$high_dir" ]]; then
            echo "$unit dat $item, khac $high_dir"
            bad=1
        fi
    done
    return "$bad"
}
