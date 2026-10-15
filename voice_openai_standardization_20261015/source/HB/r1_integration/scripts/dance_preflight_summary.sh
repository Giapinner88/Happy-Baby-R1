#!/usr/bin/env bash

hb_summarize_dance_preflight() {
    local config="$1" log="$2" slot folder line reason
    HB_DANCE_OK=0
    HB_DANCE_FAIL=0
    HB_DANCE_TOTAL=0

    for slot in 2 3 4 5 6 7 8; do
        folder="$(sed -n "s/^dance_${slot}:[[:space:]]*\\([^[:space:]#]*\\).*/\\1/p" "$config" | head -n 1)"
        [[ -n "$folder" && "$folder" != "\"\"" && "$folder" != "''" ]] || continue
        ((HB_DANCE_TOTAL += 1))

        if grep -Fq "[Application] Loaded Mimic $slot (" "$log"; then
            ((HB_DANCE_OK += 1))
            hb_ok "Dance $slot — $folder"
            continue
        fi

        ((HB_DANCE_FAIL += 1))
        line="$(grep -F "[Application] Dance $slot (" "$log" | tail -n 1 || true)"
        case "$line" in
            *"DISABLED:"*) reason="${line##*DISABLED: }" ;;
            *"missing required files"*) reason="thiếu thư mục, NPZ hoặc ONNX" ;;
            *) reason="không được nạp" ;;
        esac
        hb_fail "Dance $slot — $folder: $reason"
    done

    if (( HB_DANCE_TOTAL == 0 )); then
        hb_info "Dance: chưa cấu hình slot nào"
    else
        hb_info "Dance: $HB_DANCE_OK/$HB_DANCE_TOTAL nạp được trong preflight"
    fi
}
