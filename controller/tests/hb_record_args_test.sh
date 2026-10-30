#!/usr/bin/env bash
# ./hb record (máy trạm -> robot): tham số tới sensor_log trên robot phải đúng từng phần tử,
# kể cả khi người dùng không truyền gì (lỗi cũ: printf '%q ' trên mảng rỗng sinh tham số '').
# Robot được giả lập trong thư mục tạm: ssh/scp chạy cục bộ với /home/unitree đổi đường dẫn,
# cmake --build tạo một sensor_log giả ghi lại argv. Không cần mạng, robot hay DDS.
set -euo pipefail

HB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WORK="$(mktemp -d /tmp/hb-record-test.XXXXXX)"
trap 'rm -rf -- "$WORK"' EXIT
ROBOT="$WORK/robot"; STUB="$WORK/stub"; mkdir -p "$ROBOT" "$STUB"

cat >"$STUB/tailscale" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
cat >"$STUB/ssh" <<EOF
#!/usr/bin/env bash
while [[ "\$1" == -* ]]; do shift; done; shift
exec bash -c "\$(printf '%s' "\$*" | sed "s#/home/unitree#$ROBOT#g")"
EOF
cat >"$STUB/scp" <<EOF
#!/usr/bin/env bash
args=(); for a in "\$@"; do args+=("\$(sed "s#/home/unitree#$ROBOT#g" <<<"\${a#*:}")"); done
exec cp "\${args[@]}"
EOF
cat >"$STUB/cmake" <<'EOF'
#!/usr/bin/env bash
[[ "$1" == --build ]] || exit 0
mkdir -p "$2"
cat >"$2/sensor_log" <<'LOG'
#!/usr/bin/env bash
printf '%s\n' "$@" >"$HB_TEST_ARGV"
out=""
while (( $# )); do
    [[ -n "$1" ]] || { echo "empty argument" >&2; exit 2; }
    [[ "$1" == --out ]] && out="$2"
    shift
done
echo "sample" >"$out"; echo "{}" >"$out.meta.json"
LOG
chmod +x "$2/sensor_log"
EOF
chmod +x "$STUB"/*

check() {  # check <expected argv after --out FILE, one per line> -- <hb record args...>
    local expected="$1"; shift
    local pulled="$WORK/pulled"; rm -rf "$pulled"
    PATH="$STUB:$PATH" HB_MODE=dev HB_LOG_DIR="$pulled" HB_TEST_ARGV="$WORK/argv" \
        UNITREE_NETWORK_INTERFACE=eth10 "$HB_ROOT/hb" record "$@" </dev/null >"$WORK/out.txt" 2>&1 || {
        cat "$WORK/out.txt" >&2; echo "FAIL: ./hb record $* exited non-zero" >&2; exit 1; }
    local csv; csv="$(sed -n 4p "$WORK/argv")"
    [[ "$csv" == "$ROBOT/hb_logs/sensors_"*.csv ]] || { echo "FAIL: bad --out '$csv'" >&2; exit 1; }
    local want; want="$(printf '%s\n' --iface eth10 --out "$csv" $expected)"
    [[ "$(cat "$WORK/argv")" == "$want" ]] || {
        echo "FAIL: argv for '$*'"; diff <(echo "$want") "$WORK/argv"; exit 1; } >&2
    [[ -s "$pulled/$(basename "$csv")" && -s "$pulled/$(basename "$csv").meta.json" ]] || {
        echo "FAIL: CSV not pulled for '$*'" >&2; exit 1; }
}

check ""
check "--duration 5 --decimate 10 --all-motors --no-bms" --duration 5 --decimate 10 --all-motors --no-bms
echo "hb_record_args_test: PASS"
