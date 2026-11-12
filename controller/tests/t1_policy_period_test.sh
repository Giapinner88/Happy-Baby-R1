#!/usr/bin/env bash
# Chu kỳ gait đi theo bundle, không theo locomotion.yaml:
#   1. config chuẩn (không có khoá chu kỳ nào) -> lấy đúng chu kỳ của bundle;
#   2. đổi MỖI walk_policy -> chu kỳ đổi theo, không sửa gì khác;
#   3. đặt override lệch metadata -> vẫn bị từ chối khởi động.
# Mục 3 là tripwire cũ: nới chu kỳ ra khỏi YAML không được phép làm mất nó.
set -euo pipefail
root=${1:?PROJECT_ROOT}
binary=${2:?RUN_R1}
temp=$(mktemp -d /tmp/hb-t1-period.XXXXXX)
trap 'rm -rf "$temp"' EXIT
cp -r "$root/config" "$temp/config"
ln -s "$root/policies" "$temp/policies"
ln -s "$root/motions" "$temp/motions"
config="$temp/config/locomotion.yaml"
pristine="$temp/locomotion.pristine.yaml"
cp "$config" "$pristine"

for key in walk_gait_period_s teleop_gait_period_s gait_settle_dwell_s; do
    if grep -q "^${key}:" "$pristine"; then
        echo "locomotion.yaml van con '${key}'; chu ky phai den tu bundle" >&2
        exit 1
    fi
done

# Chu kỳ kỳ vọng đọc TỪ BUNDLE đang chọn, không ghim số: ghim số ở đây thì
# đổi walk_policy lại phải sửa test, đúng thứ việc này muốn bỏ.
bundle_period() {
    sed -n 's/^gait_period_s:[[:space:]]*//p' "$temp/policies/locomotion/$1/policy.yaml" |
        head -1 | sed 's/\.0$//; s/^\([0-9]*\.[0-9]*[1-9]\)0*$/\1/'
}
# Printed period: "<s>s", or "policy" for a contract whose policy owns its
# clock (multigait v2573: no gait_period_s in its bundle).
period_token() {
    local p
    p="$(bundle_period "$1")"
    if [[ -n "$p" ]]; then echo "${p}s"; else echo policy; fi
}
walk_now="$(sed -n 's/^walk_policy:[[:space:]]*//p' "$pristine" | head -1)"
teleop_now="$(sed -n 's/^teleop_policy:[[:space:]]*//p' "$pristine" | head -1)"
[[ -n "$teleop_now" ]] || teleop_now="$walk_now"

# 1. Chu kỳ in ra phải đúng của bundle đang chọn, và phải ghi nguồn [bundle].
HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/valid.log" 2>&1
grep -q "gait_period=$(period_token "$walk_now"), teleop_gait_period=$(period_token "$teleop_now")" \
    "$temp/valid.log"
grep -q 'gait_period=.*\[bundle\]' "$temp/valid.log"

# 2. Đổi một dòng walk_policy là đủ: chu kỳ và dwell đi theo bundle mới. Chọn
#    một bundle có chu kỳ KHÁC bundle đang dùng để phép thử có ý nghĩa.
other=flat_plus_gait_0917_armfree
[[ "$walk_now" != "$other" ]] || other=t1_settle05_iter19999
sed -i "s|^walk_policy:.*|walk_policy: ${other}|" "$config"
HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/swapped.log" 2>&1
grep -q "gait_period=$(bundle_period "$other")s" "$temp/swapped.log"
grep -q "gait_settle_dwell_s=$(sed -n 's/^gait_settle_dwell_s:[[:space:]]*//p' \
    "$temp/policies/locomotion/$other/policy.yaml" | head -1)s (bundle)" "$temp/swapped.log"
cp "$pristine" "$config"

# 3. Override lệch metadata vẫn phải chặn khởi động, từng route một.
for key in walk_gait_period_s teleop_gait_period_s; do
    cp "$pristine" "$config"
    # The override must really differ from the selected bundle period;
    # a fixed value breaks as soon as walk_policy changes.
    route="$walk_now"
    [[ "$key" == teleop_gait_period_s ]] && route="$teleop_now"
    value=0.6
    [[ "$(bundle_period "$route")" == 0.6 ]] && value=1.0
    printf '%s: %s\n' "$key" "$value" >> "$config"
    if HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/invalid.log" 2>&1; then
        echo "preflight accepted a mismatched $key" >&2
        exit 1
    fi
    grep -q 'gait_period_s' "$temp/invalid.log"
done

# 4. Bundle thiếu khoá và YAML cũng không đặt -> báo lỗi rõ, không im lặng chạy
#    bằng một mặc định nào đó.
cp "$pristine" "$config"
mkdir -p "$temp/stripped/policies/locomotion/stripped"
cp "$root/policies/locomotion/t1_settle05_iter19999/policy.onnx" \
   "$temp/stripped/policies/locomotion/stripped/policy.onnx"
grep -vE '^gait_(period_s|settle_dwell_s):' \
    "$root/policies/locomotion/t1_settle05_iter19999/policy.yaml" \
    > "$temp/stripped/policies/locomotion/stripped/policy.yaml"
cp -r "$temp/config" "$temp/stripped/config"
ln -s "$root/motions" "$temp/stripped/motions"
sed -i 's|^walk_policy:.*|walk_policy: stripped|' "$temp/stripped/config/locomotion.yaml"
if HB_PROJECT_DIR="$temp/stripped" "$binary" --preflight > "$temp/stripped.log" 2>&1; then
    echo "preflight accepted a bundle with no gait_period_s" >&2
    exit 1
fi
grep -q 'khong xac dinh duoc gait_period_s' "$temp/stripped.log"

echo 'T1_POLICY_PERIOD_OK: chu ky tu bundle, doi ten la du, override lech van bi tu choi'
