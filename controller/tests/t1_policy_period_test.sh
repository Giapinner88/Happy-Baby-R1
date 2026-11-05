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

# 1. Bundle đang chọn: t1_settle05_iter19999 (1.0s) + teleop u4_seed42 (0.6s).
HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/valid.log" 2>&1
grep -q 'gait_period=1s, teleop_gait_period=0.6s' "$temp/valid.log"
grep -q 'gait_period=1s.*\[bundle\]' "$temp/valid.log"
grep -q 'gait_settle_dwell_s=0.5s (bundle)' "$temp/valid.log"

# 2. Đổi một dòng walk_policy là đủ: chu kỳ và dwell đi theo bundle mới.
sed -i 's|^walk_policy:.*|walk_policy: flat_plus_gait_0917_armfree|' "$config"
HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/swapped.log" 2>&1
grep -q 'gait_period=0.6s' "$temp/swapped.log"
grep -q 'gait_settle_dwell_s=0.5s (bundle)' "$temp/swapped.log"
cp "$pristine" "$config"

# 3. Override lệch metadata vẫn phải chặn khởi động, từng route một.
for key in walk_gait_period_s teleop_gait_period_s; do
    cp "$pristine" "$config"
    value=0.6
    [[ "$key" == teleop_gait_period_s ]] && value=1.0
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
