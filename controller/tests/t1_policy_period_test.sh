#!/usr/bin/env bash
set -euo pipefail
root=${1:?PROJECT_ROOT}
binary=${2:?RUN_R1}
temp=$(mktemp -d /tmp/hb-t1-period.XXXXXX)
trap 'rm -rf "$temp"' EXIT
cp -r "$root/config" "$temp/config"
ln -s "$root/policies" "$temp/policies"
ln -s "$root/motions" "$temp/motions"
HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/valid.log" 2>&1
grep -q 'gait_period=1s, teleop_gait_period=0.6s' "$temp/valid.log"
for key in walk_gait_period_s teleop_gait_period_s; do
    cp "$root/config/locomotion.yaml" "$temp/config/locomotion.yaml"
    value=0.6
    [[ "$key" == teleop_gait_period_s ]] && value=1.0
    sed -i "s/^${key}:.*/${key}: ${value}/" "$temp/config/locomotion.yaml"
    if HB_PROJECT_DIR="$temp" "$binary" --preflight > "$temp/invalid.log" 2>&1; then
        echo "preflight accepted a mismatched $key" >&2
        exit 1
    fi
    grep -q 'gait_period_s' "$temp/invalid.log"
done
echo 'T1_POLICY_PERIOD_OK: independent periods, mismatched metadata rejected'
