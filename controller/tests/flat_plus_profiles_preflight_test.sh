#!/usr/bin/env bash
set -euo pipefail

root=${1:?usage: flat_plus_profiles_preflight_test.sh PROJECT_ROOT RUN_R1}
run_r1=${2:?usage: flat_plus_profiles_preflight_test.sh PROJECT_ROOT RUN_R1}
shift 2
tmp=$(mktemp -d /tmp/hb_flat_plus_preflight.XXXXXX)
trap 'rm -rf "$tmp"' EXIT

if (( $# > 0 )); then
    profiles=("$@")
else
    profiles=(h4 gait gait_0917_v2 gait_0917_v3 gait_0917_v4 gait_0917_v5_a1 gait_0917_v5_a2a gait_0917_armfree_settle_override)
fi

for profile in "${profiles[@]}"; do
    mkdir -p "$tmp/$profile/config"
    case "$profile" in
        h4) example=locomotion_flat_plus.example.yaml ;;
        gait) example=locomotion_flat_plus_gait.example.yaml ;;
        gait_0917_v2) example=locomotion_flat_plus_gait_0917_v2.example.yaml ;;
        gait_0917_v3) example=locomotion_flat_plus_gait_0917_v3.example.yaml ;;
        gait_0917_v4) example=locomotion_flat_plus_gait_0917_v4.example.yaml ;;
        gait_0917_v5_a1) example=locomotion_flat_plus_gait_0917_v5_a1.example.yaml ;;
        gait_0917_v5_a2a) example=locomotion_flat_plus_gait_0917_v5_a2a.example.yaml ;;
        gait_0917_armfree_settle_override) example=locomotion_flat_plus_gait_0917_v2.example.yaml ;;
        *) echo "Unknown profile: $profile" >&2; exit 2 ;;
    esac
    cp "$root/config/$example" "$tmp/$profile/config/tuning.yaml"
    if [[ "$profile" == gait_0917_armfree_settle_override ]]; then
        sed -i \
            -e 's/^flat_model:.*/flat_model: policy_flat_plus_gait_0917_armfree.onnx/' \
            -e 's/^gait_settle_dwell_s:.*/gait_settle_dwell_s: 0.50/' \
            "$tmp/$profile/config/tuning.yaml"
    fi
    ln -s "$root/policies" "$tmp/$profile/policies"
    if [[ "$profile" == gait_0917_armfree_settle_override ]]; then
        output=$(HB_PROJECT_DIR="$tmp/$profile" "$run_r1" --preflight 2>&1) || {
            printf '%s\n' "$output" >&2
            exit 1
        }
        printf '%s\n' "$output"
        [[ "$output" == *"gait_settle_dwell_s=0.5s (runtime config)"* ]]
    else
        HB_PROJECT_DIR="$tmp/$profile" "$run_r1" --preflight
    fi
done
echo "flat_plus_profiles_preflight_test: PASS"
