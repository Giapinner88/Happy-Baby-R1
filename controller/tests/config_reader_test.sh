#!/usr/bin/env bash
set -euo pipefail

READER="$1"
ENTRY_CONFIG="$2"

# shellcheck disable=SC1090
source "$READER"

# Ghim route policy đang active để đổi model phải cập nhật manifest và
# preflight cùng lúc; policy legacy vẫn giữ riêng để rollback.
[[ "$(hb_config_get "$ENTRY_CONFIG" walk_policy)" == "flat_plus_gait_0917_armfree" ]]
[[ "$(hb_config_get "$ENTRY_CONFIG" teleop_policy)" == "u5_seed42" ]]
[[ -z "$(hb_config_get "$ENTRY_CONFIG" flat_model)" ]]
# Bundle dùng cùng tập con YAML: script đọc contract bằng chính reader này.
BUNDLE_YAML="$(dirname "$ENTRY_CONFIG")/../policies/locomotion/flat_plus_gait_0917_armfree/policy.yaml"
[[ "$(hb_config_get "$BUNDLE_YAML" contract)" == "flat_plus_gait_h4_v1" ]]
read -r rel contract < <(hb_selected_policy "$ENTRY_CONFIG" walk_policy)
[[ "$rel" == "policies/locomotion/flat_plus_gait_0917_armfree/policy.onnx" ]]
[[ "$contract" == "flat_plus_gait_h4_v1" ]]
read -r rel contract < <(hb_selected_policy "$ENTRY_CONFIG" teleop_policy)
[[ "$rel" == "policies/locomotion/u5_seed42/policy.onnx" ]]
[[ "$contract" == "r1_wb_legs_gait_h4_v1_blind" ]]
[[ "$(hb_config_get "$ENTRY_CONFIG" network_interface)" == "eth10" ]]

TMP_DIR="$(mktemp -d /tmp/hb-config-reader-test.XXXXXX)"
trap 'rm -rf -- "$TMP_DIR"' EXIT
mkdir -p "$TMP_DIR/nested"
printf '%s\n' \
    'selected: root-before' \
    'include: nested/child.yaml' \
    'selected: root-after' >"$TMP_DIR/root.yaml"
printf '%s\n' \
    'selected: child' \
    'include: grandchild.yaml' >"$TMP_DIR/nested/child.yaml"
printf '%s\n' 'selected: grandchild' >"$TMP_DIR/nested/grandchild.yaml"
[[ "$(hb_config_get "$TMP_DIR/root.yaml" selected)" == "root-after" ]]

printf '%s\n' 'include: cycle-b.yaml' >"$TMP_DIR/cycle-a.yaml"
printf '%s\n' 'include: cycle-a.yaml' >"$TMP_DIR/cycle-b.yaml"
if hb_config_get "$TMP_DIR/cycle-a.yaml" selected >/dev/null 2>&1; then
    echo "Expected include cycle to fail" >&2
    exit 1
fi

# A grouped selector resolves exactly as a flat one for both deploy roles.
mkdir -p "$TMP_DIR/config" "$TMP_DIR/policies/locomotion/armfree/test"
printf '%s\n' 'contract: flat_plus_gait_h4_v1' >"$TMP_DIR/policies/locomotion/armfree/test/policy.yaml"
printf '%s\n' 'walk_policy: armfree/test' 'teleop_policy: ""' >"$TMP_DIR/config/tuning.yaml"
for role in walk_policy teleop_policy; do
    [[ "$(hb_selected_policy "$TMP_DIR/config/tuning.yaml" "$role")" == \
        'policies/locomotion/armfree/test/policy.onnx flat_plus_gait_h4_v1' ]]
done

# The user-facing add/list workflow must also understand grouped selectors.
mkdir -p "$TMP_DIR/integration/scripts" "$TMP_DIR/controller/config"
cp "$READER" "$TMP_DIR/integration/scripts/config_reader.sh"
cp "$(dirname "$READER")/policy_bundles.sh" "$TMP_DIR/integration/scripts/policy_bundles.sh"
printf '%s\n' 'walk_policy: group/example' 'teleop_policy: group/example' >"$TMP_DIR/controller/config/tuning.yaml"
printf 'fixture actor bytes' >"$TMP_DIR/source.onnx"
bash "$TMP_DIR/integration/scripts/policy_bundles.sh" add "$TMP_DIR/source.onnx" group/example >/dev/null
[[ -s "$TMP_DIR/controller/policies/locomotion/group/example/policy.yaml" ]]
listing="$(bash "$TMP_DIR/integration/scripts/policy_bundles.sh" list)"
[[ "$listing" == *group/example* && "$listing" == *'walk(R2+A)'* && "$listing" == *'teleop(R2+X)'* ]]
for name in ../escape group//bad group/../escape /absolute group/; do
    if bash "$TMP_DIR/integration/scripts/policy_bundles.sh" add "$TMP_DIR/source.onnx" "$name" >/dev/null 2>&1; then
        echo "Expected add to reject invalid selector '$name'" >&2; exit 1
    fi
done
if bash "$TMP_DIR/integration/scripts/policy_bundles.sh" add "$TMP_DIR/source.onnx" group/example >/dev/null 2>&1; then
    echo 'Expected add to reject an existing bundle' >&2; exit 1
fi
for name in '' . .. ../u2a_seed42 /armfree/test armfree/../u2a_seed42 \
        armfree//test armfree/./test armfree/test/ 'armfree\test' \
        _archive/flat_plus armfree/_hidden 'armfree/name with spaces'; do
    if hb_valid_policy_bundle_name "$name"; then
        echo "Expected invalid bundle selector to fail: '$name'" >&2; exit 1
    fi
    # Empty walk means legacy route, so use an invalid teleop selector here.
    if [[ -n "$name" ]]; then
        printf '%s\n' 'walk_policy: armfree/test' "teleop_policy: $name" >"$TMP_DIR/config/tuning.yaml"
        if hb_selected_policy "$TMP_DIR/config/tuning.yaml" teleop_policy >/dev/null 2>&1; then
            echo "Expected selected invalid bundle path to fail: '$name'" >&2; exit 1
        fi
    fi
done
