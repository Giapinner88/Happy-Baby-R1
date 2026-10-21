#!/usr/bin/env bash
set -euo pipefail

READER="$1"
ENTRY_CONFIG="$2"

# shellcheck disable=SC1090
source "$READER"

# Ghim route policy đang active để đổi model phải cập nhật manifest và
# preflight cùng lúc; policy legacy vẫn giữ riêng để rollback.
[[ "$(hb_config_get "$ENTRY_CONFIG" walk_policy)" == "flat_plus_gait_0917_armfree" ]]
[[ "$(hb_config_get "$ENTRY_CONFIG" teleop_policy)" == "u2a_seed42" ]]
[[ -z "$(hb_config_get "$ENTRY_CONFIG" flat_model)" ]]
# Bundle dùng cùng tập con YAML: script đọc contract bằng chính reader này.
BUNDLE_YAML="$(dirname "$ENTRY_CONFIG")/../policies/locomotion/flat_plus_gait_0917_armfree/policy.yaml"
[[ "$(hb_config_get "$BUNDLE_YAML" contract)" == "flat_plus_gait_h4_v1" ]]
read -r rel contract < <(hb_selected_policy "$ENTRY_CONFIG" walk_policy)
[[ "$rel" == "policies/locomotion/flat_plus_gait_0917_armfree/policy.onnx" ]]
[[ "$contract" == "flat_plus_gait_h4_v1" ]]
read -r rel contract < <(hb_selected_policy "$ENTRY_CONFIG" teleop_policy)
[[ "$rel" == "policies/locomotion/u2a_seed42/policy.onnx" ]]
[[ "$contract" == "r1_wb_legs_v1_blind" ]]
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
