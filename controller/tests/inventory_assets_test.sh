#!/usr/bin/env bash
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="$(cd "$TEST_DIR/../.." && pwd)"
INVENTORY="$HB_ROOT/controller/tools/inventory_assets.sh"
OUTPUT="$(mktemp /tmp/hb-inventory-test.XXXXXX.tsv)"
trap 'rm -f -- "$OUTPUT"' EXIT

"$INVENTORY" --root "$HB_ROOT" --output "$OUTPUT"

EXPECTED_HEADER=$'kind\tpath\tstatus\tonnx\tnpz\taudio\tmanifest\tsha256'
[[ "$(head -n 1 "$OUTPUT")" == "$EXPECTED_HEADER" ]]

grep -Fq $'policy\tcontroller/policies/locomotion/flat_plus\t' "$OUTPUT"
grep -Fq $'policy\tcontroller/policies/dance/' "$OUTPUT"
grep -Fq $'generated\tcontroller/build\t' "$OUTPUT"

# Generated build output must never be reported as a deployable policy package.
! awk -F '\t' '$1 == "policy" && $2 ~ /(^|\/)build($|\/)/ { found = 1 } END { exit found }' "$OUTPUT"

echo "inventory_assets_test: PASS"
