#!/usr/bin/env bash
set -euo pipefail

root=${1:?usage: policy_artifact_manifest_test.sh PROJECT_ROOT}
manifest="$root/policies/locomotion/flat_plus/SHA256SUMS"

test -f "$manifest"
cd "$(dirname "$manifest")"
sha256sum -c "$(basename "$manifest")"

# Every shipped artifact must be covered, so a new or swapped policy cannot
# reach the robot without its checksum being recorded here.
missing=0
for artifact in *.onnx; do
    if ! awk '{print $2}' "$(basename "$manifest")" | grep -qxF "$artifact"; then
        echo "not in SHA256SUMS: $artifact" >&2
        missing=1
    fi
done
((missing == 0))
echo "policy_artifact_manifest_test: PASS"
