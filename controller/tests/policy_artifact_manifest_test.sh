#!/usr/bin/env bash
set -euo pipefail

root=${1:?usage: policy_artifact_manifest_test.sh PROJECT_ROOT}
loco="$root/policies/locomotion"

# Archive pool (route flat_model cũ): mọi artifact phải có trong SHA256SUMS.
manifest="$loco/_archive/flat_plus/SHA256SUMS"
test -f "$manifest"
(
    cd "$(dirname "$manifest")"
    sha256sum --quiet -c "$(basename "$manifest")"
    missing=0
    for artifact in *.onnx; do
        if ! awk '{print $2}' SHA256SUMS | grep -qxF "$artifact"; then
            echo "not in SHA256SUMS: $artifact" >&2
            missing=1
        fi
    done
    ((missing == 0))
)

# Bundle: mỗi thư mục chọn được có policy.onnx + policy.yaml, và sha256 trong
# policy.yaml khớp file, để một model mới/đổi không lên robot mà thiếu hash.
bundles=0
for dir in "$loco"/*/; do
    name="$(basename "$dir")"
    [[ "$name" == _* ]] && continue
    [[ -s "$dir/policy.onnx" ]] || { echo "bundle $name: missing policy.onnx" >&2; exit 1; }
    [[ -s "$dir/policy.yaml" ]] || { echo "bundle $name: missing policy.yaml" >&2; exit 1; }
    expected="$(sed -n 's/^sha256:[[:space:]]*"\{0,1\}\([0-9a-f]\{64\}\)"\{0,1\}[[:space:]]*\(#.*\)\{0,1\}$/\1/p' "$dir/policy.yaml")"
    actual="$(sha256sum "$dir/policy.onnx" | awk '{print $1}')"
    [[ "$expected" == "$actual" ]] || {
        echo "bundle $name: policy.yaml sha256=$expected but policy.onnx=$actual" >&2
        exit 1
    }
    grep -Eq '^contract:[[:space:]]*[a-z0-9_]+' "$dir/policy.yaml" || {
        echo "bundle $name: missing contract" >&2; exit 1; }
    bundles=$((bundles + 1))
done
((bundles > 0)) || { echo "no locomotion bundle found under $loco" >&2; exit 1; }
echo "policy_artifact_manifest_test: PASS ($bundles bundle)"
