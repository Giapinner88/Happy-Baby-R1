#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
config="$script_dir/config/tuning.yaml"

if ! grep -Eq '^[[:space:]]*locomotion_policy:[[:space:]]*flat/policy_goc\.onnx([[:space:]]*#.*)?$' "$config"; then
  echo "Flat acceptance requires locomotion_policy: flat/policy_goc.onnx" >&2
  exit 1
fi

# Scene 0° là mặt phẳng tham chiếu riêng, không dùng các scene dốc/Rough.
exec "$script_dir/run_rough_stack.sh" 0 "$@"
