#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
angle="${1:-}"

case "${angle}" in
  0)
    scene_name="scene_slope_0.xml"
    scene_label="flat 0-degree reference"
    ;;
  5|10|15|20|25|30|35|40)
    scene_name="scene_slope_${angle}.xml"
    scene_label="direct ${angle}-degree slope (no flat patch)"
    ;;
  5-platform|10-platform|15-platform|20-platform|25-platform|30-platform)
    slope_angle="${angle%-platform}"
    scene_name="scene_slope_${slope_angle}_platform.xml"
    scene_label="${slope_angle}-degree ramps with a 2 m flat spawn platform"
    ;;
  *)
    echo "Usage: $0 {0|5|10|15|20|25|30|35|40|5-platform|10-platform|15-platform|20-platform|25-platform|30-platform} [extra unitree_mujoco options]" >&2
    echo "Example: $0 40" >&2
    exit 2
    ;;
esac
shift

simulator="${script_dir}/../simulate/build/unitree_mujoco"
scene="${script_dir}/../unitree_robots/r1/${scene_name}"

if [[ ! -x "${simulator}" ]]; then
  echo "Simulator binary not found: ${simulator}" >&2
  echo "Build it first in unitree_mujoco/simulate/build." >&2
  exit 1
fi

if [[ ! -f "${scene}" ]]; then
  echo "Slope scene not found: ${scene}" >&2
  exit 1
fi

echo "Starting R1 on ${scene_label} (W: +X/uphill, S: -X/downhill)"
exec "${simulator}" -r r1 -n lo -s "${scene}" "$@"
