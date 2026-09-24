#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
simulator="${script_dir}/../simulate/build/unitree_mujoco"
scene="${script_dir}/../unitree_robots/r1/scene_mixed_terrain.xml"

if [[ ! -x "${simulator}" ]]; then
  echo "Simulator binary not found: ${simulator}" >&2
  echo "Build it first in unitree_mujoco/simulate/build." >&2
  exit 1
fi

if [[ ! -f "${scene}" ]]; then
  echo "Mixed terrain scene not found: ${scene}" >&2
  exit 1
fi

echo "Starting mixed-terrain simulator only. This command does not start or stop run_policy."
exec "${simulator}" -r r1 -n lo -s "${scene}" "$@"
