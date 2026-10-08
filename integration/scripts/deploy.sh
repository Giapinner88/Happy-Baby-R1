#!/usr/bin/env bash
# One-command deploy from the HB repository root.
# It accepts the model selected by config/locomotion.yaml, then delegates all robot
# actions to deploy_stack.sh (backup, sync, ARM64 build, preflight, safe restart,
# including the robot-local WebXR/vendor IK runtime).
set -euo pipefail

HB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MIN_TMP_MB="${HB_DEPLOY_MIN_TMP_MB:-2048}"

if (($# != 0)); then
    echo "Usage: $0" >&2
    echo "Advanced actions: $HB_ROOT/integration/scripts/deploy_stack.sh <command>" >&2
    exit 2
fi

AVAILABLE_KB="$(df -Pk /tmp | awk 'NR == 2 {print $4}')"
REQUIRED_KB=$((MIN_TMP_MB * 1024))
if [[ ! "$AVAILABLE_KB" =~ ^[0-9]+$ ]] || (( AVAILABLE_KB < REQUIRED_KB )); then
    echo "Need at least ${MIN_TMP_MB} MiB free in /tmp for local model preflight; free space first." >&2
    exit 1
fi

exec "$HB_ROOT/integration/scripts/deploy_stack.sh" deploy --accept-policy
