#!/usr/bin/env bash
# Install the isolated workstation interpreter for the pinned R1-A5 vendor IK.
set -euo pipefail

DEST="${HB_TELEOP_VENDOR_ENV_DIR:-$HOME/.local/share/hb/teleop/vendor_ik_env}"
if [[ -e "$DEST" ]]; then
    echo "[FAIL] Environment already exists: $DEST" >&2
    exit 2
fi
command -v conda >/dev/null || { echo "[FAIL] conda is required" >&2; exit 2; }
conda create -y -p "$DEST" -c conda-forge \
    python=3.10.20 pip pinocchio=3.1.0 casadi=3.6.7 numpy=1.26.4
"$DEST/bin/python" -m pip install \
    meshcat==0.3.2 matplotlib==3.7.5 logging-mp==0.2.1
"$DEST/bin/python" -c 'import casadi, pinocchio, meshcat, matplotlib, logging_mp, numpy'
echo "[READY] HB_TELEOP_VENDOR_PYTHON=$DEST/bin/python"
