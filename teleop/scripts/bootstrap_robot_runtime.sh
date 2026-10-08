#!/usr/bin/env bash
# Install the ARM64 userspace needed by the robot-local WebXR runtime.
# Runs as the normal robot user; it never touches motors or systemd.
set -euo pipefail

PREFIX="${HB_TELEOP_MINIFORGE_ROOT:-$HOME/.local/share/hb/teleop/miniforge3}"
ENV_NAME="${HB_TELEOP_RUNTIME_ENV:-hb_teleop}"
MINIFORGE_URL="${HB_TELEOP_MINIFORGE_URL:-https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh}"
TMP="${TMPDIR:-/tmp}/miniforge-hb-$$.sh"

[[ "$(uname -m)" == "aarch64" ]] || { echo "[BOOTSTRAP] cần ARM64/aarch64" >&2; exit 2; }
ROOT="${HB_ROOT:-$HOME/HB}"
ENV_PYTHON="$PREFIX/envs/$ENV_NAME/bin/python"
RUNTIME_PYTHONPATH="$ROOT/teleop/src:$ROOT/teleop/third_party/xr_teleoperate/teleop/televuer/src:/home/unitree/Foundation_Unitree/unitree_sdk2_python"
if [[ -x "$ENV_PYTHON" ]] && \
   PYTHONPATH="$RUNTIME_PYTHONPATH" "$ENV_PYTHON" -c '
import aiohttp, casadi, cv2, numpy, pinocchio, vuer, televuer, unitree_sdk2py
from importlib.metadata import version
assert version("vuer") == "0.0.60"
assert version("cyclonedds") == "0.10.2"
' >/dev/null 2>&1; then
    echo "[BOOTSTRAP] OK existing env=$PREFIX/envs/$ENV_NAME"
    exit 0
fi
mkdir -p "$(dirname "$PREFIX")"
if [[ ! -x "$PREFIX/bin/conda" ]]; then
    echo "[BOOTSTRAP] tải Miniforge ARM64..."
    trap 'rm -f "$TMP"' EXIT
    curl -fL --retry 3 "$MINIFORGE_URL" -o "$TMP"
    bash "$TMP" -b -p "$PREFIX"
fi

source "$PREFIX/etc/profile.d/conda.sh"
conda config --set auto_activate_base false >/dev/null
if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "[BOOTSTRAP] tạo env $ENV_NAME"
    conda create -y -n "$ENV_NAME" python=3.10 numpy=1.26 pyyaml opencv casadi pinocchio scipy
fi
conda install -y -n "$ENV_NAME" numpy=1.26 opencv casadi pinocchio scipy pyyaml

echo "[BOOTSTRAP] cài WebXR dependencies"
conda run --no-capture-output -n "$ENV_NAME" python -m pip install --upgrade --no-cache-dir \
    'vuer[all]==0.0.60' 'params-proto==2.13.2' 'logging-mp==0.2.1' \
    'typing-extensions==4.15.0' 'meshcat' 'matplotlib'

# Unitree's Python SDK uses the same CycloneDDS ABI already shipped by the
# robot Foundation tree.  Pin the Python binding to that ABI; the newest
# binding expects headers that are not present in the vendor 0.10.x tree.
CYCLONEDDS_HOME="${HB_TELEOP_CYCLONEDDS_HOME:-/home/unitree/Foundation_Unitree/third_party/cyclonedds-0.10.2/install}"
CYCLONEDDS_INCLUDE="/home/unitree/Foundation_Unitree/third_party/cyclonedds-0.10.2/src/core/ddsi/include"
echo "[BOOTSTRAP] cài CycloneDDS Python binding 0.10.2"
CYCLONEDDS_HOME="$CYCLONEDDS_HOME" CFLAGS="-I$CYCLONEDDS_INCLUDE" \
    conda run --no-capture-output -n "$ENV_NAME" python -m pip install --upgrade --no-cache-dir \
    'cyclonedds==0.10.2'

echo "[BOOTSTRAP] kiểm tra import"
conda run --no-capture-output -n "$ENV_NAME" env \
    PYTHONPATH="$RUNTIME_PYTHONPATH" \
    python -c 'import aiohttp, casadi, cv2, numpy, pinocchio, vuer, televuer, unitree_sdk2py; print("ROBOT_VENDOR_TELEOP_ENV_OK")'
echo "[BOOTSTRAP] OK env=$PREFIX/envs/$ENV_NAME"
