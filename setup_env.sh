#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_NAME="${ENV_NAME:-r1_env}"
PYTHON_VERSION="${PYTHON_VERSION:-3.10}"
INSTALL_MJLAB="${INSTALL_MJLAB:-1}"
INSTALL_ISAACLAB_DOCKER="${INSTALL_ISAACLAB_DOCKER:-1}"
BUILD_ISAACLAB_DOCKER="${BUILD_ISAACLAB_DOCKER:-1}"
ISAACLAB_DOCKER_IMAGE="${ISAACLAB_DOCKER_IMAGE:-unitree-sim:latest}"
PIP_CACHE_DIR="${PIP_CACHE_DIR:-$ROOT_DIR/data/cache/pip}"

export PYTHONNOUSERSITE=1
export PIP_CACHE_DIR
export HAPPY_BABY_R1_ROOT="$ROOT_DIR"

log() {
  printf '\n[%s] %s\n' "$(date '+%H:%M:%S')" "$*"
}

die() {
  printf '\n[ERROR] %s\n' "$*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"
}

conda_run() {
  conda run -n "$ENV_NAME" "$@"
}

docker_cmd() {
  if [[ -n "${DOCKER_CMD:-}" ]]; then
    read -r -a _docker_parts <<<"$DOCKER_CMD"
    "${_docker_parts[@]}" "$@"
    return $?
  fi

  if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    docker "$@"
    return $?
  fi

  if command -v sudo >/dev/null 2>&1 && command -v docker >/dev/null 2>&1 && sudo -n docker info >/dev/null 2>&1; then
    sudo docker "$@"
    return $?
  fi

  die "Docker is not available to this shell. Install Docker/NVIDIA Container Toolkit, join the docker group, or run with DOCKER_CMD='sudo docker'."
}

ensure_conda_env() {
  need_cmd conda
  if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    log "Using existing conda env: $ENV_NAME"
  else
    log "Creating conda env: $ENV_NAME python=$PYTHON_VERSION"
    conda create -n "$ENV_NAME" "python=$PYTHON_VERSION" -y
  fi
}

ensure_dirs() {
  mkdir -p \
    "$ROOT_DIR/data/cache/pip" \
    "$ROOT_DIR/data/cache/warp" \
    "$ROOT_DIR/data/runs" \
    "$ROOT_DIR/data/policies" \
    "$ROOT_DIR/data/models"
}

install_mjlab_stack() {
  if [[ "$INSTALL_MJLAB" != "1" ]]; then
    log "Skipping MJLab host install. Set INSTALL_MJLAB=1 to enable it."
    return 0
  fi

  log "Installing MJLab-compatible stack into $ENV_NAME"
  conda_run python -m pip install --upgrade pip setuptools wheel

  # Keep these pins aligned with the Unitree RL MJLab checkout used by this workspace.
  # Newer mjlab checkouts removed APIs that this Unitree tree still imports.
  conda_run python -m pip install --force-reinstall \
    "mjlab==1.2.0" \
    "mujoco==3.9.0" \
    "mujoco-warp==3.9.0.1" \
    "rsl-rl-lib==5.0.1"

  log "Installing Unitree RL MJLab editable package without changing third_party source"
  conda_run python -m pip install --no-build-isolation --no-deps -e "$ROOT_DIR/third_party/unitree_rl_mjlab"
}

install_unitree_sdk_python() {
  if [[ "$INSTALL_MJLAB" != "1" ]]; then
    return 0
  fi

  if [[ -d "$ROOT_DIR/third_party/unitree_sdk2_python" ]]; then
    log "Installing Unitree SDK2 Python editable package"
    conda_run python -m pip install --no-build-isolation -e "$ROOT_DIR/third_party/unitree_sdk2_python"
  else
    log "Skipping Unitree SDK2 Python: third_party/unitree_sdk2_python not found"
  fi
}

install_isaaclab_docker_stack() {
  if [[ "$INSTALL_ISAACLAB_DOCKER" != "1" ]]; then
    log "Skipping IsaacLab Docker setup. Set INSTALL_ISAACLAB_DOCKER=1 to enable it."
    return 0
  fi

  [[ -f "$ROOT_DIR/third_party/unitree_sim_isaaclab/Dockerfile" ]] || die "Missing third_party/unitree_sim_isaaclab/Dockerfile"
  [[ -d "$ROOT_DIR/third_party/unitree_rl_lab" ]] || die "Missing third_party/unitree_rl_lab"

  if docker_cmd image inspect "$ISAACLAB_DOCKER_IMAGE" >/dev/null 2>&1 && [[ "$BUILD_ISAACLAB_DOCKER" != "force" ]]; then
    log "Using existing IsaacLab Docker image: $ISAACLAB_DOCKER_IMAGE"
  elif [[ "$BUILD_ISAACLAB_DOCKER" == "1" || "$BUILD_ISAACLAB_DOCKER" == "force" ]]; then
    log "Building IsaacLab Docker image from third_party/unitree_sim_isaaclab: $ISAACLAB_DOCKER_IMAGE"
    docker_cmd build \
      -t "$ISAACLAB_DOCKER_IMAGE" \
      -f "$ROOT_DIR/third_party/unitree_sim_isaaclab/Dockerfile" \
      "$ROOT_DIR/third_party/unitree_sim_isaaclab"
  else
    log "IsaacLab Docker image is missing and BUILD_ISAACLAB_DOCKER=$BUILD_ISAACLAB_DOCKER, so it was not built."
  fi

  log "IsaacLab training will run through scripts/run_r1_isaaclab_docker.sh"
}

verify_mjlab() {
  if [[ "$INSTALL_MJLAB" != "1" ]]; then
    return 0
  fi

  log "Verifying MJLab/R1 imports"
  conda_run python -c "import mjlab, src, mujoco, mujoco_warp, warp, rsl_rl; import mjlab.utils.os as osmod; assert hasattr(osmod, 'update_assets'); print('MJLab stack OK')"
}

verify_isaaclab_docker_if_available() {
  if [[ "$INSTALL_ISAACLAB_DOCKER" != "1" ]]; then
    return 0
  fi

  log "Checking IsaacLab Docker image"
  if docker_cmd image inspect "$ISAACLAB_DOCKER_IMAGE" >/dev/null 2>&1; then
    log "IsaacLab Docker image is available: $ISAACLAB_DOCKER_IMAGE"
  else
    log "IsaacLab Docker image is not available yet: $ISAACLAB_DOCKER_IMAGE"
  fi
}

print_next_steps() {
  cat <<EOF

Setup complete for env: $ENV_NAME

MJLab smoke train:
  PYTHONNOUSERSITE=1 conda run -n $ENV_NAME python scripts/r1_policy_workspace.py train mjlab \\
    --terrain flat --num-envs 1 --max-iterations 1 --run-name smoke \\
    --agent.save-interval=1 --gpu-ids None

Collect policy:
  python scripts/r1_policy_workspace.py collect mjlab

IsaacLab short train through Docker:
  scripts/run_r1_isaaclab_docker.sh python scripts/r1_policy_workspace.py train rl_lab \\
    --num-envs 1 --max-iterations 1

All generated outputs go under:
  data/runs/
  data/policies/
  data/cache/
EOF
}

main() {
  cd "$ROOT_DIR"
  ensure_dirs
  ensure_conda_env
  install_mjlab_stack
  install_unitree_sdk_python
  install_isaaclab_docker_stack
  verify_mjlab
  verify_isaaclab_docker_if_available
  print_next_steps
}

main "$@"
