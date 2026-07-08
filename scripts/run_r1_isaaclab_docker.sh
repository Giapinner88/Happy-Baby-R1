#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ISAACLAB_DOCKER_IMAGE:-unitree-sim:latest}"
CONTAINER_WORKDIR="${CONTAINER_WORKDIR:-/workspace/Happy-Baby-R1}"

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

  printf '[ERROR] Docker is not available. Try DOCKER_CMD="sudo docker" %s\n' "$0" >&2
  exit 1
}

if [[ $# -eq 0 ]]; then
  set -- python scripts/r1_policy_workspace.py train rl_lab --num-envs 1 --max-iterations 1
fi

mkdir -p "$ROOT_DIR/data/cache/pip" "$ROOT_DIR/data/cache/isaaclab" "$ROOT_DIR/data/runs/rl_lab" "$ROOT_DIR/data/policies/rl_lab"

docker_cmd run --rm -it \
  --gpus all \
  --network host \
  -e OMNI_KIT_ALLOW_ROOT=1 \
  -e PYTHONNOUSERSITE=1 \
  -e PIP_CACHE_DIR="$CONTAINER_WORKDIR/data/cache/pip" \
  -e XDG_CACHE_HOME="$CONTAINER_WORKDIR/data/cache" \
  -e HAPPY_BABY_R1_ROOT="$CONTAINER_WORKDIR" \
  -v "$ROOT_DIR:$CONTAINER_WORKDIR" \
  -w "$CONTAINER_WORKDIR" \
  "$IMAGE" \
  bash -lc '
    set -Eeuo pipefail
    if command -v conda >/dev/null 2>&1; then
      eval "$(conda shell.bash hook)"
      conda activate unitree_sim_env
    fi
    python -m pip install -e third_party/unitree_rl_lab/source/unitree_rl_lab
    exec "$@"
  ' happy-baby-r1 "$@"
