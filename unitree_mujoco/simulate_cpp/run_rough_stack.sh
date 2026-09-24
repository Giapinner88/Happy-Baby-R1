#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
scene_choice="${1:-default}"
if [[ $# -gt 0 ]]; then shift; fi

case "${scene_choice}" in
  default)
    scene="${script_dir}/../unitree_robots/r1/scene.xml"
    ;;
  mixed)
    scene="${script_dir}/../unitree_robots/r1/scene_mixed_terrain.xml"
    ;;
  0|5|10|15|20|25|30|35|40)
    scene="${script_dir}/../unitree_robots/r1/scene_slope_${scene_choice}.xml"
    ;;
  5-platform|10-platform|15-platform|20-platform|25-platform|30-platform)
    slope_angle="${scene_choice%-platform}"
    scene="${script_dir}/../unitree_robots/r1/scene_slope_${slope_angle}_platform.xml"
    ;;
  *)
    if [[ "${scene_choice}" = /* ]]; then
      scene="${scene_choice}"
    else
      scene="$(realpath -e -- "${scene_choice}")"
    fi
    ;;
esac

policy="${script_dir}/build/run_policy"
simulator="${script_dir}/../simulate/build/unitree_mujoco"

if [[ ! -x "${policy}" ]]; then
  echo "Policy binary not found: ${policy}" >&2
  exit 1
fi
if [[ ! -x "${simulator}" ]]; then
  echo "Simulator binary not found: ${simulator}" >&2
  exit 1
fi
if [[ ! -f "${scene}" ]]; then
  echo "Scene not found: ${scene}" >&2
  exit 1
fi
if pgrep -x run_policy >/dev/null || pgrep -x unitree_mujoco >/dev/null; then
  echo "A run_policy or unitree_mujoco process is already running." >&2
  echo "Close the old stack first so scene auto-detection is unambiguous." >&2
  exit 1
fi

policy_log="$(mktemp /tmp/r1_rough_policy.XXXXXX.log)"
policy_pid=""
sim_pid=""
sim_process_group=""

is_sim_alive() {
  if [[ -n "${sim_process_group}" ]]; then
    kill -0 -- "-${sim_process_group}" 2>/dev/null
  else
    [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null
  fi
}

signal_sim() {
  local signal="$1"
  if [[ -n "${sim_process_group}" ]]; then
    kill "-${signal}" -- "-${sim_process_group}" 2>/dev/null
  elif [[ -n "${sim_pid}" ]]; then
    kill "-${signal}" "${sim_pid}" 2>/dev/null
  fi
}

cleanup() {
  trap - EXIT INT TERM
  set +e
  is_sim_alive && signal_sim TERM
  if [[ -n "${policy_pid}" ]] && kill -0 "${policy_pid}" 2>/dev/null; then
    kill -INT "${policy_pid}" 2>/dev/null
  fi
  for _ in {1..30}; do
    sim_alive=0
    policy_alive=0
    is_sim_alive && sim_alive=1
    [[ -n "${policy_pid}" ]] && kill -0 "${policy_pid}" 2>/dev/null && policy_alive=1
    (( sim_alive == 0 && policy_alive == 0 )) && break
    sleep 0.1
  done
  is_sim_alive && signal_sim KILL
  if [[ -n "${policy_pid}" ]] && kill -0 "${policy_pid}" 2>/dev/null; then
    kill -TERM "${policy_pid}" 2>/dev/null
  fi
  [[ -n "${sim_pid}" ]] && wait "${sim_pid}" 2>/dev/null
  [[ -n "${policy_pid}" ]] && wait "${policy_pid}" 2>/dev/null
  echo "Policy log: ${policy_log}"
}
trap cleanup EXIT INT TERM

echo "[1/2] Loading locomotion policy before simulator..."
(
  cd -- "${script_dir}/build"
  exec stdbuf -oL -eL ./run_policy "$@"
) > >(tee "${policy_log}") 2>&1 &
policy_pid=$!

deadline=$((SECONDS + 30))
until grep -Fq "POLICY ĐÃ NẠP XONG" "${policy_log}"; do
  if ! kill -0 "${policy_pid}" 2>/dev/null; then
    echo "Policy exited before it became ready." >&2
    exit 1
  fi
  if (( SECONDS >= deadline )); then
    echo "Timed out waiting 30 seconds for the policy to load." >&2
    exit 1
  fi
  sleep 0.1
done

echo "[2/2] Policy ready; starting simulator with scene: ${scene}"
simulator_args=(-r r1 -n lo -s "${scene}")
if [[ -n "${MUJOCO_RECORD_PATH:-}" ]]; then
  simulator_args+=(
    --record "${MUJOCO_RECORD_PATH}"
    --record-width "${MUJOCO_RECORD_WIDTH:-1280}"
    --record-height "${MUJOCO_RECORD_HEIGHT:-720}"
    --record-fps "${MUJOCO_RECORD_FPS:-30}"
    --record-camera-distance "${MUJOCO_RECORD_CAMERA_DISTANCE:-5.0}"
    --record-camera-azimuth "${MUJOCO_RECORD_CAMERA_AZIMUTH:-135.0}"
    --record-camera-elevation "${MUJOCO_RECORD_CAMERA_ELEVATION:--15.0}"
  )
  echo "[RECORDER] Direct MuJoCo camera output: ${MUJOCO_RECORD_PATH}"
fi
if [[ "${MUJOCO_HEADLESS:-0}" == "1" ]]; then
  if ! command -v xvfb-run >/dev/null 2>&1; then
    echo "MUJOCO_HEADLESS=1 nhưng không tìm thấy xvfb-run." >&2
    exit 1
  fi
  echo "[HEADLESS] Starting MuJoCo in an isolated Xvfb display"
  setsid xvfb-run -a -s "-screen 0 1280x720x24" \
    "${simulator}" "${simulator_args[@]}" &
  sim_process_group=$!
else
  "${simulator}" "${simulator_args[@]}" &
fi
sim_pid=$!

set +e
wait -n "${policy_pid}" "${sim_pid}"
status=$?
set -e
exit "${status}"
