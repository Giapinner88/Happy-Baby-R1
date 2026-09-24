#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
expert="${1:-}"
mode="${2:-full}"
output_dir="${3:-/tmp/${expert}_expert_acceptance_$(date +%Y%m%d_%H%M%S)}"

case "${expert}" in
  flat|slope_up|slope_down) ;;
  *)
    echo "Usage: $0 {flat|slope_up|slope_down} [full|smoke|list] [output_dir]" >&2
    exit 2
    ;;
esac
if [[ "${mode}" != "full" && "${mode}" != "smoke" && "${mode}" != "list" ]]; then
  echo "Usage: $0 {flat|slope_up|slope_down} [full|smoke|list] [output_dir]" >&2
  exit 2
fi

mkdir -p -- "${output_dir}"
summary="${output_dir}/summary.csv"
printf 'expert,scene,angle_deg,category,direction,vx,vy,yaw,control_result,process_result,exit_code,log,csv\n' > "${summary}"
control_failures=0
process_errors=0
cases=0

run_case() {
  local scene="$1" angle="$2" category="$3" direction="$4"
  local vx="$5" vy="$6" yaw="$7" hold="$8"
  local slug="${scene}_${category}_${direction}"
  local log="${output_dir}/${slug}.log"
  local trace="${output_dir}/${slug}.csv"
  local result="NOT_RUN" process_result="NOT_RUN" status=0 result_line=""

  ((cases += 1))
  if [[ "${mode}" != "list" ]]; then
    echo "[EXPERT_ACCEPTANCE] expert=${expert} ${cases}: scene=${scene} angle=${angle} category=${category} direction=${direction} cmd=(${vx},${vy},${yaw})"
    set +e
    "${script_dir}/run_sim.sh" expert "${expert}" "${scene}" \
      --autotest locomotion gesture=0 warmup=2 "hold=${hold}" \
      "vx=${vx}" "vy=${vy}" "yaw=${yaw}" \
      max_tilt_deg=25 max_drop=0.25 max_drift=0.30 "csv=${trace}" \
      > "${log}" 2>&1
    status=$?
    set -e
    result_line="$(grep -E 'RESULT=(PASS|FAIL)' "${log}" | tail -n 1 || true)"
    if [[ "${result_line}" == *"RESULT=PASS"* ]]; then
      result="PASS"
    else
      result="FAIL"
      ((control_failures += 1))
    fi
    if [[ -n "${result_line}" && ( ${status} -eq 0 || ${status} -eq 2 || ${status} -eq 3 ) ]]; then
      process_result="OK"
    else
      process_result="ERROR"
      ((process_errors += 1))
    fi
    echo "[EXPERT_ACCEPTANCE] control=${result} process=${process_result} exit=${status} log=${log}"
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${expert}" "${scene}" "${angle}" "${category}" "${direction}" \
    "${vx}" "${vy}" "${yaw}" "${result}" "${process_result}" \
    "${status}" "${log}" "${trace}" >> "${summary}"
}

run_angle() {
  local angle="$1"
  local direct_scene="${angle}" transition_scene="${angle}-platform"
  if [[ "${angle}" == "0" ]]; then
    direct_scene="flat"
    transition_scene="flat"
  fi
  run_case "${direct_scene}" "${angle}" STAND neutral 0 0 0 6
  run_case "${transition_scene}" "${angle}" STRAIGHT up 0.3 0 0 8
  run_case "${transition_scene}" "${angle}" STRAIGHT down -0.3 0 0 8
  if [[ "${mode}" == "smoke" ]]; then return; fi
  run_case "${transition_scene}" "${angle}" DIAGONAL up 0.25 0.20 0 8
  run_case "${transition_scene}" "${angle}" DIAGONAL down -0.25 0.20 0 8
  run_case "${direct_scene}" "${angle}" CROSS left 0 0.3 0 6
  run_case "${direct_scene}" "${angle}" CROSS right 0 -0.3 0 6
  run_case "${direct_scene}" "${angle}" TURN left 0 0 0.5 6
  run_case "${direct_scene}" "${angle}" TURN right 0 0 -0.5 6
}

for angle in 0 5 10 15 20 25 30; do run_angle "${angle}"; done
run_case mixed mixed STRAIGHT traverse 0.4 0 0 48

echo "[EXPERT_ACCEPTANCE] expert=${expert} cases=${cases} control_failures=${control_failures} process_errors=${process_errors} summary=${summary}"
if [[ "${mode}" == "list" ]]; then exit 0; fi
if (( control_failures > 0 || process_errors > 0 )); then exit 1; fi
