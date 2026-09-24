#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
variant="${1:-}"
package="${2:-}"
output_dir="${3:-/tmp/meta_policy_ablation_${variant}_$(date +%Y%m%d_%H%M%S)}"
if [[ -z "${variant}" || ! -f "${package}/params/deploy.yaml" ]]; then
  echo "Usage: $0 VARIANT PACKAGE_DIR [OUTPUT_DIR]" >&2
  exit 2
fi

mkdir -p -- "${output_dir}"
output_dir="$(cd -- "${output_dir}" && pwd)"
summary="${output_dir}/summary.csv"
printf 'variant,scene,case,vx,vy,yaw,hold_s,control_result,process_result,exit_code,log,csv\n' > "${summary}"
control_failures=0
process_errors=0

run_case() {
  local scene="$1" name="$2" vx="$3" vy="$4" yaw="$5" hold="$6"
  local resolved_scene="${scene}"
  if [[ "${resolved_scene}" == "flat" ]]; then
    resolved_scene="default"
  fi
  local log="${output_dir}/${name}.log" trace="${output_dir}/${name}.csv"
  local result="FAIL" process_result="ERROR" status result_line
  echo "[META_POLICY_ABLATION] variant=${variant} case=${name} scene=${scene}"
  set +e
  "${script_dir}/run_rough_stack.sh" "${resolved_scene}" --meta-package "${package}" \
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
    ((control_failures += 1))
  fi
  if [[ -n "${result_line}" && ( ${status} -eq 0 || ${status} -eq 2 || ${status} -eq 3 ) ]]; then
    process_result="OK"
  else
    ((process_errors += 1))
  fi
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${variant}" "${scene}" "${name}" "${vx}" "${vy}" "${yaw}" "${hold}" \
    "${result}" "${process_result}" "${status}" "${log}" "${trace}" >> "${summary}"
  echo "[META_POLICY_ABLATION] control=${result} process=${process_result} exit=${status}"
}

# Compact transition-focused suite. Thresholds are identical to the 64-case
# matrix; this suite is an ablation screen, not a replacement for that matrix.
run_case flat flat_stand 0 0 0 8
run_case 15-platform platform_up 0.3 0 0 10
run_case 15-platform platform_down -0.3 0 0 10
run_case 15 direct_up 0.3 0 0 10
run_case mixed mixed_traverse 0.4 0 0 48

echo "[META_POLICY_ABLATION] variant=${variant} control_failures=${control_failures} process_errors=${process_errors} summary=${summary}"
if (( control_failures > 0 || process_errors > 0 )); then exit 1; fi
