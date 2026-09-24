#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-${script_dir}/results/meta_policy_paper_20260828/soak}"
runs="${2:-3}"

if ! [[ "${runs}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [OUTPUT_DIR] [POSITIVE_RUN_COUNT]" >&2
  exit 2
fi

mkdir -p -- "${output_dir}"
output_dir="$(cd -- "${output_dir}" && pwd)"
summary="${output_dir}/summary.csv"
printf 'run,scene,sequence,hold_s,control_result,process_result,exit_code,log,csv\n' \
  > "${summary}"
control_failures=0
process_errors=0

for ((run = 1; run <= runs; ++run)); do
  log="${output_dir}/mixed_shuttle_${run}.log"
  trace="${output_dir}/mixed_shuttle_${run}.csv"
  result="FAIL"
  process_result="ERROR"
  echo "[META_POLICY_SOAK] run=${run}/${runs} scene=mixed sequence=mixed_shuttle hold=108"
  set +e
  "${script_dir}/run_sim.sh" meta_policy mixed \
    --autotest locomotion gesture=0 warmup=2 hold=108 sequence=mixed_shuttle \
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
  printf '%s,mixed,mixed_shuttle,108,%s,%s,%s,%s,%s\n' \
    "${run}" "${result}" "${process_result}" "${status}" "${log}" "${trace}" \
    >> "${summary}"
  echo "[META_POLICY_SOAK] control=${result} process=${process_result} exit=${status}"
done

echo "[META_POLICY_SOAK] runs=${runs} control_failures=${control_failures} process_errors=${process_errors}"
if (( control_failures > 0 || process_errors > 0 )); then exit 1; fi
