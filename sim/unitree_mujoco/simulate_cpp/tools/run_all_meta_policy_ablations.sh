#!/usr/bin/env bash
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
output_root="${1:-${repo_dir}/results/meta_policy_paper_20260828/ablations/closed_loop}"
baseline="${repo_dir}/policy/locomotion/meta/meta_policy_v1"
variants_root="${repo_dir}/policy/locomotion/meta/meta_policy_ablations_20260828"

variants=(
  baseline
  target_one_layer
  raw_action
  hard_switch
  history_1
  history_10
  history_25
  no_neutral
  no_hysteresis
  no_dwell
)

mkdir -p -- "${output_root}"
output_root="$(cd -- "${output_root}" && pwd)"
printf 'variant,exit_code,summary\n' > "${output_root}/runs.csv"

failures=0
for variant in "${variants[@]}"; do
  package="${variants_root}/${variant}"
  if [[ "${variant}" == "baseline" ]]; then
    package="${baseline}"
  fi
  result_dir="${output_root}/${variant}"
  console_log="${output_root}/${variant}.console.log"
  echo "[META_POLICY_ABLATIONS] variant=${variant} package=${package}"
  set +e
  "${script_dir}/run_meta_policy_ablation_suite.sh" \
    "${variant}" "${package}" "${result_dir}" > "${console_log}" 2>&1
  status=$?
  set -e
  printf '%s,%s,%s\n' "${variant}" "${status}" "${result_dir}/summary.csv" \
    >> "${output_root}/runs.csv"
  if (( status != 0 )); then
    ((failures += 1))
  fi
done

echo "[META_POLICY_ABLATIONS] variants=${#variants[@]} nonzero_suites=${failures}"
# A nonzero suite means at least one control/process case did not pass. Keep all
# variants running so the paper receives a complete, directly comparable screen.
if (( failures > 0 )); then exit 1; fi
