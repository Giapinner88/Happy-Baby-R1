#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
meta_package="${script_dir}/policy/locomotion/meta/slope_meta_v6"
meta_policy_package="${script_dir}/policy/locomotion/meta/meta_policy_v1"
rma_meta_package="${script_dir}/policy/locomotion/meta/rma_meta_v1"
arma_package="${script_dir}/policy/locomotion/arma/flat_o83_k50_z8_v1"

usage() {
  cat <<'EOF'
Usage:
  ./run_sim.sh meta <scene> [extra run_policy options]
  ./run_sim.sh meta_policy <scene> [extra run_policy options]
  ./run_sim.sh rma_meta <scene> [extra run_policy options]
  ./run_sim.sh gait_v3b <scene> [extra run_policy options]
  ./run_sim.sh gait_v3f <scene> [extra run_policy options]
  ./run_sim.sh gait-preflight <policy.onnx>
  ./run_sim.sh arma <scene> [extra run_policy options]
  ./run_sim.sh expert <flat|slope_up|slope_down> <scene> [extra run_policy options]
  ./run_sim.sh rma_expert <flat|slope_up|slope_down> <scene> [extra run_policy options]
  ./run_sim.sh single <scene> [extra run_policy options]
  ./run_sim.sh sim <scene> [extra simulator options]
  ./run_sim.sh preflight
  ./run_sim.sh preflight-meta-policy
  ./run_sim.sh preflight-rma-meta
  ./run_sim.sh preflight-arma [bundle_dir]
  ./run_sim.sh meta_policy-smoke [output_dir]
  ./run_sim.sh meta_policy-full [output_dir]
  ./run_sim.sh rma_meta-smoke [output_dir]
  ./run_sim.sh rma_meta-full [output_dir]
  ./run_sim.sh build
  ./run_sim.sh list

Scenes:
  flat, 0, 5, 10, 15, 20, 25, 30, 35, 40,
  5-platform, 10-platform, 15-platform, 20-platform, 25-platform, 30-platform,
  mixed, default

Examples:
  ./run_sim.sh meta mixed
  ./run_sim.sh meta_policy mixed
  ./run_sim.sh rma_meta mixed
  ./run_sim.sh arma flat
  ./run_sim.sh expert flat flat
  ./run_sim.sh expert slope_up 15
  ./run_sim.sh expert slope_down 15
  ./run_sim.sh meta 30-platform
  ./run_sim.sh single 15
EOF
}

normalize_scene() {
  case "$1" in
    flat) printf '0\n' ;;
    *) printf '%s\n' "$1" ;;
  esac
}

configure_and_build() {
  if [[ ! -f "${script_dir}/build/CMakeCache.txt" ]]; then
    cmake -S "${script_dir}" -B "${script_dir}/build" -DCMAKE_BUILD_TYPE=Release
  fi
  cmake --build "${script_dir}/build" -j"${BUILD_JOBS:-2}"
}

command="${1:-}"
case "${command}" in
  list)
    echo "Modes: meta, meta_policy, rma_meta, arma, gait_v3b, gait_v3f, gait-preflight, expert, rma_expert, single, sim, meta_policy-smoke, meta_policy-full, rma_meta-smoke, rma_meta-full"
    echo "Experts: flat slope_up slope_down"
    echo "Scenes: flat 0 5 10 15 20 25 30 35 40 5-platform 10-platform 15-platform 20-platform 25-platform 30-platform mixed default"
    ;;
  build)
    configure_and_build
    ;;
  preflight)
    configure_and_build
    ctest --test-dir "${script_dir}/build" --output-on-failure
    (
      cd -- "${meta_package}"
      sha256sum --check SHA256SUMS
    )
    "${script_dir}/build/slope_meta_preflight" "${meta_package}" 200
    ;;
  preflight-meta-policy)
    configure_and_build
    ctest --test-dir "${script_dir}/build" --output-on-failure \
      -R 'meta_policy|slope_meta_selector|slope_meta_command_governor|expert_action_adapter'
    (
      cd -- "${meta_policy_package}"
      sha256sum --check SHA256SUMS
    )
    "${script_dir}/build/slope_meta_preflight" "${meta_policy_package}" 200
    ;;
  preflight-rma-meta)
    configure_and_build
    ctest --test-dir "${script_dir}/build" --output-on-failure \
      -R 'rma_meta|slope_meta_command_governor|expert_action_adapter'
    (
      cd -- "${rma_meta_package}"
      sha256sum --check SHA256SUMS
    )
    "${script_dir}/build/rma_meta_preflight" "${rma_meta_package}" 200
    ;;
  preflight-arma)
    configure_and_build
    selected_arma_package="${2:-${arma_package}}"
    if [[ ! -d "${selected_arma_package}" ]]; then
      echo "A-RMA package not found: ${selected_arma_package}" >&2
      echo "Export the trained bundle there or call arma_preflight directly." >&2
      exit 1
    fi
    exec "${script_dir}/build/arma_preflight" "${selected_arma_package}"
    ;;
  meta_policy-smoke|meta_policy-full)
    acceptance_mode="${command#meta_policy-}"
    exec "${script_dir}/tools/run_meta_policy_acceptance.sh" \
      "${acceptance_mode}" "${2:-}"
    ;;
  rma_meta-smoke|rma_meta-full)
    acceptance_mode="${command#rma_meta-}"
    exec "${script_dir}/tools/run_rma_meta_acceptance.sh" \
      "${acceptance_mode}" "${2:-}"
    ;;
  expert|rma_expert)
    expert_name="${2:-}"
    scene="$(normalize_scene "${3:-flat}")"
    shift $(( $# >= 3 ? 3 : $# ))
    expert_package="${meta_policy_package}"
    flat_expert_name="flat_common_pd.onnx"
    if [[ "${command}" == rma_expert ]]; then
      expert_package="${rma_meta_package}"
      flat_expert_name="flat.onnx"
    fi
    case "${expert_name}" in
      flat)
        expert_policy="${expert_package}/exported/${flat_expert_name}"
        ;;
      slope_up|up)
        expert_policy="${expert_package}/exported/slope_up.onnx"
        ;;
      slope_down|down)
        expert_policy="${expert_package}/exported/slope_down.onnx"
        ;;
      *)
        echo "Expert không hợp lệ: ${expert_name}" >&2
        echo "Chọn một trong: flat, slope_up, slope_down" >&2
        exit 2
        ;;
    esac
    configure_and_build
    exec "${script_dir}/run_rough_stack.sh" \
      "${scene}" --locomotion-policy "${expert_policy}" "$@"
    ;;
  meta|meta_policy|rma_meta|arma|single)
    scene="$(normalize_scene "${2:-mixed}")"
    shift $(( $# >= 2 ? 2 : $# ))
    configure_and_build
    if [[ "${command}" == meta ]]; then
      exec "${script_dir}/run_rough_stack.sh" \
        "${scene}" --meta-package "${meta_package}" "$@"
    fi
    if [[ "${command}" == meta_policy ]]; then
      exec "${script_dir}/run_rough_stack.sh" \
        "${scene}" --meta-package "${meta_policy_package}" "$@"
    fi
    if [[ "${command}" == rma_meta ]]; then
        exec "${script_dir}/run_rough_stack.sh" \
          "${scene}" --rma-meta-package "${rma_meta_package}" "$@"
    fi
    if [[ "${command}" == arma ]]; then
      exec "${script_dir}/run_rough_stack.sh" \
        "${scene}" --arma-package "${arma_package}" "$@"
    fi
    exec "${script_dir}/run_rough_stack.sh" "${scene}" "$@"
    ;;
  gait_v3b|gait_v3f)
    scene="$(normalize_scene "${2:-flat}")"
    shift $(( $# >= 2 ? 2 : $# ))
    tuning_file="${script_dir}/config/tuning_gait_v3b.example.yaml"
    if [[ "${command}" == gait_v3f ]]; then
      tuning_file="${script_dir}/config/tuning_gait_v3f.example.yaml"
    fi
    if [[ ! -f "${tuning_file}" ]]; then
      echo "Gait v3 tuning template not found: ${tuning_file}" >&2
      exit 1
    fi
    SIMULATE_CPP_TUNING_CONFIG="${tuning_file}" \
      exec "${script_dir}/run_rough_stack.sh" "${scene}" "$@"
    ;;
  gait-preflight)
    policy_path="${2:-}"
    if [[ -z "${policy_path}" || ! -f "${policy_path}" ]]; then
      echo "Usage: ./run_sim.sh gait-preflight /abs/path/policy.onnx" >&2
      exit 2
    fi
    configure_and_build
    exec "${script_dir}/build/single_policy_smoke_test" "${policy_path}"
    ;;
  sim)
    scene="$(normalize_scene "${2:-mixed}")"
    shift $(( $# >= 2 ? 2 : $# ))
    case "${scene}" in
      mixed)
        exec "${script_dir}/run_mixed_terrain_sim.sh" "$@"
        ;;
      default)
        exec "${script_dir}/../simulate/build/unitree_mujoco" \
          -r r1 -n lo -s "${script_dir}/../unitree_robots/r1/scene.xml" "$@"
        ;;
      *)
        exec "${script_dir}/run_slope_sim.sh" "${scene}" "$@"
        ;;
    esac
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
