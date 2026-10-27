#!/usr/bin/env bash
# Source-only helper for the small YAML subset used by controller config.
# It intentionally matches Tuning::LoadFromFileImpl: comments are stripped,
# double quotes are removed, and include files are processed in line order.

declare -A HB_CONFIG_READER_ACTIVE=()
HB_CONFIG_READER_KEY=""
HB_CONFIG_READER_VALUE=""

hb_config_reader_trim() {
    local value="$1"
    while [[ -n "$value" && "${value:0:1}" =~ [[:space:]] ]]; do
        value="${value:1}"
    done
    while [[ -n "$value" && "${value: -1}" =~ [[:space:]] ]]; do
        value="${value:0:${#value}-1}"
    done
    printf '%s' "$value"
}

hb_config_reader_unquote() {
    local value="$1"
    if [[ ${#value} -ge 2 && "${value:0:1}" == '"' && "${value: -1}" == '"' ]]; then
        value="${value:1:${#value}-2}"
    fi
    printf '%s' "$value"
}

hb_config_reader_walk() {
    local input_path="$1"
    local canonical_path line key value include_path

    [[ -r "$input_path" ]] || {
        echo "Config file is not readable: $input_path" >&2
        return 1
    }
    canonical_path="$(readlink -f -- "$input_path")" || {
        echo "Could not resolve config path: $input_path" >&2
        return 1
    }
    if [[ -n "${HB_CONFIG_READER_ACTIVE[$canonical_path]+present}" ]]; then
        echo "Config include cycle detected at: $canonical_path" >&2
        return 1
    fi
    HB_CONFIG_READER_ACTIVE["$canonical_path"]=1

    while IFS= read -r line || [[ -n "$line" ]]; do
        line="${line%%#*}"
        line="$(hb_config_reader_trim "$line")"
        [[ -n "$line" && "$line" == *:* ]] || continue

        key="$(hb_config_reader_trim "${line%%:*}")"
        value="$(hb_config_reader_trim "${line#*:}")"
        value="$(hb_config_reader_unquote "$value")"
        if [[ "$key" == "include" ]]; then
            [[ -n "$value" ]] || {
                echo "Empty include in config: $canonical_path" >&2
                unset 'HB_CONFIG_READER_ACTIVE[$canonical_path]'
                return 1
            }
            include_path="$value"
            [[ "$include_path" = /* ]] || include_path="$(dirname "$canonical_path")/$include_path"
            hb_config_reader_walk "$include_path" || {
                unset 'HB_CONFIG_READER_ACTIVE[$canonical_path]'
                return 1
            }
        elif [[ "$key" == "$HB_CONFIG_READER_KEY" ]]; then
            HB_CONFIG_READER_VALUE="$value"
        fi
    done <"$canonical_path"

    unset 'HB_CONFIG_READER_ACTIVE[$canonical_path]'
}

# Print the effective value of KEY after recursively resolving include files.
# Later entries override earlier entries, exactly as the C++ tuning loader does.
hb_config_get() {
    [[ $# -eq 2 ]] || {
        echo "Usage: hb_config_get <entry-config.yaml> <key>" >&2
        return 2
    }
    HB_CONFIG_READER_KEY="$2"
    HB_CONFIG_READER_VALUE=""
    HB_CONFIG_READER_ACTIVE=()
    hb_config_reader_walk "$1" || return
    printf '%s\n' "$HB_CONFIG_READER_VALUE"
}

# Bundle selectors are relative component paths, e.g. u2a_seed42 or
# armfree/nomirror_d20. Keep in sync with PolicyBundle::ValidName.
hb_valid_policy_bundle_name() {
    [[ "$1" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*(/[A-Za-z0-9][A-Za-z0-9._-]*)*$ ]]
}

# Print "<rel-path> <contract>" of the locomotion policy selected by ROLE
# (walk_policy | teleop_policy) in ENTRY-CONFIG, relative to the controller dir.
# Mirrors Application::InitControllers: bundle route when walk_policy is set
# (teleop_policy empty = same bundle), otherwise legacy flat_model route.
hb_selected_policy() {
    [[ $# -eq 2 && ( "$2" == walk_policy || "$2" == teleop_policy ) ]] || {
        echo "Usage: hb_selected_policy <entry-config.yaml> <walk_policy|teleop_policy>" >&2
        return 2
    }
    local cfg="$1" role="$2" high_dir name contract model
    high_dir="$(cd "$(dirname "$cfg")/.." && pwd)" || return 1
    name="$(hb_config_get "$cfg" walk_policy)" || return 1
    if [[ -n "$name" ]]; then
        if [[ "$role" == teleop_policy ]]; then
            local teleop
            teleop="$(hb_config_get "$cfg" teleop_policy)" || return 1
            [[ -z "$teleop" ]] || name="$teleop"
        fi
        hb_valid_policy_bundle_name "$name" || {
            echo "$role must name a relative bundle or group/name: '$name'" >&2
            return 1
        }
        contract="$(hb_config_get "$high_dir/policies/locomotion/$name/policy.yaml" contract)" || return 1
        [[ -n "$contract" ]] || { echo "bundle '$name' has no contract" >&2; return 1; }
        printf '%s %s\n' "policies/locomotion/$name/policy.onnx" "$contract"
        return 0
    fi
    model="$(hb_config_get "$cfg" flat_model)" || return 1
    contract="$(hb_config_get "$cfg" flat_policy_contract)" || return 1
    [[ -n "$model" && "$model" != */* && "$model" != "." && "$model" != ".." ]] || {
        echo "flat_model must name a file, not a path: '$model'" >&2
        return 1
    }
    case "$contract" in
        legacy_83) printf '%s %s\n' "policies/flat/$model" "$contract" ;;
        flat_plus_h4_v1|flat_plus_h5_v1|flat_plus_gait_h4_v1)
            printf '%s %s\n' "policies/locomotion/_archive/flat_plus/$model" "$contract" ;;
        *) echo "unsupported flat_policy_contract '$contract'" >&2; return 1 ;;
    esac
}
