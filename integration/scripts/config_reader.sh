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
