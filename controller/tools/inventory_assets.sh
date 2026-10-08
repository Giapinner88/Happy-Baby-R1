#!/usr/bin/env bash
# Read-only inventory of runtime packages and generated boundaries.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DEFAULT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd -P)"
ROOT="$DEFAULT_ROOT"
OUTPUT=""

usage() {
    cat <<'USAGE'
Usage: inventory_assets.sh [--root PATH] [--output PATH]

Emit a TSV inventory without modifying the inspected tree. Counts only files
immediately inside each reported directory; nested artifacts are boundaries,
not recursively merged into package counts.
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)
            [[ $# -ge 2 ]] || { echo "--root requires PATH" >&2; exit 2; }
            ROOT="$2"
            shift 2
            ;;
        --output)
            [[ $# -ge 2 ]] || { echo "--output requires PATH" >&2; exit 2; }
            OUTPUT="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -d "$ROOT" && -r "$ROOT" ]] || {
    echo "Inventory root is missing or unreadable: $ROOT" >&2
    exit 1
}
ROOT="$(cd "$ROOT" && pwd -P)"

if [[ -n "$OUTPUT" ]]; then
    output_parent="$(dirname "$OUTPUT")"
    [[ -d "$output_parent" && -w "$output_parent" ]] || {
        echo "Inventory output directory is missing or unwritable: $output_parent" >&2
        exit 1
    }
    exec >"$OUTPUT"
fi

printf 'kind\tpath\tstatus\tonnx\tnpz\taudio\tmanifest\tsha256\n'

relative_path() {
    local path="$1"
    if [[ "$path" == "$ROOT"/* ]]; then
        printf '%s' "${path#"$ROOT"/}"
    else
        printf '%s' "$path"
    fi
}

count_immediate_files() {
    local directory="$1"
    local category="$2"
    local count=0
    local name

    while IFS= read -r name; do
        case "$category:$name" in
            onnx:*.onnx) count=$((count + 1)) ;;
            npz:*.npz) count=$((count + 1)) ;;
            audio:*.wav|audio:*.mp3|audio:*.ogg|audio:*.flac|audio:*.loop) count=$((count + 1)) ;;
            manifest:MANIFEST.yaml|manifest:manifest.yaml|manifest:manifest.json|manifest:model_manifest.conf) count=$((count + 1)) ;;
            sha256:SHA256SUMS|sha256:*.sha256|sha256:*.sha256sum) count=$((count + 1)) ;;
        esac
    done < <(find "$directory" -mindepth 1 -maxdepth 1 -type f -printf '%f\n' | sort)

    printf '%s' "$count"
}

emit_directory() {
    local kind="$1"
    local directory="$2"
    local status="$3"
    [[ -d "$directory" && -r "$directory" ]] || return 0

    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$kind" "$(relative_path "$directory")" "$status" \
        "$(count_immediate_files "$directory" onnx)" \
        "$(count_immediate_files "$directory" npz)" \
        "$(count_immediate_files "$directory" audio)" \
        "$(count_immediate_files "$directory" manifest)" \
        "$(count_immediate_files "$directory" sha256)"
}

# Runtime policy packages. The dance row is deliberately one row per package;
# candidate/checkpoint directories stay visible instead of being silently
# merged. `backup` is archived content and is not a runtime package.
emit_directory policy "$ROOT/controller/policies/flat" runtime
emit_directory policy "$ROOT/controller/policies/locomotion/flat_plus" runtime
emit_directory policy "$ROOT/controller/policies/gestures" runtime
if [[ -d "$ROOT/controller/policies/dance" ]]; then
    while IFS= read -r directory; do
        [[ "$(basename "$directory")" == "backup" ]] && continue
        emit_directory policy "$directory" runtime
    done < <(find "$ROOT/controller/policies/dance" -mindepth 1 -maxdepth 1 -type d -print | sort)
fi

# Motion packages are also reported at their immediate package boundary.
if [[ -d "$ROOT/controller/motions" ]]; then
    while IFS= read -r directory; do
        emit_directory motion "$directory" runtime
    done < <(find "$ROOT/controller/motions" -mindepth 1 -maxdepth 1 -type d -print | sort)
fi

# Generated boundaries are never policy packages. Report them separately so a
# cleanup can be explicit and independently verified.
while IFS= read -r directory; do
    emit_directory generated "$directory" generated
done < <(
    find "$ROOT" -mindepth 2 -maxdepth 4 -type d \
        \( -name build -o -name 'build_*' -o -name __pycache__ -o -name .pytest_cache \) \
        -print | sort -u
)
