#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
JOBS="${JOBS:-$(nproc)}"
MODE="${1:-all}"
source "$SCRIPT_DIR/status_log.sh"

case "$MODE" in
    all|high|voice|integration|presets) ;;
    *) echo "Usage: $0 [all|high|voice|integration|presets]" >&2; exit 2 ;;
esac
includes() { [[ "$MODE" == "all" || "$MODE" == "$1" ]]; }

ensure_onnxruntime_links() {
    local arch lib_dir runtime_lib soname
    arch="$(uname -m)"
    if [[ "$arch" == "aarch64" || "$arch" == "arm64" ]]; then
        lib_dir="$HB_ROOT/controller/thirdparty/onnxruntime_aarch64/lib"
    else
        lib_dir="$HB_ROOT/controller/thirdparty/onnxruntime/lib"
    fi

    runtime_lib="$(find "$lib_dir" -maxdepth 1 -type f \
        -name 'libonnxruntime.so.*' -print | sort -V | tail -n 1)"
    [[ -n "$runtime_lib" ]] || {
        echo "ONNX Runtime shared library not found in $lib_dir" >&2
        exit 1
    }
    soname="$(readelf -d "$runtime_lib" | sed -n \
        's/.*Library soname: \[\([^]]*\)\].*/\1/p' | head -n 1)"
    [[ "$soname" =~ ^libonnxruntime\.so\.[0-9]+(\.[0-9]+)*$ ]] || {
        echo "Unexpected ONNX Runtime SONAME '$soname' in $runtime_lib" >&2
        exit 1
    }

    if [[ "$(basename "$runtime_lib")" != "$soname" ]]; then
        ln -sfn "$(basename "$runtime_lib")" "$lib_dir/$soname"
    fi
    ln -sfn "$soname" "$lib_dir/libonnxruntime.so"
}

run_build_stage() {
    local label="$1" log rc
    shift
    log="$(mktemp "/tmp/hb-build-${label//[^a-zA-Z0-9]/_}.XXXXXX.log")"
    if "$@" >"$log" 2>&1; then
        rm -f -- "$log"
        hb_ok "$label build"
        return 0
    else
        rc=$?
        hb_fail "$label build (details: $log)"
        grep -Ei 'CMake Error|fatal error:|error:|undefined reference|FAILED:|Error [0-9]+|No space left' "$log" | tail -n 8 >&2 || true
        return "$rc"
    fi
}

# Build tăng dần theo NỘI DUNG, không chỉ theo mtime. Robot nhận source từ nhiều máy/cây;
# file mới có thể mang mtime cũ hơn object build từ cây khác và make bỏ qua nó
# (2026-11-06: Tuning.cpp mới, Tuning.cpp.o cũ -> run_r1 không biết khoá mới). Trước khi
# build: touch mọi source có sha256 khác bản ghi của lần build thành công trước (chưa có
# bản ghi -> touch hết, build lại toàn bộ một lần). Sau khi build thành công: ghi bản ghi mới.
# Các build_* chạy trong `if` của run_build_stage (set -e không áp dụng) nên mỗi bước
# phải tự `|| return`: lỗi build không được rơi xuống commit_source_record.
source_record() { echo "$1/.hb_source_sha256"; }

touch_changed_sources() {
    local src="$1" build="$2" record new
    record="$(source_record "$build")"
    new="$record.pending"
    mkdir -p "$build"
    (cd "$src" && find . \( -path ./build -o -path './build_*' -o -path ./thirdparty -o -path ./.venv \) -prune -o \
        -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.c' -o -name '*.hpp' -o -name '*.h' \
                  -o -name CMakeLists.txt -o -name '*.cmake' \) -print0 |
        sort -z | xargs -0 -r sha256sum) >"$new"
    if [[ -f "$record" ]]; then
        comm -13 <(sort "$record") <(sort "$new")
    else
        cat "$new"
    fi | cut -c67- | (cd "$src" && tr '\n' '\0' | xargs -0 -r touch --)
}

commit_source_record() {
    local record
    record="$(source_record "$1")"
    mv -f "$record.pending" "$record"
}

build_high() {
    ensure_onnxruntime_links || return
    touch_changed_sources "$HB_ROOT/controller" "$HB_ROOT/controller/build" || return
    cmake -S "$HB_ROOT/controller" -B "$HB_ROOT/controller/build" -DCMAKE_BUILD_TYPE=Release || return
    cmake --build "$HB_ROOT/controller/build" --target run_r1 --parallel "$JOBS" || return
    commit_source_record "$HB_ROOT/controller/build"
}

build_voice_bridge() {
    touch_changed_sources "$HB_ROOT/voice/unitree_bridge" "$HB_ROOT/voice/unitree_bridge/build" || return
    cmake -S "$HB_ROOT/voice/unitree_bridge" \
          -B "$HB_ROOT/voice/unitree_bridge/build" \
          -DCMAKE_BUILD_TYPE=Release -DUNITREE_SDK2_ROOT="$SDK_ROOT" || return
    cmake --build "$HB_ROOT/voice/unitree_bridge/build" --target r1_bridge --parallel "$JOBS" || return
    commit_source_record "$HB_ROOT/voice/unitree_bridge/build"
}

build_integration() {
    touch_changed_sources "$HB_ROOT/integration" "$HB_ROOT/integration/build" || return
    cmake -S "$HB_ROOT/integration" -B "$HB_ROOT/integration/build" -DCMAKE_BUILD_TYPE=Release || return
    cmake --build "$HB_ROOT/integration/build" --target hb_integration --parallel "$JOBS" || return
    commit_source_record "$HB_ROOT/integration/build"
}

if includes high; then
    run_build_stage "controller" build_high
fi

if includes voice || includes presets; then
    SDK_ROOT="${UNITREE_SDK2_ROOT:-}"
    if [[ -z "$SDK_ROOT" ]]; then
        for candidate in "$HOME/unitree_sdk2-main" "$HOME/unitree_sdk2"; do
            if [[ -f "$candidate/include/unitree/robot/r1/audio/audio_client.hpp" ]]; then
                SDK_ROOT="$candidate"
                break
            fi
        done
    fi
    [[ -n "$SDK_ROOT" ]] || { echo "Unitree SDK2 source tree for R1 audio not found" >&2; exit 1; }
    run_build_stage "voice_bridge" build_voice_bridge
fi

if includes integration || includes presets; then
    run_build_stage "integration" build_integration
fi

if includes voice || includes presets; then
    UV_BIN="${UV_BIN:-$HOME/.local/bin/uv}"
    [[ -x "$UV_BIN" ]] || { echo "uv not found: $UV_BIN" >&2; exit 1; }
    run_build_stage "voice_environment" "$UV_BIN" sync --frozen --project "$HB_ROOT/voice"
fi

ARTIFACTS=()
if includes high; then
    ARTIFACTS+=("$HB_ROOT/controller/build/run_r1")
fi
if includes voice || includes presets; then
    ARTIFACTS+=("$HB_ROOT/voice/unitree_bridge/build/r1_bridge")
fi
if includes integration || includes presets; then
    ARTIFACTS+=("$HB_ROOT/integration/build/hb_integration")
fi
for artifact in "${ARTIFACTS[@]}"; do
    [[ -x "$artifact" ]] || { hb_fail "build artifact missing: $artifact"; exit 1; }
done
