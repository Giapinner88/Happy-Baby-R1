#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
JOBS="${JOBS:-$(nproc)}"

cmake -S "$HB_ROOT/high_level_2" -B "$HB_ROOT/high_level_2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HB_ROOT/high_level_2/build" --target run_r1 --parallel "$JOBS"

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
cmake -S "$HB_ROOT/voice_r1/unitree_bridge" \
      -B "$HB_ROOT/voice_r1/unitree_bridge/build" \
      -DCMAKE_BUILD_TYPE=Release -DUNITREE_SDK2_ROOT="$SDK_ROOT"
cmake --build "$HB_ROOT/voice_r1/unitree_bridge/build" --target r1_bridge --parallel "$JOBS"

cmake -S "$HB_ROOT/r1_integration" -B "$HB_ROOT/r1_integration/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$HB_ROOT/r1_integration/build" --target hb_integration --parallel "$JOBS"

UV_BIN="${UV_BIN:-$HOME/.local/bin/uv}"
[[ -x "$UV_BIN" ]] || { echo "uv not found: $UV_BIN" >&2; exit 1; }
"$UV_BIN" sync --frozen --project "$HB_ROOT/voice_r1"

file "$HB_ROOT/high_level_2/build/run_r1" \
     "$HB_ROOT/voice_r1/unitree_bridge/build/r1_bridge" \
     "$HB_ROOT/r1_integration/build/hb_integration"

