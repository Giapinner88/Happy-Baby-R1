#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
VOICE_DIR="$HB_ROOT/voice"
PYTHON="$VOICE_DIR/.venv/bin/python"

if [[ -r /etc/hb/stack.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source /etc/hb/stack.env
    set +a
fi
export HB_ROOT
# Load the optional search tool from the same canonical HB deploy tree.
OBJECT_SEARCH_OVERLAY="${HB_OBJECT_SEARCH_ROOT:-$HB_ROOT/object_search}/experiments/r1_object_grounding/p5_voice_search/voice_overlay"
if [[ -f "$OBJECT_SEARCH_OVERLAY/sitecustomize.py" && -f /etc/hb/object_search_enabled ]]; then
    export R1_P5_VOICE_ENABLED=1 R1_P5_PORT=5591
    export PYTHONPATH="$OBJECT_SEARCH_OVERLAY:$VOICE_DIR${PYTHONPATH:+:$PYTHONPATH}"
fi
cd "$VOICE_DIR"
exec "$PYTHON" -m hb_voice
