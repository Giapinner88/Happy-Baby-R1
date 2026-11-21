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
ASSISTANT_DIR="${HB_ASSISTANT_ROOT:-$HB_ROOT/assistant}"
if [[ -f "$ASSISTANT_DIR/overlay/sitecustomize.py" && -f /etc/hb/assistant_enabled ]]; then
    # Unified voice tasks (find/face/approach, consent gate) served by hb_assistant; replaces the
    # object_search overlay and hb_local_approach tools. See integration/scripts/install_assistant.sh.
    export R1_ASSISTANT_VOICE=1 R1_P5_VOICE_ENABLED=0 R1_ASSISTANT_PORT=5595
    export R1_ASSISTANT_CAMERA_LOCK=/run/hb-approach/camera.lock
    export PYTHONPATH="$ASSISTANT_DIR/overlay:$ASSISTANT_DIR/runtime:$VOICE_DIR${PYTHONPATH:+:$PYTHONPATH}"
elif [[ -f "$OBJECT_SEARCH_OVERLAY/sitecustomize.py" && -f /etc/hb/object_search_enabled ]]; then
    export R1_P5_VOICE_ENABLED=1 R1_P5_PORT=5591
    export PYTHONPATH="$OBJECT_SEARCH_OVERLAY:$VOICE_DIR${PYTHONPATH:+:$PYTHONPATH}"
fi
cd "$VOICE_DIR"
exec "$PYTHON" -m hb_voice
