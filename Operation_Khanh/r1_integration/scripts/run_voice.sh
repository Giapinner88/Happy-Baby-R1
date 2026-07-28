#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
VOICE_DIR="$HB_ROOT/voice_r1"
PYTHON="$VOICE_DIR/.venv/bin/python"

if [[ -r /etc/hb/stack.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source /etc/hb/stack.env
    set +a
fi
export HB_ROOT
cd "$VOICE_DIR"
exec "$PYTHON" -m hb_voice
