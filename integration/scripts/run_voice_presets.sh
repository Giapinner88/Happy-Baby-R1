#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HB_ROOT="${HB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
PYTHON="$HB_ROOT/voice/.venv/bin/python"

if [[ -r /etc/hb/stack.env ]]; then
    set -a
    # shellcheck disable=SC1091
    source /etc/hb/stack.env
    set +a
fi

export HB_ROOT
export PYTHONPATH="$HB_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
cd "$HB_ROOT/voice_presets"
exec "$PYTHON" -m voice_presets
