#!/usr/bin/env bash
set -euo pipefail

# One-shot operator test: read one R1 camera JPEG, query Realtime, preserve evidence.
# Run this script on the robot. It does not advertise tools or send motor commands.

if [[ "$EUID" -ne 0 ]]; then
    echo "Run on the robot with: sudo tools/run_realtime_vision_test.sh [output-dir]" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VOICE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${1:-/tmp/hb_realtime_vision_$RUN_ID}"
CAPTURE_DIR="$(mktemp -d /tmp/hb-realtime-vision-capture.XXXXXX)"
CAMERA_PROBE="/home/unitree/Foundation_Unitree/tools/r1_video_rpc_probe.py"

[[ -f "$CAMERA_PROBE" ]] || { echo "Camera probe not found: $CAMERA_PROBE" >&2; exit 2; }
[[ -x "$VOICE_DIR/.venv/bin/python" ]] || { echo "Voice Python not found" >&2; exit 2; }
[[ -r /etc/hb/stack.env ]] || { echo "Cannot read /etc/hb/stack.env" >&2; exit 2; }
[[ ! -e "$OUTPUT_DIR" ]] || { echo "Output already exists: $OUTPUT_DIR" >&2; exit 2; }

python3 "$CAMERA_PROBE" \
    --interface "${UNITREE_NETWORK_INTERFACE:-eth10}" \
    --timeout 0.5 \
    --requests 3 \
    --save-dir "$CAPTURE_DIR"

mapfile -t IMAGES < <(find "$CAPTURE_DIR" -maxdepth 1 -type f -name '*.jpg' -print)
[[ "${#IMAGES[@]}" -eq 1 ]] || {
    echo "Expected exactly one captured JPEG, found ${#IMAGES[@]}" >&2
    exit 2
}

set -a
# shellcheck disable=SC1091
source /etc/hb/stack.env
set +a

exec "$VOICE_DIR/.venv/bin/python" "$SCRIPT_DIR/test_realtime_vision.py" \
    --image "${IMAGES[0]}" \
    --model gpt-realtime-2.1 \
    --output-dir "$OUTPUT_DIR"
