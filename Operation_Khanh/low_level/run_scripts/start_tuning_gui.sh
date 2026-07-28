#!/bin/bash
# Start PD Tuning GUI
set -e
trap 'echo "\n🛑 Stopping..."; kill -9 $BRIDGE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTERFACE=${1:-eth10}
BRIDGE_BIN="$DIR/build/r1_dds_bridge"
GUI_PY="$DIR/python_gui/r1_tuning_gui.py"

if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ C++ Bridge not found. Please build first."
    exit 1
fi

export CYCLONEDDS_URI="file://$HOME/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml"

echo "📡 Starting C++ DDS Bridge..."
cd "$DIR"
"$BRIDGE_BIN" "$INTERFACE" &
BRIDGE_PID=$!

sleep 1.5

echo "🖥️ Starting Python GUI (Tuning)..."
cd "$DIR/python_gui"
python3 "$GUI_PY" "$INTERFACE"

echo "🛑 GUI closed."
