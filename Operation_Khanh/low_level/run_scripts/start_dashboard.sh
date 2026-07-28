#!/bin/bash
# Start read-only sensor dashboard
set -e

trap 'echo "\n🛑 Stopping..."; kill -9 $BRIDGE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INTERFACE=${1:-eth10}
BRIDGE_BIN="$DIR/build/r1_dds_bridge"
GUI_PY="$DIR/python_gui/r1_sensor_dashboard.py"

if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ C++ Bridge not found. Please build first."
    exit 1
fi

echo "========================================="
echo " STARTING SENSOR DASHBOARD (READ-ONLY)"
echo " DDS Interface : $INTERFACE"
echo "========================================="

export CYCLONEDDS_URI="file://$HOME/unitree_sdk2/thirdparty/cyclonedds/cyclonedds.xml"

echo "📡 Starting C++ DDS Bridge in monitor-only mode..."
cd "$DIR"
"$BRIDGE_BIN" "$INTERFACE" --monitor &
BRIDGE_PID=$!

sleep 1.5

echo "🖥️ Starting PySide6 Dashboard..."
python3 "$GUI_PY"

echo "🛑 GUI closed."
