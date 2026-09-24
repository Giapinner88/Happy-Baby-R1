#!/bin/bash

# Check arguments
if [ -z "$1" ]; then
    echo "Usage: ./start_gui_bridge.sh <network_interface>"
    echo "Options for <network_interface>:"
    echo "  - <interface_name> (e.g. eno1, wlp111s0 - for real robot control)"
    echo "  - lo               (for local MuJoCo simulation)"
    echo "  - auto             (auto-detect first active network interface)"
    echo "Example:"
    echo "  ./start_gui_bridge.sh auto"
    echo "  ./start_gui_bridge.sh lo"
    exit 1
fi

INTERFACE=$1

if [ "$INTERFACE" = "auto" ]; then
    echo "🔍 Auto-detecting network interface..."
    # Find first active non-loopback IPv4 interface
    DETECTED_IFACE=$(ip -o addr show | awk '$3 == "inet" && $2 !~ /lo/ {print $2}' | head -n 1)
    if [ -n "$DETECTED_IFACE" ]; then
        INTERFACE=$DETECTED_IFACE
        echo "📡 Detected active network interface: $INTERFACE"
    else
        INTERFACE="lo"
        echo "⚠️ No active ethernet/wifi interface found. Falling back to loopback 'lo' for local simulation."
    fi
fi


# Resolve script directory and workspace root (4 levels up)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

# Build paths relative to workspace root
BRIDGE_BIN="$WORKSPACE_ROOT/unitree_sdk2/build/bin/r1_dds_bridge"
GUI_PY="$SCRIPT_DIR/r1_joint_gui_bridge.py"
PYTHON_ENV="$WORKSPACE_ROOT/.venv/bin/python"

# Verify files exist
if [ ! -f "$BRIDGE_BIN" ]; then
    echo "❌ Error: C++ Bridge binary not found at $BRIDGE_BIN. Please build it first."
    exit 1
fi

if [ ! -f "$GUI_PY" ]; then
    echo "❌ Error: Python GUI script not found at $GUI_PY."
    exit 1
fi

# Function to clean up background processes on exit
cleanup() {
    echo ""
    echo "🛑 Stopping GUI and C++ DDS Bridge..."
    if [ ! -z "$BRIDGE_PID" ]; then
        kill $BRIDGE_PID 2>/dev/null
    fi
    exit 0
}

# Trap INT/TERM signals to ensure cleanup is run
trap cleanup SIGINT SIGTERM EXIT

echo "🚀 Starting C++ DDS Bridge on interface: $INTERFACE..."
$BRIDGE_BIN $INTERFACE &
BRIDGE_PID=$!

# Give the C++ bridge a moment to initialize DDS
sleep 1.5

# Check if bridge is still running
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "❌ Error: C++ DDS Bridge failed to start or crashed."
    exit 1
fi

echo "🎨 Starting Pygame GUI..."
$PYTHON_ENV $GUI_PY $INTERFACE
