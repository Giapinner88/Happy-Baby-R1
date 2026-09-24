#!/bin/bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INTERFACE=${1:-auto}

# Activate virtual environment if it exists (e.g. on PC1)
if [ -d "$DIR/../../../../.venv" ]; then
    source "$DIR/../../../../.venv/bin/activate"
elif [ -d "/home/khanh248/Documents/HB/Mujoco/.venv" ]; then
    source "/home/khanh248/Documents/HB/Mujoco/.venv/bin/activate"
fi

# Detect network interface if set to auto
if [ "$INTERFACE" = "auto" ]; then
    echo "🔍 Auto-detecting network interface..."
    # Find first active non-loopback IPv4 interface
    DETECTED_IFACE=$(ip -o addr show | awk '$3 == "inet" && $2 !~ /lo/ {print $2}' | head -n 1)
    if [ -n "$DETECTED_IFACE" ]; then
        INTERFACE=$DETECTED_IFACE
        echo "📡 Detected active network interface: $INTERFACE"
    else
        INTERFACE="lo"
        echo "⚠️ No active interface found. Falling back to loopback 'lo'."
    fi
fi

# Check for PySide6 or PyQt5
python3 -c "import PySide6" 2>/dev/null || python3 -c "import PyQt5" 2>/dev/null || {
    echo "ERROR: Neither PySide6 nor PyQt5 is installed."
    echo "Please install PyQt5: sudo apt install python3-pyqt5"
    exit 1
}

# Find DDS Bridge binary
BRIDGE_BIN=""
if [ -f "$DIR/build/r1_dds_bridge" ]; then
    # Standalone build (PC2 Jetson)
    BRIDGE_BIN="$DIR/build/r1_dds_bridge"
elif [ -f "$DIR/../../../../unitree_sdk2/build/bin/r1_dds_bridge" ]; then
    # Workspace build (PC1)
    BRIDGE_BIN="$DIR/../../../../unitree_sdk2/build/bin/r1_dds_bridge"
fi

if [ -z "$BRIDGE_BIN" ] || [ ! -f "$BRIDGE_BIN" ]; then
    echo "ERROR: DDS Bridge binary not found!"
    echo "Please build the bridge. On PC2, run:"
    echo "  cd $DIR && mkdir -p build && cd build && cmake .. && make -j4"
    exit 1
fi

echo "=========================================="
echo "Starting R1 Joint Tuner + DDS Bridge..."
echo "Interface: $INTERFACE"
echo "Bridge Bin: $BRIDGE_BIN"
echo "=========================================="
echo ""

# Start C++ DDS Bridge in background
echo "Starting C++ DDS Bridge..."
"$BRIDGE_BIN" "$INTERFACE" &
BRIDGE_PID=$!

# Wait a moment for bridge to initialize
sleep 1

# Check if bridge is running
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "ERROR: Failed to start DDS Bridge!"
    exit 1
fi

echo "DDS Bridge started (PID: $BRIDGE_PID)"
echo "Starting GUI..."
echo ""

# Function to clean up background processes on exit
cleanup() {
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "Stopping DDS Bridge (PID: $BRIDGE_PID)..."
        kill $BRIDGE_PID 2>/dev/null
    fi
}

# Trap INT/TERM/EXIT signals to ensure cleanup is run
trap cleanup EXIT SIGINT SIGTERM

# Run the GUI
cd "$DIR"
python3 r1_professional_gui.py "$INTERFACE"
GUI_EXIT=$?

exit $GUI_EXIT
