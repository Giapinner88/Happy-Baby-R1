#!/bin/bash
# Start script for R1 Professional Joint Tuner GUI + DDS Bridge

# Get the directory where this script is located
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Absolute path to project root (Mujoco directory)
PROJECT_ROOT="/home/khanh248/Documents/HB/Mujoco"

# Activate virtual environment
if [ -d "$PROJECT_ROOT/.venv" ]; then
    source "$PROJECT_ROOT/.venv/bin/activate"
fi

# Check for PySide6
if ! python -c "import PySide6" 2>/dev/null; then
    echo "Installing PySide6..."
    pip install PySide6 numpy
fi

# Parse interface argument
INTERFACE=${1:-auto}

echo "=========================================="
echo "Starting R1 Joint Tuner + DDS Bridge..."
echo "Interface: $INTERFACE"
echo "Project Root: $PROJECT_ROOT"
echo "=========================================="
echo ""

# Find C++ DDS Bridge binary
BRIDGE_BIN="$PROJECT_ROOT/unitree_sdk2/build/bin/r1_dds_bridge"
if [ ! -f "$BRIDGE_BIN" ]; then
    echo "ERROR: DDS Bridge binary not found at $BRIDGE_BIN"
    echo "Please build the bridge first:"
    echo "  cd $PROJECT_ROOT/unitree_sdk2/build"
    echo "  cmake .. && make"
    exit 1
fi

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

