#!/usr/bin/env bash
# Deploy low_level code to robot and build
set -e

if [ -z "$ROBOT" ]; then
    echo "🔍 Searching for robot..."
    for ip in "192.168.12.2" "100.82.165.36" "unitree-r1"; do
        if ping -c 1 -W 1 "$ip" &> /dev/null; then
            ROBOT="unitree@$ip"
            echo "✅ Found robot at: $ip"
            break
        fi
    done
fi

if [ -z "$ROBOT" ]; then
    echo "❌ Error: Could not reach the robot."
    exit 1
fi

DEST="${DEST:-~/HB/low_level}"
cd "$(dirname "$0")/.."

echo ">>> Rsync to $ROBOT:$DEST ..."
ssh "$ROBOT" "mkdir -p $DEST"

rsync -avz --progress \
    --exclude 'build/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    ./ "$ROBOT:$DEST/"

if [[ "$1" != "--no-build" ]]; then
    echo ">>> Building on robot ..."
    ssh "$ROBOT" "cd $DEST && mkdir -p build && cd build && cmake .. && make -j4"
fi

echo ""
echo "✓ Done. To run on robot:"
echo "    ssh -Y $ROBOT"
echo "    cd $DEST"
echo "    ./run_scripts/start_grouped_gui.sh auto"
