#!/usr/bin/env bash
# Incremental build script for controller. Pass --clean only when required.
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
if [[ "${1:-}" == "--clean" ]]; then
    cmake --build build --target clean
fi
JOBS="${HB_BUILD_JOBS:-$(nproc)}"
cmake --build build --parallel "$JOBS"

test -x build/run_r1
echo ""
echo "✓ Built successfully: $(pwd)/build/run_r1"
