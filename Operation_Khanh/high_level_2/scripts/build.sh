#!/usr/bin/env bash
# Incremental build script for high_level_2. Pass --clean only when required.
set -euo pipefail
cd "$(dirname "$0")/.."
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
if [[ "${1:-}" == "--clean" ]]; then
    cmake --build build --target clean
fi
cmake --build build --target run_r1 --parallel "$(nproc)"

test -x build/run_r1
echo ""
echo "✓ Built successfully: $(pwd)/build/run_r1"
