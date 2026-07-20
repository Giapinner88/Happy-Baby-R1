#!/usr/bin/env bash
# Build tại chỗ (PC hoặc robot — CMake tự chọn onnxruntime theo kiến trúc CPU)
set -e
cd "$(dirname "$0")/.."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
echo ""
echo "✓ Build xong: $(pwd)/run_r1"
