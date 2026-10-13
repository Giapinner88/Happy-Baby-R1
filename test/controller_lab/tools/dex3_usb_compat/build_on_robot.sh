#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
OUTPUT="$SCRIPT_DIR/libdex3_claim_interface1.so"

g++ -std=c++17 -O2 -fPIC -shared -Wall -Wextra -Werror \
  "$SCRIPT_DIR/dex3_claim_interface1.cpp" -ldl -o "$OUTPUT"

file "$OUTPUT"
sha256sum "$OUTPUT"
