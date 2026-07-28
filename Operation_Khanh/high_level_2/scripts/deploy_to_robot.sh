#!/usr/bin/env bash
# Compatibility wrapper: deployment is owned by r1_integration.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/../../r1_integration/scripts/deploy_stack.sh" deploy "$@"
