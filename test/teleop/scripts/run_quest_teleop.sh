#!/usr/bin/env bash
# Chuỗi Quest 3 -> IK tay/đầu -> sidecar trên robot -> run_r1 (chủ rt/lowcmd duy nhất).
#
# Script này dựng môi trường, đồng bộ teleop lên robot rồi gọi
# scripts/teleop/run_r1_quest3_hardware.sh. Nó không tự arm, không restart service
# và không ghi motor khi đồng bộ.
#
# Vì sao cần lớp bọc: các script upstream tính ROOT = parents[2] của chính nó, tức là
# thư mục teleop/ này sau khi đã bố cục lại. Chúng import `teleop.r1` bằng tên package,
# nên PYTHONPATH phải trỏ vào teleop/src (nơi package thật sự nằm).
set -euo pipefail

TELEOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$TELEOP_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

# Load the single operator-editable YAML file from HB/teleop/config.
# shellcheck disable=SC1091
source "$TELEOP_DIR/scripts/load_network_config.sh"
hb_load_network_config

# A previous Ctrl+C can leave the HTTPS bridge or one pipeline stage alive.
# Clean only processes identified as this HB teleop pipeline before preflight.
"$TELEOP_DIR/scripts/cleanup_teleop.sh" "${HB_TELEOP_VUER_PORT:-8012}"

# Địa chỉ robot: dùng chung cơ chế dò 3 tầng với controller (ROBOT env -> robot.env -> ping).
if [[ -z "${ROBOT:-}" ]]; then
    # shellcheck disable=SC1091
    source "$TELEOP_DIR/scripts/_find_robot.sh"
    find_robot
fi
export ROBOT

# Luôn đồng bộ + xác minh sidecar trước khi mở phiên, tránh chạy bundle cũ trên robot.
# Đặt HB_TELEOP_AUTO_DEPLOY=0 nếu muốn bỏ qua bước này sau khi đã deploy thủ công.
if [[ "${HB_TELEOP_AUTO_DEPLOY:-1}" == "1" ]]; then
    echo "[run_quest_teleop] Đồng bộ và xác minh teleop trên robot..."
    "$TELEOP_DIR/scripts/deploy_teleop.sh" deploy
fi

echo "[run_quest_teleop] TELEOP_DIR=$TELEOP_DIR"
echo "[run_quest_teleop] ROBOT=$ROBOT"
echo "[run_quest_teleop] HOST_IP=${HB_TELEOP_HOST_IP:-auto}"
echo "[run_quest_teleop] PYTHONPATH=$PYTHONPATH"
echo "[run_quest_teleop] ĐỌC TRƯỚC: docs/operations/r1_quest3_teleop_hardware.md + docs/safety/"

exec "$TELEOP_DIR/scripts/teleop/run_r1_quest3_hardware.sh" "$@"
