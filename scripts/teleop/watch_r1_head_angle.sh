#!/usr/bin/env bash
# Đọc liên tục góc đầu robot để nắn về giữa bằng tay trước khi bóp cò.
#
# Sidecar từ chối khởi động phiên khi |yaw| > 0.60 hoặc |pitch| > 0.35 rad: với
# phiên tương đối, đầu lệch bao nhiêu lúc chốt thì lệch bấy nhiêu suốt phiên.
# Đầu đang ZERO TORQUE nên xoay tay được; script này chỉ ĐỌC rt/lowstate và
# không bao giờ tạo publisher.
#
#   ./scripts/teleop/watch_r1_head_angle.sh     # Ctrl+C để thoát
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=../../hardware/teleop/scripts/_find_robot.sh
source "$ROOT/hardware/teleop/scripts/_find_robot.sh"
find_robot || exit 2
ssh -o BatchMode=yes "$ROBOT" 'cd /tmp && PYTHONPATH=$HOME/HB/teleop/src python3 -u -c "
import json,math,time,sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
ChannelFactoryInitialize(0, \"eth10\")
s = ChannelSubscriber(\"rt/lowstate\", LowState_); s.Init()
while True:
    m = s.Read()
    if m is not None:
        p = float(m.motor_state[29].q); y = float(m.motor_state[30].q)
        ok = \"OK \" if abs(y) <= 0.60 and abs(p) <= 0.35 else \"CHUA\"
        print(f\"{ok}  yaw {math.degrees(y):+7.1f} deg   pitch {math.degrees(p):+6.1f} deg   (can |yaw|<34.4, |pitch|<20.1)\")
    time.sleep(0.2)
"'
