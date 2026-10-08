# controller — `run_r1`

Controller C++ chạy trên robot: nạp policy ONNX/NPZ (đi bộ, nhảy, gesture), quản lý trạng thái, an toàn, và là **tiến trình duy nhất** ghi `rt/lowcmd` (`src/robot/LowCmdSender.hpp`). Chạy bằng service `hb_high_level`.

Vận hành: [../docs/operation.md](../docs/operation.md) · nút/cổng: [../docs/controls.md](../docs/controls.md) · đổi policy: [../docs/policies.md](../docs/policies.md).

## Bố cục

```text
config/tuning.yaml      entry point, chỉ include các nhóm dưới
config/runtime.yaml     DDS interface, input, lọc sensor
config/safety.yaml      arm gate, fall guard, battery, gain an toàn
config/locomotion.yaml  vận tốc, chuyển trạng thái, contract + model đi bộ đang chọn
config/postures.yaml    ngồi ghế, nằm xuống, đứng dậy
config/gestures.yaml    slot gesture, teleop
config/dance.yaml       slot nhảy 2–8
config/audio.yaml       giọng nói, gain đầu
config/*.example.yaml   bộ giá trị đã kiểm cho từng model đi bộ
policies/               locomotion/flat_plus, flat (legacy), dance, gestures, loco-teleop
motions/                getup.npz, liedown.npz (ghi bằng tools/record_motion)
scripts/                build.sh, _find_robot.sh, công cụ metadata ONNX, teleop_send_test.py
src/                    app, config, estimation, gait, input, policy, motion, robot, safety, util
tests/                  CTest: hợp đồng, golden trace, preflight từng profile
tools/                  record_motion, make_gesture.py, inventory_assets.sh
thirdparty/             cnpy, onnxruntime (x86_64 + aarch64)
```

## Build và test

```bash
make build-controller          # từ gốc repo; chỉ target run_r1
make test-controller           # build mọi target + ctest
HB_PROJECT_DIR=$PWD/controller controller/build/run_r1 --preflight   # không DDS, không động cơ
```

`run_r1 [interface]` dùng card mạng trong `config/runtime.yaml` nếu không truyền tham số.
Config sai (khóa lạ, số sai định dạng, ngoài dải) làm `run_r1` từ chối khởi động thay vì bỏ qua.
