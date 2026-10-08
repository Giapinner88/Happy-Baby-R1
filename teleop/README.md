# teleop — Quest 3 → tay + đầu R1 (chạy trên robot)

Vận hành và giới hạn: [../docs/teleop.md](../docs/teleop.md). **Chỉ dùng khi robot treo trên giá, có người giữ E-stop.**

```text
config/teleop.yaml          cấu hình duy nhất (robot_runtime.*); đổi xong: make teleop-runtime
config/*.json               hợp đồng vendor R1-A5 (hash solver, URDF, bộ lọc) — test kiểm tra
scripts/run_robot_teleop.sh pipeline của service hb_teleop_runtime
scripts/teleop/             quest_bridge.py -> run_r1_vendor_ik_stream.py -> run_r1_vendor_targets.py
scripts/*.sh, *.py          bootstrap env, cài service, cert, preflight runtime, đọc YAML
src/teleop/r1/              bridge Quest, mapping, schema, rate limit (không DDS)
src/teleop/hardware/        high_level_sidecar (UDP 127.0.0.1:5560 -> run_r1), run_teleop (đọc rt/lowstate)
src/assets/R1.urdf          mô hình cho preflight runtime
systemd/                    hb_teleop_cert, hb_teleop_runtime
tests/                      sidecar, hợp đồng vendor, an toàn tĩnh, receiver của controller
third_party/                xr_teleoperate (televuer) và R1-A5 IK pin 845b25b — không sửa
```

Sidecar không tạo DDS publisher; `controller` (`run_r1`) là nơi duy nhất ghi `rt/lowcmd`.
Đây là đường teleop **chạy trên robot** với hợp đồng H4 mà `controller` yêu cầu. Đường máy trạm, mô phỏng
và thí nghiệm ở nhánh `teleop`; sidecar/bridge của nhánh đó chưa hỗ trợ H4 nên **không** được chép đè vào đây.
`tests/test_static_safety.py` kiểm tra mọi tham số mà `run_robot_teleop.sh` truyền đều được từng tầng nhận.

```bash
make test-python      # gồm tests/ của teleop
make teleop-check     # trên robot: đọc rt/lowstate, không ghi gì
```
