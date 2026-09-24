# Test policy R1 trên nền 0°–40°, dốc trực tiếp và dốc có platform

Có mười một scene test độc lập:

- `../unitree_robots/r1/scene_slope_0.xml`
- `../unitree_robots/r1/scene_slope_5.xml`: mặt phẳng nghiêng trực tiếp 5°
- `../unitree_robots/r1/scene_slope_10.xml`: mặt phẳng nghiêng trực tiếp 10°
- `../unitree_robots/r1/scene_slope_15.xml`: mặt phẳng nghiêng trực tiếp 15°
- `../unitree_robots/r1/scene_slope_20.xml`: mặt phẳng nghiêng trực tiếp 20°
- `../unitree_robots/r1/scene_slope_25.xml`: mặt phẳng nghiêng trực tiếp 25°
- `../unitree_robots/r1/scene_slope_30.xml`: mặt phẳng nghiêng trực tiếp 30°
- `../unitree_robots/r1/scene_slope_35.xml`: mặt phẳng nghiêng trực tiếp 35°
- `../unitree_robots/r1/scene_slope_40.xml`: mặt phẳng nghiêng trực tiếp 40°
- `../unitree_robots/r1/scene_slope_15_platform.xml`: platform phẳng 2 m và hai ramp 15°
- `../unitree_robots/r1/scene_slope_30_platform.xml`: platform phẳng 2 m và hai ramp 30°

Scene phẳng mặc định `scene.xml` không bị thay đổi. Với mọi scene dốc, `+X` là
hướng lên dốc: phím `W` đi lên và `S` đi xuống. Các scene trực tiếp `5`–`40`
không có ô phẳng ở giữa; robot spawn trực tiếp trên mặt nghiêng. Hai scene `*-platform`
giữ vùng phẳng để kiểm tra đứng yên và cho robot tiếp cận dốc từ địa hình giống
phân bố reset lúc train.

## Cách chạy an toàn (khuyên dùng)

Một lệnh nạp policy xong trước, rồi mới mở simulator và tự truyền scene tuyệt
đối để ray-caster quét đúng scene:

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp
./run_rough_stack.sh default
./run_rough_stack.sh 15-platform
./run_rough_stack.sh 30-platform
```

Để spawn trực tiếp trên dốc, dùng:

```bash
./run_rough_stack.sh 5
./run_rough_stack.sh 10
./run_rough_stack.sh 15
./run_rough_stack.sh 20
./run_rough_stack.sh 25
./run_rough_stack.sh 30
./run_rough_stack.sh 35
./run_rough_stack.sh 40
```

Policy Rough 270-D hiện tại không quan sát base linear velocity. Vì vậy các
scene trực tiếp dùng để đánh giá hành vi trên dốc, nhưng không được hiểu là
policy chắc chắn giữ vận tốc bằng 0 trên một mặt nghiêng đồng nhất.

## Cách chạy thủ công hai terminal

Terminal 1, chạy policy trước và đợi `POLICY ĐÃ NẠP XONG`:

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp/build
./run_policy
```

Terminal 2, sau đó mới khởi động MuJoCo:

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp
./run_slope_sim.sh 0
./run_slope_sim.sh 15-platform   # hoặc 30-platform
./run_slope_sim.sh 5             # dốc trực tiếp: 5, 10, ..., 35 hoặc 40
```

Trong cửa sổ `R1 KEYBOARD CONTROL`, phím `1` chọn Locomotion, `W` đi lên dốc và
`S` đi xuống dốc. Để đổi model, chỉ sửa `locomotion_policy` trong
`config/tuning.yaml` rồi khởi động lại `run_policy`.

## Ghi chú đánh giá

- Độ tăng cao `tan(góc)` lần lượt là `0.087489`, `0.176327`, `0.267949`,
  `0.363970`, `0.466308`, `0.577350`, `0.700208`, `0.839100 m/m` cho các góc
  5°, 10°, 15°, 20°, 25°, 30°, 35°, 40°.
- Cả mười một scene test dùng contact friction `1.0 0.005 0.0001`. Mặt dốc trực
  tiếp, platform và ramp đều có `priority=1`, nên hệ số trượt hiệu dụng với bàn
  chân là `1.0`.
- Contract test xác nhận tám scene trực tiếp chỉ có một plane nghiêng, còn hai
  scene platform có vùng phẳng đúng 2 m và ramp nối liên tục tại `x=±1 m`.
- Kết quả MuJoCo chỉ đánh giá policy trong mô hình mô phỏng; không phải xác nhận
  an toàn để chạy robot thật ở cùng góc dốc.
