# R1 mixed terrain course

Scene `../unitree_robots/r1/scene_mixed_terrain.xml` gom nhiều địa hình cố định
trên cùng một nền phẳng rộng. Scene này độc lập với `scene.xml` và các scene
`scene_slope_*.xml`; các baseline cũ không bị thay đổi.

## Bố cục

Robot spawn thẳng đứng tại `(x,y)=(0,0)`. Vùng trung tâm `4 x 4 m` hoàn toàn
phẳng để khởi động policy và đổi hướng trước khi vào từng lane.

```text
                                 +Y

                      stairs 5 cm     stairs 10 cm
                           ^                ^
                           |                |

 rough patch  <----  flat spawn 4 x 4 m  ---->  slope 5 deg   (y=+3)
   4 x 4 m                                     slope 15 deg  (y= 0)
                                               slope 30 deg  (y=-3)

                    six 5 cm box obstacles
                                 -Y
```

| Module | Vị trí chính | Kích thước/độ cao |
|---|---:|---|
| Spawn phẳng | tâm `(0,0)` | vùng trống `4 x 4 m` |
| Dốc 5° | bắt đầu `x=3`, `y=3` | 3 m lên, platform 1 m, 3 m xuống; đỉnh 0.261 m |
| Dốc 15° | bắt đầu `x=3`, `y=0` | 3 m lên, platform 1 m, 3 m xuống; đỉnh 0.776 m |
| Dốc 30° | bắt đầu `x=3`, `y=-3` | 3 m lên, platform 1 m, 3 m xuống; đỉnh 1.500 m |
| Cầu thang 5 cm | `x=-1`, đi theo `+Y` | 4 bậc lên, landing 1 m, 4 lần hạ; đỉnh 0.20 m |
| Cầu thang 10 cm | `x=1`, đi theo `+Y` | 4 bậc lên, landing 1 m, 4 lần hạ; đỉnh 0.40 m |
| Gồ ghề | `x=-7..-3`, `y=-2..2` | 64 tile, relief cố định 2-8 cm |
| Hộp thấp | phía `-Y` | 6 hộp footprint khác nhau, cùng cao đúng 5 cm |

Ba dốc là ba lane độc lập và đều trở về nền `z=0`. Robot không phải vượt dốc
khác trước khi vào lane cần test. Rough patch dùng pattern xác định, không dùng
random runtime nên các lần test có thể so sánh trực tiếp.

## Cách chạy hai terminal độc lập

Terminal 1 chỉ chạy policy và đợi log `POLICY ĐÃ NẠP XONG`:

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp/build
./run_policy
```

Terminal 2 chỉ mở mixed-terrain simulator, không khởi tạo hoặc dừng policy:

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp
./run_mixed_terrain_sim.sh
```

Hai tiến trình có vòng đời độc lập: đóng cửa sổ MuJoCo không dừng `run_policy`,
và `Ctrl+C` ở terminal policy không tự đóng simulator. Script môi trường nhận
thêm các tùy chọn của `unitree_mujoco` qua phần đối số phía sau nếu cần.

`rough_scene_path: auto` trong `config/tuning.yaml` sẽ phát hiện đúng đường dẫn
scene từ tiến trình simulator. Sau khi thay đổi scene XML phải đóng và mở lại
simulator; tiến trình đang chạy không tự reload hình học.

## Contact và height scan

Mọi geom địa hình dùng chung:

```xml
priority="1" condim="3" group="0" friction="1.0 0.005 0.0001"
```

Bàn chân R1 vẫn giữ `priority=1`, `condim=6`, friction trượt `0.9`. Contact đã
compile giữa chân và nền phải có `dim=6` và friction trượt hiệu dụng `1.0`.
`group=0` làm toàn bộ floor, ramp, stair, rough tile và hộp 5 cm xuất hiện trong
height scan 187 điểm của RoughController.

## Kiểm tra contract

```bash
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

Test `mixed_terrain_scene_contract` kiểm tra:

- contract robot `nq=31`, `nu=24` và keyframe spawn;
- góc dốc 5°/15°/30°, chiều dài mặt dốc và các điểm nối;
- bậc thang đúng 5 cm/10 cm, không có khe giữa tread;
- đủ 64 rough tile, relief 2-8 cm và không có lỗ giữa tile;
- đủ 6 hộp, mặt trên đúng `z=0.05 m`;
- raycast thấy đúng cao độ của từng loại địa hình;
- contact chân-nền hiệu dụng là `dim=6`, friction trượt `1.0`.

Đây là kiểm tra simulator và policy trong MuJoCo, không xác nhận an toàn để dựng
các địa hình cùng kích thước cho robot thật.
