# Bộ cảnh cho D001

Mỗi file ở đây là một profile D001 hoàn chỉnh, chỉ khác nhau ở khối `scene`.
Truyền thẳng vào `--dataset-scene-config` để xem trước hoặc để thu.

```bash
# xem trước — dựng ĐÚNG cảnh sẽ được thu, không phải một bản gần giống
conda run --no-capture-output -n unitree_sim_env python \
  experiments/r1_dataset/quest3_sim_v1/tools/preview_scene.py \
  --dataset-scene-config experiments/r1_dataset/quest3_sim_v1/D001/config/scenes/simple_room.json \
  --device cuda:0

# thu bằng cảnh đó
make dataset-d001-live HOST_IP=192.168.1.106 \
  DATASET_ARGS="--gui --dataset-config experiments/r1_dataset/quest3_sim_v1/D001/config/scenes/simple_room.json"
```

## Các biến thể

| File | Khung cảnh | Vật trên bàn |
|---|---|---|
| `bare.json` | mặt đất phẳng | chai mustard, hộp đường |
| `simple_room.json` | phòng có tường và rèm | chai mustard, hộp đường |
| `warehouse.json` | nhà kho | chai mustard, hộp đường |
| `office.json` | văn phòng | chai mustard, hộp đường |
| `blocks.json` | mặt đất phẳng | ba khối đỏ / xanh lá / xanh dương |
| `kitchen_ycb.json` | mặt đất phẳng | bốn vật YCB có physics |
| `mug_and_bin.json` | mặt đất phẳng | cốc (tĩnh) + khay KLT (tĩnh) |

Khung cảnh làm camera đầu robot nhìn thấy bối cảnh có vân thay vì nền xám —
điều kiện cần cho học thị giác. Đổi lại nó tốn render: đo ngày 2026-09-06,
`simple_room` cho `achieved_control_hz` 6.4 so với vài chục ở cảnh trống trên
cùng máy. **Đo lại trước khi chốt dùng cái nào để thu.**

## Danh mục đã kiểm chứng

Mọi đường dẫn dưới đây đã được kiểm bằng một yêu cầu HEAD tới kho asset Isaac
5.1 và đều trả 200.

**Khung cảnh** — dùng trong `scene.environment.name`:

`simple_room` · `office` · `hospital` · `warehouse` · `warehouse_full` ·
`warehouse_shelves` · `warehouse_forklifts` · `grid`

`Modular_Warehouse` có trong danh mục của NVIDIA nhưng file gốc trả 404, nên
không được liệt kê.

**Vật YCB có physics** — `kind: "ycb_rigid"`, dùng trong `asset`:

`003_cracker_box` · `004_sugar_box` · `005_tomato_soup_can` · `006_mustard_bottle`

Đó là **toàn bộ** bốn vật có biến thể `Axis_Aligned_Physics` trong Isaac 5.1.

**Vật YCB chỉ có hình học** — `kind: "ycb_static"`:

`002_master_chef_can` · `007_tuna_fish_can` · `008_pudding_box` ·
`009_gelatin_box` · `010_potted_meat_can` · `011_banana` · `019_pitcher_base` ·
`021_bleach_cleanser` · `024_bowl` · `025_mug` · `035_power_drill` ·
`036_wood_block` · `037_scissors` · `040_large_marker` · `051_large_clamp` ·
`052_extra_large_clamp` · `061_foam_brick`

**Prop khác** — `kind: "usd_rigid"` hoặc `"usd_static"`, dùng trong `usd_relpath`:

| Đường dẫn | Rơi được? |
|---|---|
| `Props/Blocks/red_block.usd` (và `green_`, `blue_`, `yellow_`, `nvidia_cube`) | ✅ `usd_rigid` |
| `Props/Mugs/SM_Mug_A2.usd` (và `B1`, `C1`, `D1`) | ❌ chỉ `usd_static` |
| `Props/KLT_Bin/small_KLT.usd` | ❌ chỉ `usd_static` |
| `Props/Rubiks_Cube/`, `Props/Food/`, `Props/Beaker/`, `Props/Sektion_Cabinet/` | chưa thử |

**Cái bẫy phải biết:** `usd_rigid` chỉ chạy với asset mà prim gốc nhận
`RigidBodyAPI` gắn lúc spawn. Asset không nhận sẽ làm Isaac ném
`RuntimeError: Failed to find a rigid body when resolving '/World/DatasetObject_N'`
ngay ở `sim.reset()`. `Props/Blocks/*` nhận được; `Props/Mugs/*` và
`Props/KLT_Bin/*` thì không — đã thử cả ba. Gặp lỗi đó thì đổi sang `usd_static`.

## Ràng buộc vị trí

Tầm với tối đa của tay R1 là **0.383 m** tính từ vai; vai phải ở world
`(0.033, -0.086, 1.006)`. Chân robot chìa ra tới `x = 0.304` nên bàn không đặt
gần hơn `x = 0.35`. Dải x dùng được cho vật vì thế chỉ là **[0.31, 0.40]**.

Mặt bàn ở `z = 0.95` (ngang tầm vai) để thành phần thẳng đứng gần như bằng
không — nếu hạ bàn xuống 0.80 thì chênh cao 0.18 m ăn hết ngân sách tầm với và
không vật nào chạm tới được.

Đặt vật ở `z = 1.00` để nó rơi nhẹ xuống mặt bàn và nằm yên ở khoảng `z = 0.98`.
