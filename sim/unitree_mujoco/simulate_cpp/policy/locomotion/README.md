# Kho policy locomotion R1

Kho này chứa các ONNX locomotion dành cho `simulate_cpp`. Các model
Dance/Mimic và model không có contract R1 24-action không được đưa vào đây.

## Chuyển policy

Chỉ sửa dòng `locomotion_policy` trong `config/tuning.yaml`, ví dụ:

```yaml
locomotion_policy_dir: ../policy/locomotion/
locomotion_policy: flat/policy_v9.onnx
```

Sau đó tắt và chạy lại `build/run_policy`; không cần build lại. Controller tự
chọn theo kích thước input ONNX:

- `83-D` → `FlatController`.
- `270-D` → `RoughController` với height scan.
- `332-D` → `FlatPlusController`, chỉ khi metadata khai đúng history H4.
- `335-D` → `FlatPlusGaitController` (H4 + one-hot `stand,walk,w2s`) sau khi
  metadata/FSM gate và golden-trace parity pass.
- `415-D` → `FlatPlusH5Controller` (history H5).
- thư mục có `params/deploy.yaml` trong `locomotion_policy` → `ArmaController`,
  adapter history + actor gắn với đúng Flat base view của bundle.
- `--arma-package DIR` vẫn là alias tương thích để chọn bundle trực tiếp.

Runtime tự chọn profile từ contract/thư mục policy: `Flat` dùng gait `0.6 s`,
`Rough` 270-D và các policy trong `slope/` dùng gait `0.6 s`. Mỗi profile có bộ
`speed_*` và `fast_speed_*` riêng ngay đầu `config/tuning.yaml`; `TAB` chọn trực
tiếp bộ thường/nhanh, không còn nhân chung cả ba trục.

Khi chọn model trong `rough/`, để `rough_scene_path: auto`. Controller đọc trực
tiếp `-s/--scene` từ tiến trình `unitree_mujoco` R1 đang chạy, nên ray-caster
luôn nạp cùng scene với simulator. Hãy chạy `run_policy` trước; controller nạp
ONNX xong rồi chờ simulator xuất hiện để robot nhận PD command ngay khi khởi
tạo. Nếu có nhiều simulator R1 dùng scene khác nhau, controller sẽ không chọn đại.

## Danh sách model

### `flat/` — nguồn `HB/high_level_2/policies/flat`

| Policy | Input | Controller |
|---|---:|---|
| `policy_0.onnx` | 83 | Flat |
| `policy_1.onnx` | 83 | Flat |
| `policy_2.onnx` | 83 | Flat |
| `policy_3.onnx` | 83 | Flat |
| `policy_4.onnx` | 83 | Flat |
| `policy_5.onnx` | 83 | Flat |
| `policy_6.onnx` | 83 | Flat |
| `policy_7.onnx` | 83 | Flat |
| `policy_8.onnx` | 83 | Flat |
| `policy_10_07.onnx` | 83 | Flat |
| `policy_11_07.onnx` | 83 | Flat |
| `policy_goc.onnx` | 83 | Flat |
| `policy_r1.onnx` | 83 | Flat |
| `policy_r1_1.onnx` | 83 | Flat |
| `policy_r1_flat.onnx` | 83 | Flat |
| `policy_r1_flat_2.onnx` | 83 | Flat |
| `policy_v2.onnx` | 83 | Flat |
| `policy_v3.onnx` | 83 | Flat |
| `policy_v4.onnx` | 83 | Flat |
| `policy_v5.onnx` | 83 | Flat |
| `policy_v6.onnx` | 83 | Flat |
| `policy_v8.onnx` | 83 | Flat |
| `policy_v9.onnx` | 83 | Flat |

### History/gait model contracts

Các model mới phải mang metadata đầy đủ, không được chọn chỉ bằng số chiều:

| Contract | Input | Layout |
|---|---:|---|
| `flat_plus_h4_v1` | 332 | 4 frame, term-major, oldest→newest |
| `flat_plus_gait_h4_v1` | 335 | H4 + `stand,walk,w2s` one-hot hiện tại |
| `flat_plus_h5_v1` | 415 | 5 frame, term-major, oldest→newest |

History được backfill bằng frame đầu sau reset. Gait dùng FSM canonical với
ngưỡng tịnh tiến/góc riêng và bộ lọc signed IMU/joint velocity; không dùng
contact/terrain truth. Parity với `gait_golden_trace.txt` đã được chứng minh.

Các artifact Flat-Plus hiện có được đặt tại `flat_plus/`:

| Policy | Input | Controller |
|---|---:|---|
| `flat_plus/policy_flat_plus_h4_v1.onnx` | 332 | `FlatPlusController` |
| `flat_plus/policy_flat_plus_h5_v1.onnx` | 415 | `FlatPlusH5Controller` |
| `flat_plus/policy_flat_plus_gait_h4_v1.onnx` | 335 | `FlatPlusGaitController` |

Bundle train mới `r1_flat_dev_20260907/` còn có gait-conditioned MLP và hai
LSTM ablation. Chúng được giữ ở `policy/candidates/flat_plus_20260907/`:

| Candidate | Input/I-O | Trạng thái |
|---|---|---|
| `gait/policy_flat_plus_gait_h4_v1.onnx` | 335 → 24, single I/O | đã promote vào `flat_plus/` |
| `lstm/policy_flat_plus_gait_lstm_paper_v1.onnx` | 86 + h/c → 24 + h/c | research, không deploy |
| `lstm/policy_flat_plus_gait_lstm_capacity_v1.onnx` | 86 + h/c → 24 + h/c | research, không deploy |

Gait đã qua contract gate nên được promote vào `policy/locomotion/flat_plus/`;
hai LSTM vẫn ở candidate vì CMake sẽ tự quét mọi ONNX dưới
`policy/locomotion/` như một single-policy route và không hỗ trợ stateful I/O.

### A-RMA bundle theo từng Flat policy

Mỗi policy Flat cải tiến có một bundle riêng, ví dụ:

```text
policy/locomotion/arma/flat_plus_h4_rma_v1/
policy/locomotion/arma/flat_plus_h5_rma_v1/
policy/locomotion/arma/flat_plus_gait_h4_rma_v1/
```

Mỗi bundle chứa:

```text
params/deploy.yaml
models/adapter.onnx
models/actor.onnx
```

`deploy.yaml` phải khai thêm `base_policy_contract`, `actor_view`,
`actor_base_dim`, `actor_history_steps`. Các actor view được hỗ trợ là:

| View | Base dim | Actor dim với latent 8 |
|---|---:|---:|
| `base83` | 83 | 91 |
| `flat_plus_h4` | 332 | 340 |
| `flat_plus_h5` | 415 | 423 |
| `flat_plus_gait_h4` | 335 | 343 |

Hai ONNX phải có metadata role/hash đối ứng; runtime từ chối bundle thiếu hoặc
sai provenance. Chọn bundle bằng `locomotion_policy` trong `tuning.yaml`:

```bash
./run_sim.sh single flat
```

`./run_sim.sh arma flat` vẫn chạy bundle mặc định cũ để tương thích.

Nếu chưa có artifact đã train, lệnh sẽ dừng ở preflight và không chạm route
legacy.

### `legacy/` — model Flat cũ riêng của `simulate_cpp`

| Policy | Input | Controller |
|---|---:|---|
| `policy_1.onnx` | 83 | Flat |
| `policy_r1.onnx` | 83 | Flat |
| `policy_r1_l1.onnx` | 83 | Flat |

Tên `legacy/policy_1.onnx` và `legacy/policy_r1.onnx` giống tên trong `flat/`
nhưng nội dung model khác nhau. `policy_r1_flat.onnx` cũ không chép lần hai vì
trùng SHA-256 với `flat/policy_r1_flat.onnx`.

### `rough/` — locomotion có height scan

| Policy | Input | Controller |
|---|---:|---|
| `policy_r1_rough.onnx` | 270 | Rough |

### `slope/` — chuyên gia dốc không dùng height scan

| Policy | Input | Controller | Hướng chính |
|---|---:|---|---|
| `policy_up_v1.onnx` | 83 | Flat | Lên dốc |
| `policy_up_v2.onnx` | 83 | Flat | Lên dốc |
| `policy_up_v4.onnx` | 83 | Flat | Lên dốc |
| `policy_up_v6.onnx` | 83 | Flat | Lên dốc |
| `policy_down_v1.onnx` | 83 | Flat | Xuống dốc |
| `policy_down_v2.onnx` | 83 | Flat | Xuống dốc |
| `policy_down_v4.onnx` | 83 | Flat | Xuống dốc |
| `policy_down_v6.onnx` | 83 | Flat | Xuống dốc |

Các model này dùng gait period `0.6 s` và vẫn giữ actor contract 83-D.

## Ranh giới kiểm tra

Việc ONNX load đúng và khớp contract chỉ xác nhận kết nối phần mềm. Khả năng
giữ thăng bằng phải được test riêng cho từng model, scene, tốc độ và hướng dốc;
kết quả MuJoCo không phải xác nhận an toàn trên robot thật.
