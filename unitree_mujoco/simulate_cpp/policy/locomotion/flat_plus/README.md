# Flat-Plus deployment policies

Các ONNX deploy được lấy từ bundle training
`r1_flat_dev_20260907/`. Folder training gốc vẫn được giữ nguyên tại
`simulate_cpp/r1_flat_dev_20260907/` để truy vết checkpoint, manifest và
golden trace. Hai file H4/H5 bên dưới đã được đối chiếu SHA-256 với bundle mới.

| File | Contract | Input | Nguồn |
|---|---|---:|---|
| `policy_flat_plus_h4_v1.onnx` | `flat_plus_h4_v1` | 332-D | `H4_H5/DEPLOY/` arm `h4_scratch` |
| `policy_flat_plus_h5_v1.onnx` | `flat_plus_h5_v1` | 415-D | `H4_H5/arms/h5_candidate/` |
| `policy_flat_plus_gait_h4_v1.onnx` | `flat_plus_gait_h4_v1` | 335-D | `GAIT/gait/` |
| `policy_flat_plus_gait_walkquality_v1.onnx` | `flat_plus_gait_h4_v1` | 335-D | `results/gait_walkquality/policy.onnx` |
| `policy_flat_plus_gait_walkquality_v2.onnx` | `flat_plus_gait_h4_v1` | 335-D | HB `gait_ts005_2048_15k_seed42` export |
| `policy_flat_plus_gait_0917_v4.onnx` | `flat_plus_gait_h4_v1` | 335-D | HB 0917 v4 export |

Kiểm tra nhanh:

```bash
/tmp/simulate_cpp_framework_build/single_policy_smoke_test \
  policy/locomotion/flat_plus/policy_flat_plus_h4_v1.onnx
/tmp/simulate_cpp_framework_build/single_policy_smoke_test \
  policy/locomotion/flat_plus/policy_flat_plus_h5_v1.onnx
```

Không tự động đổi `config/tuning.yaml`; để chọn một policy, đổi riêng dòng
`locomotion_policy`:

```yaml
locomotion_policy_dir: ../policy/locomotion/
locomotion_policy: flat_plus/policy_flat_plus_h4_v1.onnx
# hoặc: flat_plus/policy_flat_plus_h5_v1.onnx
# hoặc gait walk-quality v1:
# locomotion_policy: flat_plus/policy_flat_plus_gait_walkquality_v1.onnx
# hoặc gait walk-quality v2:
# locomotion_policy: flat_plus/policy_flat_plus_gait_walkquality_v2.onnx
# hoặc gait 0917 v4:
# locomotion_policy: flat_plus/policy_flat_plus_gait_0917_v4.onnx
```

Đây là single-policy Flat-Plus, chưa phải A-RMA bundle: chưa có
`adapter.onnx`, `actor.onnx` và `params/deploy.yaml`.

## Các kết quả gait/LSTM mới

`GAIT/gait/policy_flat_plus_gait_h4_v1.onnx` đã được kiểm tra và đưa vào route deploy sau khi C++
được sửa để dùng đúng metadata FSM canonical và stability filter của training.
Bản copy provenance/candidate vẫn được giữ để đối chiếu.

- candidate/provenance gait: `../../candidates/flat_plus_20260907/gait/`
- gait walk-quality candidate: `../../results/gait_walkquality/`;
  run `gait_mlp_walkquality_2048_15k_seed42`, seed 42, 2048 envs,
  335-D input / 24-D action. Contract smoke PASS và flat closed-loop PASS
  (500 mẫu, tilt tương đối cực đại khoảng 1.86551 độ, không ngã).
- LSTM paper/capacity: `../../candidates/flat_plus_20260907/lstm/`; đây là
  ONNX stateful (`obs + h/c -> action + h/c`), `deployable=false`, không có
  route single-policy hiện tại.

Candidate walk-quality v1/v2 và 0917 v4 dùng cùng contract runtime `flat_plus_gait_h4_v1`,
nhưng giữ tên file riêng để không ghi đè gait cũ. `config/tuning.yaml` hiện
đang active v2; đổi riêng `locomotion_policy` để rollback v1 hoặc chọn profile khác.
LSTM vẫn giữ ở candidate vì chưa có route stateful.
