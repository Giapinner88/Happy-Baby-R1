# Locomotion policies

Mỗi policy đi bộ chọn được là **một thư mục**:

```text
policies/locomotion/<tên>/
  policy.onnx     model (metadata ONNX phải có policy_contract, observation_dim, action_dim, gains)
  policy.yaml     contract, sha256
```

Chọn trong `controller/config/locomotion.yaml`:

```yaml
walk_policy: <tên>      # R2+A — đi bộ thường
teleop_policy: <tên>    # R2+X — policy thứ hai; bỏ trống = dùng walk_policy
```

Thêm policy mới: `./hb policy add <file.onnx> <tên> [--contract C]`, rồi đổi
tên trong `locomotion.yaml`, chạy `./hb build && ./hb preflight` (hoặc `./hb deploy policy`
từ máy dev). `./hb policy list` liệt kê bundle hiện có. Checklist đầy đủ: `docs/policies.md`.

`_archive/flat_plus/` giữ các model 335-D/332-D cũ để so sánh/rollback qua route
`flat_model` + `flat_policy_contract` (xem README trong đó); không chọn được bằng `walk_policy`.
