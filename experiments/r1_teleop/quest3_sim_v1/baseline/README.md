# Baseline — Quest 3 → upstream R1-A5 IK

Đây là mốc gốc duy nhất của workspace. Pipeline giữ nguyên bộ giải vendor
`third_party/xr_teleoperate_v1_6` để các implementation do dự án phát triển về
sau có một đối chứng ổn định và không bị “cải thiện” ngầm.

## Contract

- Quest pose được neo theo head pose đầu phiên.
- `R1_A5_ArmIK` upstream là solver duy nhất; project code không sửa nghiệm.
- Sim cố định root/chân và không có DDS output.
- Hardware giữ đúng một owner `rt/lowcmd` và chỉ chạy theo safety gate.
- Mỗi run canonical phải có data, metrics, figures, video và manifest; thiếu
  bất kỳ nhóm nào thì run bị đánh dấu incomplete và trả exit code khác 0.

Chạy mô phỏng:

```bash
make teleop HOST_IP=<workstation-ip>
```

Cấu hình editable duy nhất là [`config/upstream_stream.json`](config/upstream_stream.json).
Run mới nằm trong `runs/baseline_upstream_<UTC>/`. Các thư mục `t007_*` trong
`runs/` là bằng chứng bất biến tạo trước khi đổi tên; không sửa ID hoặc metadata
của chúng.

Các báo cáo T007 trước đây nằm trong [`history/`](history/). Chúng giải thích
quá trình chọn baseline nhưng không định nghĩa behavior active.

## Phát triển source của dự án

Không chỉnh baseline để thử solver mới. Tạo implementation dưới `teleop/r1/`,
cấu hình riêng và so sánh trên cùng input/evidence contract. Chỉ thay baseline
khi researcher quyết định một baseline revision mới và ghi rõ compatibility.
