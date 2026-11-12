# multigait_v2_run25_iter8400 — VẬT THỬ TÍCH HỢP, KHÔNG PHẢI ỨNG VIÊN DEPLOY

**Chọn bundle này sẽ làm `run_r1` dừng khởi động, có chủ ý.** Nó tồn tại để
đường nạp của contract `r1_armfree0917_multigait_h4_v2` được kiểm bằng một
artifact thật, không phải để chạy.

## Artifact tự khai không đủ tư cách

```
deployable   False
iteration    8400        (kế hoạch 20000)
revision     armfree0917_multigait_run25_v2_2048_v1
```

Nguồn: `logs/armfree0917_multigait_run25_v2_2048/seed42_2048env_20k/policy.onnx`
trên máy train, lấy ngày 2026-11-06 khi run đang **tạm dừng** ở iteration 8400.
Checkpoint khớp là `model_8400.pt`. Chưa đánh giá hành vi đi bộ, và không nên
đánh giá: đây là bản giữa chừng, stage 1 của curriculum, `RUN` chưa vào.

## Vì sao chưa chạy được

Contract này không phải biến thể của `flat_plus_gait_h4_v1`:

| | T1 (đang chạy) | V2 |
|---|---|---|
| obs | 335 = 332 chuẩn hoá + 3 one-hot | 355 = 332 chuẩn hoá + 23 conditioning thô |
| action | 24 khớp | **25** = 24 khớp + 1 lệnh nhịp (index 24) |
| FSM | 3 mode | 8 mode + substate BRAKE/GATHER/SETTLE |
| chu kỳ gait | cố định, ghi trong bundle | **do policy phát mỗi bước** |

Bốn quyết định còn bỏ ngỏ nằm trong `plans/PHASE_E_V2_CONTRACT_NOTES.md`, và
`run_r1` in đúng bốn điều đó khi bị chọn.

## Nếu bạn định bật nó lên

Đọc cảnh báo đầu `controller/src/policy/MultiGaitH4V2Controller.hpp` trước.
`ActionSize()` trả 25 trong khi `spec::kNumJoints` là 24; điều đó chỉ an toàn
vì `ValidateExtraMetadata()` luôn ném. Bỏ lệnh ném mà không tách hai chỗ đọc
`action_scale` và ghi `last_action_` ra trước là tràn bộ đệm.
