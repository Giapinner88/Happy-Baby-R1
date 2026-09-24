# RMA-meta v5 — bản tối giản

Chỉ 3 file cần thay. Ba expert ONNX (`slope_up`, `slope_down`, `flat`) **giữ
nguyên** — chúng đóng băng, hash giống hệt bản v1 bạn đang có.

## 1. Hai ONNX — thay cùng lúc

    rma_meta_adapter.onnx    -> exported/rma_meta_adapter.onnx
    rma_meta_selector.onnx   -> exported/rma_meta_selector.onnx

Bắt buộc thay cả hai: runner kiểm tra chúng phải cùng `checkpoint_sha256` và
`registry_sha256`, lệch một cái là ném lỗi lúc khởi tạo.

## 2. Thêm một dòng vào deploy YAML

Trong khối `rma_meta:`, cạnh `crossfade_s`:

    handover_via_expert: flat

Thiếu dòng này, runner quay lại hoán đổi thẳng slope_up <-> slope_down, đúng
loại chuyển tiếp bạn thấy hỏng trên MuJoCo.

## 3. Thay `rma_meta_runner.h`

Không thay thì hai thứ trên vô nghĩa. Runner cũ có hai lỗi:

- `crossfade_from_action_` là ảnh chụp đông cứng lấy tại lúc switch. Đầu blend
  robot bị lái bằng một vector chết trong khi thân vẫn chuyển động. Đo được:
  fall 0.53 ở crossfade 0.2s, 0.77 ở 0.6s, 0.98 ở 1.2s. Bản mới giữ expert đi
  ra chạy sống suốt thời gian blend.
- Không đọc `handover_via_expert`. Bản mới chuyển qua `flat` trước rồi tự động
  đi tiếp tới expert được yêu cầu khi blend xong.

Biên dịch lại như cũ; smoke test đã PASS trên máy train.

## Kết quả

    slope_up   -> slope_down     0     <- không còn
    slope_down -> slope_up       0     <- không còn
    slope_up   -> flat         655
    slope_down -> flat        1075
    flat -> slope_up          1638
    flat -> slope_down        1899

    fall_fraction   0.4206

Vẫn chưa dùng được trên robot thật, `rma_meta.enabled` để `false`. Nguyên nhân
còn lại: expert `flat` ngã 0.0244/bước khi ở trên dốc, gấp 12-40 lần hai expert
dốc, nên bàn giao lệch vài bước là trả giá. Cần train lại `flat` với dốc nhẹ
0-8 độ mới xử lý triệt để.

Bản đầy đủ (kèm checkpoint, evaluation, tools kiểm tra) ở `to_dev_rma_v5/`.
