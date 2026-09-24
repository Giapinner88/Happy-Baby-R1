# Roadmap locomotion cải tiến R1 — chỉ mục

> **Khóa phạm vi:** mọi phiên bản trong roadmap chỉ dùng canonical
> `r1_common_pd_base83_v1` của **Flat cơ bản 83D** làm nguồn observation
> proprioceptive. Ở đây “chỉ dùng Flat 83D” nói về **nguồn frame**, không có
> nghĩa actor của mọi contract vẫn rộng 83D: H4/H5 là các view lịch sử của frame
> 83D, gait ID là conditioning hiện tại, còn latent là output của adapter. Không
> lấy observation từ policy hay route khác làm nguồn.

Mỗi phiên bản một file. Bản gộp gốc ngày 2026-09-05 giữ nguyên vẹn ở
`99_ARCHIVE_ban_gop_2026-09-05.md`. Các file 01/03/04/05 giữ phần chi tiết để
đối chiếu, nhưng status, correction và tiêu chí ở phần hiện tại của từng file
được ưu tiên; không dùng đoạn historical bên dưới để suy trạng thái mới.

## Trạng thái

| # | Phiên bản | Contract | Actor | Trạng thái | Khối lượng |
|---|---|---|---:|---|---|
| [01](01_h4_history.md) | History H=4 | `flat_plus_h4_v1` | 332 | **Đã triển khai lõi**, preflight sạch, đang train | — |
| [02](02_h5_history.md) | History H=5 | `flat_plus_h5_v1` | 415 | **Đã train seed 42** — hoà với H4, chọn H4 | Nhỏ |
| [03](03_gait_conditioned.md) | Gait-conditioned | `flat_plus_gait_h4_v1` | 335 | **Đã triển khai lõi** (train-side), chưa train | **Lớn nhất** |
| [04](04_gait_lstm.md) | Recurrent ablation | `flat_plus_gait_lstm_v1` | 86/step | **Đã triển khai lõi**, chưa train | Trung bình |
| [05](05_arma.md) | A-RMA latent | `flat_plus_arma_o83_k50_z8_v1` | 91 (chính; 340/343 là biến thể) | **Mới có schema + adapter view**; pipeline train chưa làm | Lớn, nhiều pha |

Đối chứng, **không** phải cải tiến: `Unitree-R1-Flat-Meta` (83D, common-PD) —
đây là baseline `policy_goc_2.onnx` đang chạy trên robot.

## Thứ tự phụ thuộc

```text
Flat-Meta (83D, baseline đang chạy)
   │
   ├─ 01  H4  332D ──── lõi đã xong ─── còn hardening + train
   │       │
   │       ├─ 03  Gait 335D ──┬─ 04  LSTM  (ablation của 03)
   │       │                  └─ (gait + A-RMA: chỉ sau khi 03 và 05 đều xong)
   │       │
   │       └─ 05  A-RMA 91D   (độc lập với 03)
   │
   └─ 02  H5  415D  ── song song với 01, KHÔNG kế thừa 01
```

**02 không phụ thuộc 01 về mặt kỹ thuật, nhưng 01 phụ thuộc 02 về mặt kết luận.**
Gate 4 của kế hoạch gốc đòi ma trận H1/H4/H5. Không có H5 thì việc chọn H=4 là
phỏng đoán chứ không phải kết quả đo. Nên chạy 02 **cùng đợt train với 01**.

03, 04, 05 chỉ bắt đầu triển khai/training sau khi **01 qua Gate 7**
(closed-loop simulator). V5 có thể được thiết kế và chuẩn bị tài liệu trước,
nhưng không được tạo artifact hoặc chạy training sớm hơn Gate 7.

## Thứ tự đề xuất

| Đợt | Làm gì | Vì sao |
|---|---|---|
| **1** | Triển khai 02, rồi train 01+02 cùng lúc (H1/H4/H5 × 3 seed) | 02 rẻ vì hạ tầng đã tham số hoá; train chung một đợt là cách duy nhất giữ budget/seed/DR cố định giữa các nhánh |
| **2** | Gate 5-7 cho nhánh thắng: export, parity, closed-loop sim | Chốt xem history có thật sự hơn baseline không |
| **3** | Triển khai 03 | Khối lớn nhất; chỉ đáng làm nếu đợt 2 cho thấy history có giá trị |
| **4** | 04 chạy song song 03 làm ablation | Trả lời "recurrence có hơn frame stack không" |
| **5** | 05 A-RMA | Chỉ triển khai/training sau Gate 7 của V1; sau đó vẫn độc lập với V3, và có thể benchmark trước khi mở contract gait+A-RMA |

## Bất biến áp cho mọi phiên bản

Vi phạm bất kỳ dòng nào dưới đây thì đó là **contract mới**, không phải biến thể:

- action 24D, joint order, default joint position, action scale, stiffness, damping
- gait period **0.6 s**, policy rate **50 Hz**
- actor chỉ dùng tín hiệu tái tạo được trên robot — không camera, height scan,
  contact truth, base linear velocity, terrain label
- artifact nằm ở namespace riêng; **không** ghi đè `policy_goc_2.onnx` hay
  `model_flat_common_pd.pt`
- `HB/high_level_2/config/locomotion.yaml` chỉ đổi sau khi qua hardware
  acceptance, và là một thay đổi riêng có rollback đã thử
- runtime dispatch bằng **exact contract**, không bao giờ bằng `input_dim` — hai
  vector 332D term-major và time-major không phân biệt được bằng kích thước

## Bài học từ 01, áp cho các phiên bản sau

1. **Đo, đừng suy từ source.** Layout history được xác nhận bằng tensor thật của
   env, không phải bằng đọc `ObservationManager`. Test đọc source cũng pass khi
   layout sai.
2. **Test phải có negative control.** Golden trace chỉ đáng tin sau khi tôi tự
   tạo một trace time-major và xác nhận test fail đúng chỗ.
3. **Ngưỡng float32 trong plan phải thực tế.** Với warm-start mở từ 83 lên 332
   số hạng, dùng float64 để khẳng định mapping (`max_abs ≤ 1e-10`); float32 chỉ
   kiểm tra không rò slot cũ và dùng tolerance tương đối đã đo (`max_rel ≤ 1e-3`),
   không dùng `atol ≤ 1e-6`.
4. **Capability dương, không suy diễn.** `SupportsArmOverlay()` phải được hỏi
   thẳng; chỉ route Flat legacy 83D được opt-in, history policy mặc định từ chối.
5. **Fail-closed cần nhánh cho artifact cũ.** Các model legacy trong repo không
   có `policy_contract`; resolver vẫn phải nhận đúng dimension legacy đã khóa.

## Nợ đã biết, hoãn có chủ đích

- **`Unitree-R1-Slope-Up/Down` có warm-start mặc định trỏ file không tồn tại**
  (`checkpoints/r1_flat/model_flat.pt`). Cha đúng của Slope chưa xác định —
  đoán sai sẽ tạo đúng cái lỗi âm thầm mà đợt sửa Flat-Plus nhằm loại bỏ. Chạy
  `train.py Unitree-R1-Slope-Up` trần sẽ `FileNotFoundError`; truyền
  `--warm-start-checkpoint` tường minh thì vẫn chạy. Để lại cho đợt cải tiến
  Slope. Ba arm H1/H4/H5 không bị ảnh hưởng: chúng nằm trong
  `FROM_SCRATCH_TASK_IDS` và được khớp trước nhánh Slope.

- **Index entity-local vs global model.** `SceneEntityCfg.body_ids` /
  `geom_ids` đánh số theo **entity** (robot có 25 body), còn mọi mảng
  `env.sim.model.*` đánh số theo **global model** (27 body). Đọc thẳng
  `sim.model.body_ipos[:, asset_cfg.body_ids]` sẽ lệch 2 body và trả về giá trị
  nominal của một body khác — hằng số, hữu hạn, hợp lý, và sai. Map đúng là
  `robot.indexing.body_ids[asset_cfg.body_ids]` (tương tự `indexing.geom_ids`),
  chính là map mà bản thân domain randomization dùng.

  Ghi lại vì nó đã làm tôi kết luận nhầm ngày 2026-09-06 rằng `base_com` không
  randomize. Đo lại bằng index đúng: `base_com` **có** varies per-env
  (std ≈ 0.025/0.035/0.031 trên ±0.05), và **cả 14** foot geom đều varies, dùng
  chung một giá trị trong mỗi env đúng như `shared_random=True`. Không có lỗi
  domain randomization nào; H4/H5 đang train với đúng lượng DR mà config mô tả.
  Regression test: `tests/test_arma_contract.py::test_factor_readers_use_global_model_indices`.

- **`Unitree-R1-Flat-Meta` từng warm-start ngầm** từ `model_flat_common_pd.pt`,
  di sản của thời nó là run *tạo ra* chính file đó. Là arm H1-control thì đó là
  lỗi: nó khiến ablation đo initialization thay vì cửa sổ history. Default đã bị
  bỏ ngày 2026-09-06. Lineage cũ tái lập bằng
  `--warm-start-checkpoint checkpoints/r1_flat/model_flat_goc.pt` (file này vẫn
  còn trong repo).

## File khác trong folder

- [`TRAIN_MACHINE_HANDOFF.md`](TRAIN_MACHINE_HANDOFF.md) — hướng dẫn máy train
  cho 01/02, kèm đủ bốn nhánh `H1-control`, `H4-mapped`, `H4-scratch` và `H5`.
- [`99_ARCHIVE_ban_gop_2026-09-05.md`](99_ARCHIVE_ban_gop_2026-09-05.md) — bản
  gộp gốc 1653 dòng, giữ để tra nguyên văn.
