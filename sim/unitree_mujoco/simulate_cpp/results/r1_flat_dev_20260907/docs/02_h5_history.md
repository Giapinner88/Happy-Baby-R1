# V2 — `flat_plus_h5_v1`: cửa sổ history H=5

> Nguồn observation duy nhất: canonical Flat cơ bản 83D; 415D chỉ là năm frame
> 83D đóng gói term-major.

| | |
|---|---|
| **Contract** | `flat_plus_h5_v1` |
| **Task** | `Unitree-R1-Flat-Plus-H5` |
| **Actor / critic / action** | 415 / 98 / 24 |
| **Cha** | `Unitree-R1-Flat-Meta` (giống hệt V1, **không** kế thừa từ H4) |
| **Trạng thái** | **ĐÃ TRIỂN KHAI LÕI** — chưa train/export/acceptance |
| **Khối lượng** | Nhỏ — hạ tầng đã tham số hoá sẵn |

## Đã triển khai lõi

- `history_contract.py` đã khai báo H4/H5, contract ID, dimension và metadata
  riêng cho từng cửa sổ.
- `env_cfgs.py`, `rl_cfg.py` và registry đã có task/runner
  `Unitree-R1-Flat-Plus-H5` độc lập, kế thừa trực tiếp từ Flat-Meta.
- Converter, golden trace, checker và test suite đã parameterize theo H; các
  test hiện có kiểm tra H5 actor 415D, critic 98D, action 24D, layout, reset,
  normalizer và warm-start parity.

Đây mới là contract/task readiness, không phải bằng chứng policy H5 đã học tốt
hoặc có thể deploy.

## Kết quả train — H4 vs H5, seed 42 (2026-09-06)

Cả hai from-scratch, **cùng seed 42, cùng 20000 iteration, 4096 env**, cùng
reward/DR/PPO/budget. Khác nhau đúng một thứ: độ dài cửa sổ actor.

Trung bình 200 iteration cuối:

| Metric | H4 | H5 | Δ |
|---|---:|---:|---:|
| Mean reward | 37.22 | 37.07 | −0.15 |
| Episode length | 998.86 | 998.97 | +0.11 |
| `error_vel_xy` | 0.6711 | 0.6669 | −0.0041 |
| `error_vel_yaw` | 1.1815 | 1.1871 | +0.0056 |
| `fell_over` | 0.0100 | 0.0096 | −0.0004 |
| `foot_gait` | 0.3955 | 0.3961 | +0.0006 |
| `slip_velocity_mean` | 0.1327 | 0.1316 | −0.0011 |

**Không metric nào lệch quá 1%** (trừ `fell_over` và `foot_slip`, lệch ~4%
tương đối nhưng trên một đại lượng gần 0).

### Chênh lệch có vượt nhiễu không

Một seed thì không có sai số giữa seed. Thay bằng dao động **trong chính run**
(σ gộp 200 iteration cuối của cả hai) làm thước đo nhiễu:

| Metric | \|gap\| | σ | gap/σ |
|---|---:|---:|---:|
| Mean reward | 0.150 | 0.303 | **0.50** |
| `error_vel_xy` | 0.0041 | 0.0166 | **0.25** |
| `error_vel_yaw` | 0.0056 | 0.0074 | **0.76** |
| `track_linear_velocity` | 0.0027 | 0.0069 | **0.40** |
| `track_angular_velocity` | 0.0019 | 0.0030 | **0.64** |
| `foot_gait` | 0.0006 | 0.0109 | **0.06** |
| `slip_velocity_mean` | 0.0011 | 0.0013 | **0.88** |
| `fell_over` | 0.0004 | 0.0198 | **0.02** |

`gap/σ < 1` ở **mọi** metric — chênh lệch H4/H5 nhỏ hơn dao động tự nhiên của
chính một run. Không có tín hiệu nào cho thấy cửa sổ 80 ms hơn 60 ms.

### Kết luận: **chọn H4**

Theo §6: hoà thì chọn H4 — cửa sổ ngắn hơn, ít bộ nhớ hơn, ít độ trễ hơn,
runtime đã có sẵn. Gate 4 (ma trận H1/H4/H5) coi như đã trả lời phần H4 vs H5.

### Giới hạn phải nói rõ

- **Một seed.** §6 đòi tối thiểu 3 seed; người dùng chốt so trên seed 42. Nên
  phát biểu đúng là **"chưa thấy H5 hơn H4"**, không phải "H4 hơn H5". Với
  gap/σ < 1 ở mọi trục thì thêm seed nhiều khả năng không đổi hướng kết luận,
  nhưng đó là suy đoán chứ không phải kết quả đo.
- **σ trong-run không thay được σ giữa-seed.** Nó bắt được nhiễu sampling/rollout,
  không bắt được nhiễu khởi tạo mạng. σ giữa seed thường lớn hơn — tức là ngưỡng
  thật còn dễ dãi hơn, kết luận "hoà" càng vững.
- **H1-control chưa train.** Chưa trả lời được câu "history có hơn baseline 83D
  không" — mới chỉ trả lời "H5 có hơn H4 không".
- Chưa qua Gate 5 parity đầy đủ và Gate 7 closed-loop.

## Còn phải làm

- Chạy preflight trên máy train; arm chính chạy from-scratch. Checkpoint
  warm-start H5 chỉ tạo nếu cần smoke/ablation phụ.
- Train H1/H4/H5 với cùng initialization protocol, seed, budget, PPO và domain
  randomization.
- Export/check ONNX H5, so parity và đưa H5 vào ma trận closed-loop chỉ để so
  sánh; không thêm H5 vào C++/HB trước khi quyết định schema thắng.

## 1. Vì sao phiên bản này tồn tại

Nó **không phải** một cải tiến độc lập, mà là điều kiện để kết luận về V1 có
nghĩa. Gate 4 của kế hoạch gốc yêu cầu ma trận tối thiểu **H1 / H4 / H5**:

| Nhánh | Input | Trả lời câu hỏi |
|---|---:|---|
| H1 | 83 | Đối chứng, cùng pipeline |
| H4 | 332 | Cửa sổ 60 ms có hơn không? |
| **H5** | **415** | **H=4 có phải điểm dừng đúng, hay dài hơn còn tốt hơn?** |

Không có H5 thì chọn H=4 là chọn theo phỏng đoán. Nếu H5 tốt hơn rõ rệt và lặp
lại giữa các seed, đó là tín hiệu cửa sổ nên dài hơn nữa và cả contract v1 cần
xem lại trước khi deploy — rẻ hơn nhiều so với phát hiện điều đó sau khi đã đưa
H4 lên robot.

H = 5 ở 50 Hz là cửa sổ **80 ms**, so với 60 ms của H4.

## 2. Contract

Layout giữ đúng quy tắc của mjlab, chỉ đổi số slot:

```text
Base frame:      r1_common_pd_base83_v1, 83 float32
History length:  H = 5
Actor input:     5 x 83 = 415 float32
Critic input:    98, KHÔNG stack
History order:   term-major, oldest -> newest trong từng term
Reset padding:   repeat_first (backfill frame hợp lệ đầu tiên)
```

Bảng offset sinh ra từ chính công thức đã có; **không hard-code lại**:

| Term | Block `[start, stop)` | Slice hiện tại |
|---|---|---|
| `base_ang_vel` | `[0, 15)` | `[12, 15)` |
| `projected_gravity` | `[15, 30)` | `[27, 30)` |
| `command` | `[30, 45)` | `[42, 45)` |
| `phase` | `[45, 55)` | `[53, 55)` |
| `joint_pos` | `[55, 175)` | `[151, 175)` |
| `joint_vel` | `[175, 295)` | `[271, 295)` |
| `actions` | `[295, 415)` | `[391, 415)` |

Các số này phải được **sinh** bằng `history_term_blocks(5)` /
`current_term_slices(5)` rồi mới đối chiếu với bảng, chứ không gõ tay vào code.

## 3. Vì sao kế thừa Flat-Meta chứ không phải Flat-Plus

Kế thừa H4 rồi nâng `history_length` lên 5 sẽ chạy được, nhưng làm hai nhánh
ablation không còn độc lập: mọi thay đổi sau này ở factory H4 sẽ lặng lẽ chảy
sang H5 và phá điều kiện "giữ cố định mọi thứ trừ độ dài cửa sổ". Cả hai cùng
gọi `unitree_r1_flat_meta_env_cfg()`.

## 4. Việc phải làm

`history_contract.py` đã nhận `history_length` làm tham số ở mọi hàm, nên phần
lớn là đăng ký thêm chứ không phải viết mới.

| File | Thay đổi |
|---|---|
| `src/tasks/velocity/config/r1/history_contract.py` | Tách hằng số theo H thành factory nhỏ (`contract_id(h)`, `actor_dim(h)`) thay vì hằng số module cố định cho H4; giữ nguyên tên cũ trỏ về H4 để không phá import hiện có |
| `src/tasks/velocity/config/r1/env_cfgs.py` | `unitree_r1_flat_plus_h5_env_cfg()` — cùng thân với H4, khác đúng `history_length=5` |
| `src/tasks/velocity/config/r1/rl_cfg.py` | `unitree_r1_flat_plus_h5_ppo_runner_cfg()`, `experiment_name="r1_flat_plus_h5"` |
| `src/tasks/velocity/config/r1/__init__.py` | Đăng ký `Unitree-R1-Flat-Plus-H5` |
| `src/tasks/velocity/rl/flat_plus_runner.py` | Runner nhận H làm tham số, gắn metadata theo đúng H đang chạy |
| `scripts/init_flat_plus_checkpoint.py` | Đã có `--history-length`; chỉ cần cho phép ghi ra namespace `h5_v1` |
| `scripts/check_flat_plus.py` | Nhận `--history-length` để kiểm đúng contract tương ứng |
| `scripts/make_flat_plus_golden_trace.py` | `--history-length 5` ghi ra `checkpoints/r1_flat_plus/h5_v1/golden_trace.txt` — mỗi contract một namespace, không dùng hậu tố `_h5` trên một file dùng chung |
| `tests/test_flat_plus_contract.py` | Parametrize theo H thay vì gán cứng 4 |
| `checkpoints/r1_flat_plus/h5_v1/` | Namespace artifact riêng |

**Phía C++ chưa cần đụng.** H5 chỉ tồn tại để so sánh lúc train. Chỉ khi H5
thắng và được chọn deploy thì mới thêm route 415D vào runtime — và khi đó là một
contract mới với đầy đủ vòng metadata/parity như V1 đã làm.

## 5. Bẫy

| Rủi ro | Hậu quả | Chặn bằng |
|---|---|---|
| Hard-code offset H5 | Sai term/timestep, model vẫn chạy | Sinh từ hàm, test đối chiếu bảng |
| Kế thừa từ H4 | Hai nhánh không độc lập, ablation vô nghĩa | Cùng gọi Flat-Meta |
| Dùng chung `experiment_name` | Log/checkpoint lẫn nhau | `r1_flat_plus_h5` riêng |
| So H5 với H4 khác initialization | Chênh lệch không quy được cho H | Cùng from-scratch protocol, cùng budget; mapped chỉ báo cáo riêng |
| Đưa 415D vào C++ sớm | Mở bề mặt lỗi cho một nhánh có thể bị loại | Chỉ thêm sau khi H5 được chọn |

## 6. Acceptance

Không có "gate deploy" riêng cho H5. Nó chỉ cần đủ điều kiện để **so sánh công
bằng** với H4:

- [ ] actor 415 / critic 98 / action 24 trên env thật
- [ ] layout term-major, oldest→newest, backfill — đo bằng tensor như V1 đã làm
- [ ] nếu có mapped ablation: converter parity float64 với cùng cha `model_flat_common_pd.pt`
- [ ] cùng reward / command range / DR / PPO hyperparams / budget / seed set với H4
- [ ] tối thiểu 3 seed
- [ ] báo cáo H1 vs H4 vs H5 trên cùng ma trận đánh giá

Nếu H4 và H5 hoà nhau trong sai số giữa các seed thì **chọn H4** — cửa sổ ngắn
hơn, ít bộ nhớ hơn, ít độ trễ hơn, và runtime đã có sẵn.
