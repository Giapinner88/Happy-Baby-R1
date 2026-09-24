# V4 — `flat_plus_gait_lstm_v1`: nhánh recurrent

> Mỗi step chỉ dùng canonical Flat cơ bản 83D cộng gait one-hot; không lấy
> observation proprioceptive từ policy hay route khác.

| | |
|---|---|
| **Contract** | `flat_plus_gait_lstm_v1` |
| **Task** | `Unitree-R1-Flat-Plus-Gait-LSTM` |
| **Input mỗi step** | 86 = base83 + gait one-hot 3 |
| **Action** | 24 |
| **Cha** | không có — train from scratch hoặc distill |
| **Trạng thái** | **ĐÃ TRIỂN KHAI LÕI** — chưa train |
| **Vai trò** | **Ablation nghiên cứu**, không phải đường deploy |

## Vì sao tách riêng khỏi V3

Bài báo gốc dùng **một policy recurrent** (LSTM hidden 64 + MLP [32]). Bản R1
chọn MLP + frame stack cho candidate chính, có chủ đích: nếu đổi đồng thời
history, reward routing, curriculum, gait ID **và** kiến trúc mạng, thì kết quả
tốt hơn hay xấu đi cũng không quy được cho thành phần nào.

Nhưng bỏ hẳn LSTM cũng không được: nó là kiến trúc mà bài báo thực sự chứng minh
trên G1 thật. Nên nó là một nhánh riêng, có contract riêng, chạy song song để
trả lời đúng một câu: **recurrence có hơn frame stack không, ở cùng bài toán
này?**

Bài báo **không** trả lời câu đó — nó không có ablation LSTM vs frame stack.

## Đã triển khai (2026-09-06)

Hai task riêng, vì plan đòi hai cấu hình:

```bash
python scripts/train.py Unitree-R1-Flat-Plus-Gait-LSTM           # paper-fidelity
python scripts/train.py Unitree-R1-Flat-Plus-Gait-LSTM-Capacity  # capacity-matched
```

- `GaitConditionedRNNModel` kế thừa cả `RNNModel` lẫn model partial-norm của
  V3 qua MRO `GaitConditionedRNNModel -> RNNModel -> _GaitPartialNormModel ->
  MLPModel`, nên `RNNModel.get_latent()` gọi `super()` là rơi đúng vào nhánh
  normalize-prefix-only — không phải chép lại quy tắc lần thứ hai.
- Hidden size capacity-matched **được giải ra lúc import**, không gõ tay:
  `capacity_matched_hidden()` cho **h=247**, actor 339.708 param so với MLP
  339.352 → lệch **0.10%** (giới hạn 10%). Paper h=64 chỉ 41.784 param, tức
  nhỏ hơn 8 lần — đúng lý do plan bắt buộc arm thứ hai.
- ONNX export có `h/c` là I/O thật:
  `obs[1,86], h_in[1,1,H], c_in[1,1,H] → actions[1,24], h_out, c_out`, đã kiểm
  hidden state thực sự carry qua các step. Metadata ghi `deployable=false` và
  runner từ chối export nếu graph không đúng dạng stateful.
- Per-step input là **86**, không phải 335: recurrence mới là thứ mang history,
  stack thêm frame sẽ làm nhiễu đúng biến đang đo. Runner fail nếu thấy 335.

**Chưa làm:** train, parity `h/c` nhiều step giữa PyTorch và ONNX Runtime,
distill, và (đúng như plan yêu cầu) **không** mở đường runtime C++.

## Hai cấu hình bắt buộc chạy

| Cấu hình | Mạng | Mục đích |
|---|---|---|
| `paper-fidelity` | LSTM hidden 64, 1 layer, MLP [32] | Tái hiện đúng bài báo |
| `capacity-matched` | LSTM sao cho số tham số actor nằm trong **±10%** của H4-MLP | Loại bỏ chênh lệch dung lượng |

Thiếu cấu hình thứ hai thì so sánh trộn lẫn "lợi ích của recurrence" với "chênh
lệch số tham số" — H4-MLP (512,256,128) lớn hơn MLP [32] của bài báo khoảng hai
bậc.

## Chi phí kỹ thuật riêng của nhánh recurrent

Không dùng lại được hạ tầng của V1/V3:

- `h`/`c` state phải là input **và** output của ONNX — runtime C++ hiện tại giả
  định một tensor vào, một tensor ra
- reset mask theo từng environment trong rollout
- sequence rollout + truncated BPTT trong PPO
- parity export/runtime riêng: trạng thái ẩn phải khớp giữa PyTorch và ONNX
  Runtime qua nhiều step liên tiếp, không chỉ một step
- HB và simulator cần đường chạy stateful mới, kèm quy tắc reset trạng thái ẩn ở
  mọi lần kích hoạt policy — tương đương những gì `BaseObservationHistory` đang
  làm cho V1, nhưng cho `h`/`c`

Vì vậy: **không đưa LSTM vào artifact `flat_plus_gait_h4_v1`**, và không mở
đường runtime cho nó trước khi nó thắng ablation.

## Acceptance

- [ ] Cả `paper-fidelity` và `capacity-matched` chạy đủ 3 seed
- [ ] Cùng command program, reward, DR, budget, parent initialization với V3
- [ ] Báo cáo riêng số tham số actor của từng nhánh
- [ ] Nếu LSTM thắng: phải qua parity `h`/`c` nhiều step trước khi bàn tới deploy
- [ ] Nếu hoà: chọn MLP — runtime đã có, không có trạng thái ẩn để đồng bộ

---

## Kế hoạch chi tiết (nguyên văn từ bản gộp 2026-09-05)

### Network, warm-start và ablation LSTM

Candidate chính giữ topology actor/critic `(512,256,128)` ELU. Chỉ first layer đổi:

```text
Actor: [512,332] -> [512,335]
Critic: [512,98]  -> [512,101]
```

Warm-start bảo toàn output:

1. Copy 332 cột actor H4 vào `[0,332)`.
2. Đặt ba cột gait mới `[332,335)` bằng zero.
3. Copy bias, layer sau và action std.
4. Với critic, copy 98 cột cũ và zero ba cột gait mới, hoặc khởi tạo critic mới; quyết định phải giống giữa các ablation.
5. Copy nguyên normalizer H4 cho prefix 332D; gait3 đi qua nhánh raw/pass-through và không có running statistics. Không tự normalize one-hot ở C++.

Ngay sau converter, action của policy 335D phải bằng parent 332D cho cùng H4
input với mọi gait ID. Kiểm mapping ở float64 với `max_abs <= 1e-10`; ở
float32 dùng `max_rel <= 1e-3` và kiểm tra riêng không có rò rỉ từ các cột mới.
Không dùng `atol <= 1e-6` vì phép reduction rộng hơn có sai số thứ tự cộng đã đo
được ở H4. Đây chỉ là initialization parity; sau PPO actor phải học sử dụng gait ID.

Ablation tối thiểu:

| Arm | Input/network | Mục đích |
|---|---|---|
| `flat_plus` | H4 332D, MLP | History-only control |
| `flat_plus_gait` | H4 332D + gait3, MLP | Candidate đầy đủ |
| no gait input | H4 332D, cùng routing/curriculum | Đo giá trị conditioning |
| no reward routing | H4 + gait3, mọi gait reward cùng bật | Đo routing như paper |
| no curriculum | H4 + gait3, ba mode từ đầu | Đo curriculum như paper |
| no W2S | Vẫn 335D; W2S channel luôn 0, Stand/Walk đổi trực tiếp | Đo giá trị transition mode mà không đổi capacity/input shape |
| shuffled/counterfactual gait ID | H4 + gait3 | Xác nhận actor thực sự dùng ID |

Nhánh recurrent riêng:

```text
Tên:             flat_plus_gait_lstm
Contract:        flat_plus_gait_lstm_v1
Per-step input:  current base83 + current gait3 = 86D
Paper-fidelity:  LSTM hidden 64, 1 layer, MLP [32]
Output:          action24
```

Ngoài paper-sized network, cần một LSTM capacity-matched có số parameter actor nằm trong khoảng ±10% của H4-MLP; nếu không, so sánh sẽ trộn lợi ích recurrence với chênh lệch capacity. LSTM contract phải có `h/c` state I/O, reset mask, sequence rollout, truncated-BPTT và export/runtime parity riêng. Không đưa LSTM vào artifact `flat_plus_gait_h4_v1`.

Mọi quyết định dùng ít nhất 3 seed, cùng command program, reward, domain randomization, budget và parent initialization. Kết quả paper không chứng minh LSTM tốt hơn frame stack vì paper không có ablation này.
