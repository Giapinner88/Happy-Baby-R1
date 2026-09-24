# Bàn giao máy train — `flat_plus_h4_v1`

> Scope dữ liệu: chỉ canonical Flat cơ bản 83D; H1/H4/H5 là các view khác nhau
> của cùng base frame, không trộn observation từ route khác.

> Ngày: 2026-09-06 (cập nhật cùng ngày sau audit code)
> Kế hoạch hiện tại: `documents/R1_FLAT_PLUS_PLANS/00_INDEX.md`
> Trạng thái phía dev: Gate 0-2, Gate 3a (converter) và phần packing của Gate 6
> đã PASS. Gate 3b (train smoke/save-reload/export) chưa đạt. Sau Phase 4
> (train + ablation), metadata/tensor hardening, ONNX parity và closed-loop
> acceptance vẫn phải hoàn tất trước khi nói tới release.

## Cập nhật 2026-09-06 — train chạy tại chỗ

Phần cứng máy dev **đã đổi**: hiện là **2 × NVIDIA RTX 4500 Ada** (24 GB mỗi
card, sm_89, driver 580.173.02 / CUDA 13). Ghi chú cũ "máy dev không train được
vì Quadro P2000 (Pascal, sm_61), 4 GB VRAM" **không còn đúng** và đã bị bỏ.
Training H1/H4/H5 chạy ngay trên máy này; các mục 1, 7 và 10 bên dưới giữ lại
cho trường hợp thật sự phải chuyển máy, còn quy trình bundle vẫn dùng để đóng
gói kết quả cho gate export/parity.

Sửa cùng đợt (audit code 2026-09-06):

- `scripts/train.py` **không còn** default warm-start cho `Unitree-R1-Flat-Meta`.
  Trước đó arm H1 âm thầm warm-start từ `model_flat_common_pd.pt` trong khi
  H4/H5 chạy from-scratch — ablation khi đó đo initialization chứ không đo cửa
  sổ history. Nay cả ba arm from-scratch, đúng 01 §5.2.
- Tiêu chí float32 của converter đổi từ max-relative sang `atol + rtol*|want|`
  (§4).
- `collect_flat_plus_bundle.py` đặt tên DEPLOY theo contract của arm được chọn,
  không hard-code h4 (§7).

---

## 1. Chép gì sang máy train

Từ `unitree_rl_mjlab_meta/`:

```
src/                                     # đã có task Unitree-R1-Flat-Plus
scripts/train.py
scripts/init_flat_plus_checkpoint.py
scripts/check_flat_plus.py
scripts/collect_flat_plus_bundle.py
scripts/make_flat_plus_golden_trace.py
tests/
setup.py                                             # `pip install -e .` cần
checkpoints/r1_flat/model_flat_common_pd.pt          # cha của mapped diagnostic
checkpoints/r1_flat_plus/h4_v1/golden_trace.txt      # trace chuẩn 3 runtime, H4
checkpoints/r1_flat_plus/h5_v1/golden_trace.txt      # trace chuẩn 3 runtime, H5
documents/R1_FLAT_PLUS_PLANS/00_INDEX.md
documents/R1_FLAT_PLUS_PLANS/01_h4_history.md
documents/R1_FLAT_PLUS_PLANS/02_h5_history.md
documents/R1_FLAT_PLUS_PLANS/TRAIN_MACHINE_HANDOFF.md  # file này
```

Thiếu `h5_v1/golden_trace.txt` thì `check_flat_plus.py --history-length 5`
SKIP phần inference, và `--strict` biến SKIP đó thành fail — nên nó không phải
file tùy chọn.

`model_flat_common_pd.pt` phải có SHA256:

```
b6090c47723a625859fb20f4d288b0149712ef537bb8c83025136bad3f40c1f7
```

Sai hash là dừng — mọi so sánh với đối chứng H1 sẽ vô nghĩa nếu cha khác nhau.

**Không cần** chép `HB/` hay `unitree_mujoco/`. Phần C++ ở lại máy dev.

---

## 2. Môi trường

```bash
pip install -e .          # kéo mjlab==1.2.0, rsl-rl-lib==5.0.1, onnx, scipy
```

`setup.py` nay khai báo cả `onnx` và `scipy`: `src.tasks` import `onnx` ở module
scope cho export gate, còn `scipy` là thứ mjlab dùng nhưng không khai báo. Trước
đây phải `pip install scipy` tay.

Bản pin: `mjlab 1.2.0` / `rsl-rl-lib 5.0.1`. **Đừng nâng.** Layout history 332D
phụ thuộc vào cách `ObservationManager` của đúng bản này nối các term
(term-major, oldest→newest, backfill khi reset). Bản khác có thể đổi hành vi đó
mà không có lỗi nào báo ra, vì kích thước vector vẫn là 332.

---

## 3. Preflight — chạy TRƯỚC khi train

```bash
python -m pytest tests/test_flat_plus_contract.py tests/test_flat_plus_warm_start.py -q
python scripts/check_flat_plus.py
```

Phải thấy `actor 332 / critic 98 / action 24` và `actor term order` PASS. Nếu
term order khác đi thì **dừng lại** — bảng offset mà C++ biên dịch sẵn không còn
đúng, và mọi thứ train ra sẽ không dùng được.

Kiểm cả nhánh H5:

```bash
python scripts/check_flat_plus.py --history-length 5
```

Kỳ vọng hiện tại (2026-09-06, chưa train): pytest **41 passed, 1 skipped**
(skip là parity PyTorch↔ONNX, chưa có artifact), và cả hai lệnh `check_flat_plus`
báo **0 failed, 1 skipped**. `--strict` vẫn FAIL đúng thiết kế cho tới khi có
ONNX đã train.

---

## 4. Sinh checkpoint warm start

```bash
python scripts/init_flat_plus_checkpoint.py
# -> checkpoints/r1_flat_plus/h4_v1/init_from_common_pd.pt
```

Converter mở lớp đầu `[512,83]` → `[512,332]`, copy trọng số cũ vào **slice mới
nhất** của từng term, ba slot cũ để bằng 0. Nhân bản normalizer cho cả 4 slot.
Bỏ optimizer state và đặt `common_step_counter = 0` — nếu kế thừa counter của
10000 iteration cha thì curriculum lệnh sẽ nhảy thẳng sang stage cuối ngay bước
đầu.

Checkpoint này **chỉ dùng cho nhánh mapped diagnostic** ở §5. Ba arm chính không
đụng tới nó.

Đã kiểm: output giống hệt policy cha ở float64 (lệch 1.4e-14) với mọi frame cũ
tùy ý. Ở float32 lệch tối đa 4.8e-6 tuyệt đối, do thứ tự cộng dồn 332 số hạng
khác 83 số hạng — **không phải lỗi**.

Tiêu chí: float64 `max_abs ≤ 1e-10`, float32 `|Δ| ≤ atol + rtol·|want|` với
`atol=1e-4`, `rtol=1e-3`. Không dùng ngưỡng cũ `atol ≤ 1e-6`, và **cũng không**
dùng max-relative trần như bản trước: relative bị chi phối bởi component action
nào tình cờ gần 0 — đo trên torch 2.7.0/CPU, một component `3.0e-4` lệch
`4.7e-7` đọc ra thành 1.6e-3 "relative" và làm fail ngưỡng `1e-3`, trong khi sai
số tuyệt đối lớn nhất ở mọi nơi chỉ là 4.8e-6. Rò rỉ thật từ slot cũ là O(1),
không phải O(1e-5), nên atol tuyệt đối vẫn giữ nguyên sức phát hiện.

---

## 5. Ba arm chính phải chạy

Giữ **cố định** giữa các nhánh: reward, command range, domain randomization, PPO
hyperparameters, network dims, rollout length, budget, seed set, evaluation
scenarios. Nếu không thì không quy được cải thiện cho history.

Tối thiểu **3 seed mỗi nhánh**. Không chọn bằng một run đẹp nhất.

Ba arm chính: **H1-control · H4-scratch · H5-scratch**. Đây là kết quả dùng để
chọn H4 hay H5; tất cả đều train from-scratch với cùng initialization protocol.

**From-scratch là mặc định của code, không phải kỷ luật của người chạy.**
`TrainConfig.from_task` trả `warm_start_checkpoint=None` cho cả ba task id, và
`test_ablation_arms_train_from_scratch` gọi thẳng hàm đó để khoá lại. Ai muốn
warm-start phải truyền `--warm-start-checkpoint` tường minh, và khi đó nó là
nhánh diagnostic riêng, không nằm trong kết luận H1/H4/H5.

Máy này có 2 GPU nên H4 và H5 chạy song song, mỗi arm một card:

### H4-scratch — GPU 0

```bash
python scripts/train.py Unitree-R1-Flat-Plus --gpu-ids 0
```

### H5-scratch — GPU 1 (nhánh ablation cửa sổ 80 ms)

```bash
python scripts/train.py Unitree-R1-Flat-Plus-H5 --gpu-ids 1
```

Nhánh này **bắt buộc** theo Gate 4: thiếu nó thì việc chọn H=4 là phỏng đoán chứ
không phải kết quả đo. Nó kế thừa Flat-Meta trực tiếp, không kế thừa H4, nên hai
nhánh độc lập. Log ra `logs/rsl_rl/r1_flat_plus_h5/`.

### H1-control (đối chứng 83D, cùng pipeline)

```bash
python scripts/train.py Unitree-R1-Flat-Meta --gpu-ids 0
```

> Đối chứng phải retrain bằng **cùng pipeline**, không được lấy
> `policy_goc_2.onnx` cũ ra so. Không truyền checkpoint cũ cho arm chính vì nó
> train ở thời điểm khác, budget khác.
>
> Trước 2026-09-06 lệnh trần này **âm thầm** warm-start từ
> `model_flat_common_pd.pt`, vì `Unitree-R1-Flat-Meta` từng là run *tạo ra*
> chính file đó. Default ấy đã bị bỏ. Muốn tái lập lineage cũ thì truyền tay
> `--warm-start-checkpoint checkpoints/r1_flat/model_flat_goc.pt`.

### H4-mapped (optional diagnostic)

Chỉ chạy nếu muốn đo lợi ích của warm-start; không đưa vào kết luận H1/H4/H5:

```bash
python scripts/init_flat_plus_checkpoint.py --history-length 4
python scripts/train.py Unitree-R1-Flat-Plus \
    --warm-start-checkpoint checkpoints/r1_flat_plus/h4_v1/init_from_common_pd.pt
```

Smoke trước khi chạy full: vài chục env, 2-5 iteration, kiểm không NaN/Inf,
checkpoint save/reload/export được. Rồi ~100 iteration. Rồi mới full.

Mỗi runner tự export `policy.onnx` **kèm đầy đủ metadata history** ở mỗi lần
save, và fail-closed nếu tensor không đúng contract của arm: H4
`[1,332] → [1,24]` hoặc H5 `[1,415] → [1,24]` float32. Cụ thể: file sai contract
bị **xoá khỏi run dir** rồi `save()` raise, nên training dừng và không để lại
artifact nào mà bước đóng gói có thể nhặt nhầm.

---

## 6. Kiểm ONNX trước khi đóng gói

```bash
python scripts/check_flat_plus.py \
    --history-length 4 \
    --onnx logs/rsl_rl/r1_flat_plus/<run_h4>/policy.onnx --strict

python scripts/check_flat_plus.py \
    --history-length 5 \
    --onnx logs/rsl_rl/r1_flat_plus_h5/<run_h5>/policy.onnx --strict
```

`--strict` coi mọi mục SKIP là fail. Nó kiểm 21 khoá metadata, shape/dtype
tensor, và chạy inference trên `golden_trace.txt` để chắc action hữu hạn.

---

## 7. Đóng gói — đây là thứ cần gửi về

```bash
python scripts/collect_flat_plus_bundle.py --run logs/rsl_rl/r1_flat_plus/<run_h4_scratch> --arm h4_scratch
python scripts/collect_flat_plus_bundle.py --run logs/rsl_rl/r1_flat_plus_h5/<run_h5_scratch> --arm h5_candidate
python scripts/collect_flat_plus_bundle.py --run logs/rsl_rl/r1_flat_common_pd/<run_h1>    --arm h1_control --tar
# optional diagnostic, nếu đã chạy:
python scripts/collect_flat_plus_bundle.py --run logs/rsl_rl/r1_flat_plus/<run_h4_mapped> --arm h4_mapped
```

Mỗi lệnh bổ sung một nhánh vào cùng bundle, không ghi đè nhánh trước. Chỉ sau
khi có đủ kết quả ablation mới chạy lại lệnh của arm thắng với `--select` để
populate `DEPLOY/`.

Kết quả — một thư mục duy nhất, tên theo ngày, **không** theo contract nào:

```
bundles/flat_plus_ablation_<YYYYMMDD>/
├── README.md            gì trong này, nhánh nào thắng, làm gì tiếp
├── MANIFEST.yaml        provenance máy đọc được: run nào, hash nào, contract nào
├── SHA256SUMS           kiểm sau khi truyền
├── DEPLOY/              ★ CHỈ CẦN 2 FILE NÀY ĐỂ DEPLOY
│   ├── policy_<contract arm thắng>.onnx   vd. policy_flat_plus_h5_v1.onnx
│   └── model_<contract arm thắng>.pt
├── arms/
│   ├── h1_control/      model_final.pt · policy.onnx · env/agent/train.yaml · metrics/
│   ├── h4_scratch/
│   ├── h4_mapped/
│   └── h5_candidate/
└── evaluation/          bỏ kết quả closed-loop / bảng so sánh vào đây
```

Tên file trong `DEPLOY/` sinh từ contract của arm được `--select`, không hard-code
h4 — trước 2026-09-06 một policy H5 415D sẽ được đóng gói thành
`policy_flat_plus_h4_v1.onnx` với MANIFEST ghi `actor_input_dim: 332`, đúng kiểu
lệch tên/contract mà cả vòng metadata sinh ra để chặn. Ngoài ra:

- `--arm` chỉ nhận đúng bốn giá trị trong `KNOWN_ARMS`; gõ sai là lỗi ngay, không
  âm thầm tạo nhánh mới.
- mỗi arm được đối chiếu **actor input dim đọc từ chính file ONNX** với độ dài
  cửa sổ đã khai; lệch là dừng.
- `--select h1_control` bị từ chối: H1 là đối chứng 83D, và policy 83D đang chạy
  trên robot đã là `policy_goc_2.onnx`. H1 thắng nghĩa là "history không giúp
  gì", và không có gì để ship.

Kéo về máy dev:

```bash
scp -r <train>:.../bundles/flat_plus_ablation_<date>.tar.gz .
tar xzf flat_plus_ablation_<date>.tar.gz
cd flat_plus_ablation_<date> && sha256sum -c SHA256SUMS
```

---

## 8. Kèm theo bundle, ghi rõ trong `evaluation/`

- seed nào, budget bao nhiêu iteration, thời gian train
- learning curve / return của cả ba nhánh
- tracking RMSE theo trục, fall count, time-to-fall, zero-command drift
- so sánh H4 vs H5: nếu hoà nhau trong sai số giữa seed thì **chọn H4**
- lý do chọn nhánh thắng — nếu improvement không lặp lại giữa các seed thì nói rõ

---

## 9. Ranh giới — những việc máy train KHÔNG làm

- **Không** đổi `flat_policy_contract` / `flat_model` trong
  `HB/high_level_2/config/locomotion.yaml`. Việc đó chỉ xảy ra ở máy dev, sau
  khi qua Gate 7 closed-loop, và là một thay đổi riêng có rollback đã thử.
- **Không** ghi đè `policy_goc_2.onnx`, `model_flat_common_pd.pt` hay bất kỳ
  artifact Flat/Rough/Slope/Meta nào. Bundle nằm ở namespace riêng.
- **Không** đổi gait period (0.6 s), policy rate (50 Hz), common-PD
  gains/action scale/default pose. Đổi những thứ đó là contract khác.
- **Không** deploy lên robot.

---

## 10. Về tới máy dev thì chạy tiếp gì

1. `check_flat_plus.py --onnx DEPLOY/... --strict`
2. Parity PyTorch ↔ ONNX Runtime trên `golden_trace.txt`
3. Chép ONNX vào `unitree_mujoco/simulate_cpp/policy/locomotion/history/` →
   `FlatPlusController` sẽ tự được chọn theo cặp `(policy_contract, 332)`, và
   từ chối nếu thiếu bất kỳ khoá metadata nào
4. Gate 7: closed-loop so với `policy_goc_2` trên cùng ma trận lệnh
5. Chỉ sau đó mới tới preflight phần cứng
