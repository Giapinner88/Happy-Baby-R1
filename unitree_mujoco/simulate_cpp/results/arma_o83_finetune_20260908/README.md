# ARMA o83 — finetune seed42, 2026-09-08

Gói kết quả run `Unitree-R1-Flat-Meta-ARMA` (phase `rma`), train xong 20000/20001
iteration lúc **2026-09-08 23:20:51**.

## Nội dung

| Đường dẫn | Là gì |
|---|---|
| `exported/actor.onnx` | actor để deploy (861 KB) |
| `exported/adaptation.onnx` | adapter module (469 KB) |
| `exported/arma_bundle.json` | hợp đồng ghép cặp, khoá chéo sha256 giữa actor và adapter |
| `checkpoint/model_20000.pt` | checkpoint cuối (5.8 MB) — dùng để train tiếp |
| `checkpoint/adaptation.pt` | adapter đầu vào của run này (470 KB) |
| `params/{agent,env,train}.yaml` | cấu hình đúng lúc chạy |
| `docs/05_arma.md` | tài liệu thuật toán ARMA |

## Run gốc

```
repo   : ~/mujoco_mjlab/unitree_rl_mjlab_meta
run dir: logs/rsl_rl/r1_flat_plus_arma_o83/2026-09-08_14-03-21_arma_o83_finetune_seed42
task   : Unitree-R1-Flat-Meta-ARMA
lenh   : scripts/train.py Unitree-R1-Flat-Meta-ARMA --gpu-ids [0] \
           --env.scene.num-envs 4096 --agent.max-iterations 20001 --agent.seed 42 \
           --agent.run-name arma_o83_finetune_seed42 --agent.upload-model False \
           --agent.actor.phase rma \
           --agent.actor.adapter-checkpoint checkpoints/r1_flat_plus_arma/o83_k50_z8/adaptation.pt \
           --warm-start-checkpoint logs/rsl_rl/r1_flat_plus_arma_o83/2026-09-08_00-10-27_arma_o83_teacher_seed42/model_20000.pt
thoi gian: 14:03:21 -> 23:20:51 (8h17), 4096 env, ~0.38 iter/s
```

Warm-start từ teacher `2026-09-08_00-10-27_arma_o83_teacher_seed42/model_20000.pt`.

## KHÔNG có trong gói

- 201 file `model_*.pt` trung gian — vẫn nằm ở run dir gốc
- `events.out.tfevents...` (43.5 MB TensorBoard) — vẫn ở run dir gốc
- `checkpoints/r1_flat_plus_arma/o83_k50_z8/adaptation_dataset.pt` (**6.4 GB**) — cố ý bỏ

## Kiểm tra tính toàn vẹn

```bash
cd <thu muc giai nen> && sha256sum -c SHA256SUMS
```

## Đã bổ sung vào `simulate_cpp` — 2026-09-08

Run này đã được staging thành bundle opt-in:

`policy/locomotion/arma/flat_o83_k50_z8_v1/`

- `arma_preflight`: **PASS** — adapter history 50×83, latent 8, actor 91-D.
- runtime contract test: **PASS** — inference finite và NaN input chuyển sang
  `SafetyHold`.
- closed-loop MuJoCo flat, headless, `vx=0.35 m/s`, warmup 2 s + đo 8 s:
  **PASS**, tilt cực đại 2.6754°, hạ cao 0.004778 m, 400 mẫu, không ngã.

Bundle này không được tự động đặt vào `config/tuning.yaml`; các policy cũ và
`HB/high_level_2/config/locomotion.yaml` vẫn giữ nguyên. Chưa có hardware
acceptance.
