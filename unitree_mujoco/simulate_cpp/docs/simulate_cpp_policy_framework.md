# simulate_cpp locomotion framework

Phạm vi của framework là `simulate_cpp`; HB chưa được nối vào. Mọi route mới
đều opt-in, còn `single`, `rough`, `meta` và `rma_meta` giữ đường cũ.

```text
LowState/SportState
        |
  canonical FlatController::BuildBaseObservation83 (83)
        |
  +-----+------------------+--------------------+
  |     |                  |                    |
  H4    H5                 H4 + gait            Flat bundle + A-RMA
  332   415                335                  base view -> adapter -> latent -> actor
  |     |                  |                    |
  +-----+------------------+--------------------+
        common ComputeTargetQ -> common-PD -> motor command
```

## Single-model contracts

| Route | Controller | Input layout | Selection |
|---|---|---|---|
| legacy | `FlatController` | one raw 83-D frame | no contract, dim 83 |
| legacy rough | `RoughController` | 270-D height scan | no contract, dim 270 |
| H4 | `FlatPlusController` | term-major H4, oldest→newest | `flat_plus_h4_v1` + 332 |
| H5 | `FlatPlusH5Controller` | term-major H5, oldest→newest | `flat_plus_h5_v1` + 415 |
| gait | `FlatPlusGaitController` | H4 + current one-hot `stand,walk,w2s` | `flat_plus_gait_h4_v1` + 335 |

History append xảy ra đúng một lần ở 50 Hz, reset sẽ repeat frame đầu để khớp
training. Không dùng terrain truth hoặc contact estimator trong các route này.

## A-RMA attached to a Flat bundle

`ArmaController` không phải một policy selector độc lập. Mỗi thư mục bundle là
một Flat policy cụ thể kèm adapter và actor đã train cùng observation contract:

1. Xây frame 83-D.
2. `ArmaRuntime` giữ history K×83 theo time-major và chạy adapter.
3. Dựng đúng base view đã khai trong bundle rồi ghép latent.
4. Ghép actor input `83+latent` (91), `H4+latent` (340), `H5+latent` (423),
   hoặc `H4+gait+latent` (343).
5. Actor xuất 24 action; `ComputeTargetQ` và PD dùng contract actor.

`params/deploy.yaml` là registry duy nhất. Runtime yêu cầu tensor float32,
single input/output, dimensions khớp, metadata component/contract của export và
manifest bundle đối ứng; lỗi preflight là fail-closed. Với export hiện hành,
action scale/offset/PD của train run cũng phải được ghi trong registry vì actor
ONNX chỉ mang metadata component-local. Khi adapter lỗi hoặc latent quá cũ,
action cuối được giữ và route báo `SafetyHold`.

Để chọn bundle, chỉ cần đặt `locomotion_policy` trong `config/tuning.yaml` tới
thư mục có `params/deploy.yaml`, sau đó chạy `./run_sim.sh single flat`. Đường
dẫn tới file ONNX 83/270/332/335/415 vẫn đi qua các controller cũ. Cờ
`--arma-package DIR` và `./run_sim.sh arma flat` chỉ là alias tương thích.

## Lệnh vận hành

```bash
./run_sim.sh single flat       # file policy hoặc Flat+A-RMA bundle theo tuning.yaml
./build/arma_preflight DIR     # kiểm tra một bundle cụ thể trước khi mở simulator
./run_sim.sh arma flat         # alias legacy cho bundle mặc định
./run_sim.sh build
ctest --test-dir build --output-on-failure
```

Không đổi `config/tuning.yaml` sang H4/H5/gait hoặc bundle A-RMA cho tới khi
ONNX export đã có đủ metadata contract. A-RMA luôn đi cùng base policy đã khai
trong bundle; không dùng một actor A-RMA chung cho nhiều base policy khác nhau.
