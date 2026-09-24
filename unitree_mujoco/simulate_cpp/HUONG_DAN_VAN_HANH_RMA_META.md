# Lệnh chạy RMA meta-policy

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp

./run_sim.sh preflight-rma-meta

./run_sim.sh rma_meta flat
./run_sim.sh rma_meta 5
./run_sim.sh rma_meta 10
./run_sim.sh rma_meta 15
./run_sim.sh rma_meta 20
./run_sim.sh rma_meta 25
./run_sim.sh rma_meta 30

./run_sim.sh rma_meta 5-platform
./run_sim.sh rma_meta 10-platform
./run_sim.sh rma_meta 15-platform
./run_sim.sh rma_meta 20-platform
./run_sim.sh rma_meta 25-platform
./run_sim.sh rma_meta 30-platform

./run_sim.sh rma_meta mixed
```

# Chạy riêng từng expert với scene tự chọn

```bash
./run_sim.sh rma_expert flat <scene>
./run_sim.sh rma_expert slope_up <scene>
./run_sim.sh rma_expert slope_down <scene>

./run_sim.sh rma_expert flat flat
./run_sim.sh rma_expert slope_up 15-platform
./run_sim.sh rma_expert slope_down 15-platform
```

# Chạy thủ công: GUI mở mặc định

```bash
./run_sim.sh rma_meta mixed
./run_sim.sh rma_expert flat flat
```

# Chạy ma trận kiểm tra: tự động headless

```bash
./run_sim.sh rma_meta-smoke
./run_sim.sh rma_meta-full
```
