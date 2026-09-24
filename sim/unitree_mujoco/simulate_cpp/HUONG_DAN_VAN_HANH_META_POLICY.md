# Lệnh chạy meta_policy

```bash
cd /home/khanh248/Documents/HB/Mujoco/unitree_mujoco/simulate_cpp

./run_sim.sh preflight-meta-policy

./run_sim.sh meta_policy flat
./run_sim.sh meta_policy 5
./run_sim.sh meta_policy 10
./run_sim.sh meta_policy 15
./run_sim.sh meta_policy 20
./run_sim.sh meta_policy 25
./run_sim.sh meta_policy 30

./run_sim.sh meta_policy 5-platform
./run_sim.sh meta_policy 10-platform
./run_sim.sh meta_policy 15-platform
./run_sim.sh meta_policy 20-platform
./run_sim.sh meta_policy 25-platform
./run_sim.sh meta_policy 30-platform

./run_sim.sh meta_policy mixed
```

# Chạy từng expert, tự chọn scene

```bash
./run_sim.sh expert flat <scene>
./run_sim.sh expert slope_up <scene>
./run_sim.sh expert slope_down <scene>
```

```bash
./run_sim.sh expert flat flat
./run_sim.sh expert flat mixed

./run_sim.sh expert slope_up 5
./run_sim.sh expert slope_up 15-platform
./run_sim.sh expert slope_up mixed

./run_sim.sh expert slope_down 5
./run_sim.sh expert slope_down 15-platform
./run_sim.sh expert slope_down mixed
```

```text
<scene> = flat | 5 | 10 | 15 | 20 | 25 | 30 |
          5-platform | 10-platform | 15-platform |
          20-platform | 25-platform | 30-platform | mixed
```

# Chạy ma trận kiểm tra

```bash
./run_sim.sh meta_policy-smoke
./run_sim.sh meta_policy-full
```

```text
W/S: tiến/lùi
A/D: trái/phải
Q/E: quay trái/quay phải
Tab: chậm/nhanh
Ctrl+C: dừng
```
