# R1 Unitree MuJoCo Policy Runtime

This directory is the local Happy-Baby-R1 runtime for one policy runner:

- `config.py` selects the R1 policy and defines joint order, default pose, gains,
  action scale, scene, and ONNX path.
- `policy_runner.py` is the only Python policy process.
- `unitree_mujoco2.py` runs the local R1 MuJoCo bridge.
- `unitree_sdk2py_bridge.py` connects MuJoCo state/control to Unitree DDS.
- `state_logger.py` writes runtime CSV logs to `data/sim_state_logs/`.

Do not put policy outputs or logs in this source directory. Use:

- `data/models/unitree_mujoco_policy/` for deployable ONNX policies.
- `data/runs/unitree_mujoco_policy/` for launcher logs.
- `data/sim_state_logs/` for per-step CSV state logs.

The default config expects:

```text
data/models/unitree_mujoco_policy/r1_velocity.onnx
```

To switch policy, edit `POLICY_NAME` and `POLICIES` in `config.py`. The launcher
does not choose between old scripts anymore.

Training and runtime use the same local R1 MJCF:

```text
asset/mujoco/unitree_robots/r1/R1.xml
```

Refresh it from the MJLab training source with:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/sync_r1_mujoco_asset.py
```

Run from repo root:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --interface lo \
  --domain-id 1
```

The default launcher starts the policy directly, matching the MJLab deploy
configuration: no FixStand warmup, no action fade, and no raw action clip.
`DEFAULT_Q` alone is not a stable standing controller for this R1 asset.

Useful runtime controls:

```bash
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/run_unitree_mujoco_policy.py \
  --duration 20 \
  --policy-target-rate-limit 2.0 \
  --interface lo \
  --domain-id 1
```

Add `--policy-window` to show the pygame keyboard/gamepad control window. Add
`--viewer` only when the desktop/OpenGL session is ready for MuJoCo viewer.
