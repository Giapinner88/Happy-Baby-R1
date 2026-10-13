# Happy Baby R1 Project Map

This map names the active owners for the workflows maintained in this
workspace. It intentionally does not prescribe a repository-wide rename.

## Active workflows

| Workflow | Canonical implementation | Entry point | Evidence |
| --- | --- | --- | --- |
| R1 MJLab locomotion | `training/mjlab/` | `scripts/training/r1_policy_workspace.py train mjlab` | `experiments/r1_locomotion/mjlab/` |
| R1 IsaacLab locomotion | `training/isaaclab/` | `scripts/training/r1_policy_workspace.py train rl_lab` | `experiments/r1_locomotion/isaaclab/` |
| R1 Quest 3 teleop simulation | `teleop/r1/` | `scripts/teleop/run_r1_quest3_sim.py` | `experiments/r1_teleop/quest3_sim_v1/` |
| R1 Quest 3 suspended hardware teleop | `hardware/high_level/` (sole motor owner) + `hardware/teleop/` (sidecar) | `make teleop-hardware` | `results/smoke/` + robot `~/HB/teleop/logs/` |
| Experiment records and reporting | `evidence/` | `scripts/experiments/r1_experiments.py` | `experiments/registry.json` |

`evidence/` owns the run contract that every experiment records against: which
files a run directory must contain, what the status vocabulary means, how to
read a run written before the contract existed, and how to report across
experiments. `experiments/registry.json` is the authoritative list of
experiments; tooling reads it rather than scanning directories. The contract
itself is documented in `experiments/README.md`.

`training/` is the project-owned overlay imported by the two upstream training
frameworks. `third_party/` remains read-only vendor code. `data/runs/` is
intentionally empty pending reviewed standard data; new evidence-producing
training runs are created beneath the experiment that owns the protocol.

The active runtime environments are independent: `r1_env` for MJLab,
`unitree_sim_env` for directly installed IsaacLab/Unitree RL Lab, and `tv` on
the host for Quest input. No active R1 workflow runs IsaacLab through Docker.

## Boundaries

- `hardware/high_level/` is the sole owner of embedded `rt/lowcmd`. R1 teleop
  code must not import it or create a DDS motor publisher; it may send the
  documented loopback UDP target schema to high-level's ZERO-TORQUE-only arbitration.
- `src/` remains the ROS 2/colcon workspace. `sim/` remains project-owned
  simulation/runtime support. Neither is a source for the R1 teleop v1 command
  path.
- `ai_modules/`, ROS hardware integration, legacy G1/Dex3 teleop, and the
  MuJoCo policy bridge are not changed by this work.
- `.agents/` contains agent guidance. It is not a project implementation
  directory.

## Evidence and promotion

Each framework owns its own checkpoints and model signatures. `collect` stores
an artifact under its owning experiment; it is not an approved runtime model.
Promotion requires a matching framework/task/asset evaluation manifest and
creates a new destination in `data/policies/` or `data/models/`; it never
overwrites an existing policy or model.
