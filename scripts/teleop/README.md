# R1 teleop entry points

All entry points here are simulation-only and have no hardware DDS dependency.
See `docs/operations/r1_quest3_teleop_sim.md` before using a live command bridge
or recording experiment evidence.

| Script | Environment | Role |
|---|---|---|
| `capture_quest_transport.py` | `tv` | T001-A: capture-only Quest transport data; emits no command. |
| `run_t001_b_pilot.py` | host Python 3 | **T001-B launcher.** Allocates one run id and starts both processes below. |
| `run_t007_upper_body_pilot.py` | host Python 3 | **T007 coupled upper-body launcher**, also reachable as `make teleop`. Same allocation, plus the schema-3 profile and dual-view video. |
| `quest_bridge.py` | `tv` | T001-B input bridge: Quest telemetry to `R1TeleopCommand` JSONL on stdout. |
| `run_r1_quest3_live.py` | `unitree_sim_env` | T001-B head-only, legacy T007 arm/head, or schema-3 T007 coupled upper-body simulation selected by mutually exclusive config flags. |
| `plot_r1_quest3_telemetry.py` | `r1_env` | Derives and plots head/left-wrist/right-wrist 3D position, velocity, and acceleration from a completed live run. |
| `run_r1_quest3_sim.py` | host Python 3 | Deterministic trace replay through the mapper with `FakeIsaacLabSink`. |
| `make_preflight_command_stream.py` | any | Synthetic command stream for plumbing preflight; **not** experiment evidence. |

## Why the live pilot is two processes

The Quest vendor wrapper lives in the `tv` environment (Python 3.10, `vuer`, no
IsaacLab) and IsaacLab lives in `unitree_sim_env` (Python 3.11, no `vuer`). They
cannot share one interpreter, so the bridge and the simulator run as separate
processes joined by a pipe of newline-delimited `R1TeleopCommand` JSON. Both
processes are on one host, so `time.monotonic()` is a shared `CLOCK_MONOTONIC`
timebase and command age is measured directly.

## Teleop experiment pipeline

The protocol sequence, its evidence requirements and its gates are owned by
[`experiments/r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md`](../../experiments/r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md):
`t001_a` capture, then `t001_b` live bridge into Isaac Sim, then `t002`–`t008`
simulation, with `t009`/`t010` reserved as provisional hardware protocols. The
T001-A/T001-B split is fixed by
[D001](../../decisions/r1_teleop/D001_teleop_stage_gates.md).

Ask the tool which protocols have produced evidence:

```bash
python3 scripts/experiments/r1_experiments.py protocols r1_teleop
```

## Run ids are allocated, not invented

A run id is `<protocol>_<UTC stamp>`, matching the locomotion convention. Both
runners refuse to overwrite an existing directory and refuse to start when their
stop file already exists, so a hand-typed id that has been used before fails
before the headset is involved. Let the tooling allocate one:

```bash
# T001-A: allocate a fresh t001_a_<UTC> directory under the experiment
conda run --no-capture-output -n tv python scripts/teleop/capture_quest_transport.py \
  --host-ip 10.42.0.1 --duration-s 120 --evidence \
  --cert-file ~/.config/xr_teleoperate/t001_10_42/cert.pem \
  --key-file ~/.config/xr_teleoperate/t001_10_42/key.pem

# T001-B: one id shared by the bridge and the simulator
python3 scripts/teleop/run_t001_b_pilot.py --host-ip 10.42.0.1 --duration-s 90

# T007 coupled whole-upper-body pilot: one command for the whole pipeline
make teleop HOST_IP=10.42.0.1
```

Both launchers share `teleop/r1/launcher.py`, so run-id allocation, the stop
file contract and the bridge-log staging behave identically.

Without `--evidence` the capture writes to a disposable `results/smoke/` path, so
a quick check never lands in the experiment by accident. `--output-dir` still
overrides both. Add `--dry-run` to the launcher to see the allocated paths and
the two commands without starting anything.

Create the same stop-file path from a second terminal to finish the live run
cleanly and preserve its evidence:

```bash
touch /tmp/<run-id>.stop
```

Both processes record `stop_file_requested`. `Ctrl+C` is also a graceful
`signal_SIGINT` stop; wait for the evidence-written message before closing the
terminal. A pre-existing stop file makes a new run refuse to start.

The operator must enter the immersive WebXR session in Quest Browser before
moving; a loaded page alone delivers no controller events. Hold the right
trigger as the deadman: releasing it, or removing the headset, must produce a
hold event rather than continued motion. The bridge accepts the digital trigger
flag or a valid TeleVuer analog value at/below `--trigger-value-threshold` on
the wrapper's inverted `10=released, 0=fully pressed` scale, and records every
effective transition with both raw representations in its connection JSONL.

## Known constraints

- `--disable-self-collisions` is required for the head joints to move at all.
  With the project asset config's self-collisions enabled, the R1 head joints
  are mechanically blocked and hold at zero regardless of the commanded target.
  This is a declared deviation and is recorded in each run's resolved config.
- Only the two head joints are driven **in the T001-B head-only pilot**. Arm and
  wrist targets are carried and recorded but withheld there. The T007 launcher
  drives the full coupled upper body.
- In T007, a wrist target outside the arm's reachable workspace is dispatched as
  a projected boundary solution, not exact tracking. Check
  `solver_solution_kind` and the recorded residual before reading a run as
  successful trajectory following.
- `SimulationApp.close()` has been observed not to return on this workstation.
  The runner writes every evidence file before closing and force-exits after a
  30 s shutdown grace period.
- Achieved control rate depends on GPU load; the runner records
  `achieved_control_hz` and `sim_to_wall_ratio` so a slow run is visible in the
  evidence rather than silently distorting it.

## Plot live Quest kinematics

After a completed live run, derive kinematics from its immutable command stream
into a new analysis directory:

```bash
MPLCONFIGDIR=/tmp/mpl conda run -n r1_env python scripts/teleop/plot_r1_quest3_telemetry.py \
  --input experiments/r1_teleop/quest3_sim_v1/T001/runs/t001_b_<run-id>/raw_commands.jsonl \
  --output-dir experiments/r1_teleop/quest3_sim_v1/T001/runs/t001_b_<run-id>/analysis/telemetry_kinematics
```

The PNG has position, velocity and acceleration rows, each with head, left
wrist and right wrist columns; x/y/z are separate curves. The CSV retains the
source timestamps, deadman state and segment ID. Derivatives are central finite
differences and are not calculated across gaps longer than 0.2 s, so reconnect
or dropped-command gaps appear as `NaN` rather than artificial spikes.
