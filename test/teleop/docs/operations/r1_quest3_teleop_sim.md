# R1 Quest 3 teleop (simulation-only)

R1 Quest teleop v1 is a simulation-only command path. It accepts normalized
Quest commands, maps headset-frame head/wrist poses and base velocity to an R1
simulator adapter, and fails closed on invalid, stale, or deadman-released
commands. It does not import Unitree DDS or `hardware/high_level/`.

## Boundary

- The current G1/Dex3 Quest runbook remains a legacy vendor-flow reference; it
  is not an R1 teleop procedure.
- `third_party/unitree_sim_isaaclab` is read-only reference material. Its
  current `sim_main.py` and DDS action providers expose G1 (`g129`) and H1
  (`h1_2`) paths, not an R1 task or command sink. Do not invoke it as an R1
  simulator by changing a robot-type flag.
- Its `xr_teleoperate` dependency is useful only as a reference for acquiring
  Quest/Vuer poses. A project-owned bridge must normalize those poses into the
  R1 schema, declare the frame/calibration, and be recorded in the experiment.
- A separate Quest bridge must convert headset data into newline-delimited
  `R1TeleopCommand` JSON. The schema is defined by `teleop/r1/schema.py`.
- Velocity is disabled in the default configuration. Enable it only in an
  experiment configuration after a matching IsaacLab R1 policy evaluation has
  recorded its task, R1 USD hash, and observation/action signature. When a
  config enables velocity, `--policy-manifest` is mandatory and must point to a
  promoted IsaacLab policy whose linked evaluation manifest has `status:
  "passed"`; the runner otherwise refuses to start.

T009 and T010 are provisional future hardware experiments after the T008
simulation gate. They have no executable command or hardware authority in this
document; each requires a separately approved hardware protocol and safety
record.

## Replay a deterministic trace

Run from the repository root:

```bash
python3 scripts/teleop/run_r1_quest3_sim.py \
  --input-trace experiments/r1_teleop/quest3_sim_v1/inputs/example_trace.jsonl \
  --dry-run
```

Use an owning-T output such as
`--output-dir experiments/r1_teleop/quest3_sim_v1/T001/runs/<run-id>` only for
a declared evidence run. Without it, output is disposable under `results/smoke/`.
An evidence replay stores the raw trace, resolved configuration, mapped
targets, metrics, status, provenance, and SHA-256 hashes of the teleop source
files. The runner refuses to overwrite its output directory.

For a deterministic watchdog test only, `--replay-receive-lag-s <seconds>`
adds a receipt lag to every timestamp in a trace. It is valid only with
`--input-trace`; use a lag greater than `command_timeout_s` to exercise the
stale-command hold path. It does not model a live Quest clock or simulator
latency.

The runner is a trace/safety adapter. Connecting it to a directly installed
IsaacLab R1 control environment requires an experiment-specific simulator sink
that respects the upper-body/lower-body ownership contract; it is not a
hardware bridge.

## Capture Quest transport only

To check Quest-to-host transport before any R1 command bridge exists, use the
capture-only endpoint from the repository root. It records vendor-wrapper
pose/controller telemetry but never maps it to `R1TeleopCommand`, calls a
simulator sink, or opens DDS/hardware communication.

```bash
conda run --no-capture-output -n tv python scripts/teleop/capture_quest_transport.py \
  --host-ip 10.42.0.1 \
  --cert-file /home/ubuntu22/.config/xr_teleoperate/t001_10_42/cert.pem \
  --key-file /home/ubuntu22/.config/xr_teleoperate/t001_10_42/key.pem \
  --output-dir experiments/r1_teleop/quest3_sim_v1/T001/runs/t001_a_<run-id>
```

Open the printed HTTPS URL in Quest Browser, accept the local certificate if
required, then move the headset/controllers. A completed run with zero
`motion_data_ready` samples is inconclusive, not proof of a connection.
`--no-capture-output` makes the endpoint URL visible immediately rather than
when the timed capture exits.

For a repeatable IP connection, create a separate certificate with a Subject
Alternative Name (SAN) for the current host IP; do not overwrite an existing
lab certificate. Keep the private key out of version control, then pass both
paths to `--cert-file` and `--key-file`. The capture endpoint remains
read-only with respect to the R1: it records transport telemetry only.

## T001-B live Quest-to-Isaac Sim pilot

Run this only after T001-A has captured valid live Quest state data. The two processes
use separate Python environments and communicate through JSONL; the bridge
writes commands only to its standard output while its connection evidence goes
to `--connection-log`. The `HB` hotspot is configured with fixed IPv4 gateway
`10.42.0.1/24`; Vuer uses fixed TCP port `8012`. The Quest certificate contains
that IP in its SAN.

T001-A and T001-B must remain split into evidence gates: capture data, apply the declared
trace to simulation, calibrate, synchronize, then evaluate mimic/IK. Do not
combine those gates into one operator command or treat an earlier stage as
evidence for a later one. The live bridge/simulator command below is therefore
only the T001-B simulation-application stage, not T001-A transport capture.

```bash
python3 scripts/teleop/run_t001_b_pilot.py \
  --host-ip 10.42.0.1 --duration-s 180 \
  --cert-file /home/ubuntu22/.config/xr_teleoperate/t001_10_42/cert.pem \
  --key-file /home/ubuntu22/.config/xr_teleoperate/t001_10_42/key.pem
```

The launcher allocates `T001/runs/t001_b_<UTC>/`, records the matching stop
file, and moves the staged bridge log into that immutable run on completion.

To stop a live run deliberately, use a second terminal and create the declared
stop file printed by the launcher:

```bash
touch <printed-stop-file>
```

Both processes then stop at their next loop boundary and finalize their logs.
`status.json` and the bridge log record `stop_file_requested`; do not delete the
file before inspecting the completed evidence. `Ctrl+C` is also handled as a
graceful `signal_SIGINT` stop, but wait for the evidence-written message before
closing the terminal. A stop-file path that already exists makes a new run
refuse to start, preventing accidental immediate stops.

This pilot drives only `head_yaw_joint` and `head_pitch_joint`; arm/wrist
targets are logged but withheld, pelvis is pinned, and base velocity is an
invariant violation. The bridge rejects non-finite, non-orthonormal, and
improper-reflection pose matrices before emitting a command. Head tracking
metrics use the observed state after the physics steps for that control cycle,
not the state immediately before a target is applied.

`--disable-self-collisions` is a declared simulator workaround for the current
R1 head/cervical collision issue. It is not a model fix and must remain visible
in the resolved configuration when interpreting T001-B results.

## T007 coupled upper-body simulation

Run the whole pipeline with one command from the repository root:

```bash
make teleop HOST_IP=10.42.0.1
```

`make teleop-dry-run` prints the allocated run paths and both underlying
commands without starting anything. The launcher is
`scripts/teleop/run_t007_upper_body_pilot.py`; it allocates the run id, stop
file and evidence directory exactly as the T001-B launcher does.

The underlying `--whole-upper-body-config` selects the schema-3 coupled solver.
It owns waist yaw, both arms, and head in one atomic target while fixing the
root and legs. It is mutually exclusive with the legacy `--arm-head-config`.

Two options change what a run means and must be read from its resolved config:

- `allow_projected_position_solution` (on by default in the T007 profile)
  dispatches the closest reachable joint solution for a wrist target outside the
  arm's workspace, marked `solver_solution_kind: "projected"` with its residual.
  It is a bounded best effort, not tracking evidence. With it off, such a target
  holds every controlled joint instead.
- `control_waist_roll` adds torso lean as a 14th joint. It is a declared
  simulation-only deviation from the real R1-A5 motor interface and is off by
  default.

`--dual-view` (default in the launcher; disable with `--single-view`) records a
second mirrored evidence camera and stores both side views in one synchronized
video, so an arm occluded in one view stays visible in the other.

Method limitations are recorded in
`experiments/r1_teleop/quest3_sim_v1/T007/T007.md` and
`docs/teleop/r1_upper_body_ik.md`. This remains a simulation-only path and does
not authorize DDS or real-robot output.

## Plot head and wrist transport kinematics

After the live run has completed, create a separate derived-data directory from
its immutable `raw_commands.jsonl`:

```bash
MPLCONFIGDIR=/tmp/mpl conda run -n r1_env python scripts/teleop/plot_r1_quest3_telemetry.py \
  --input experiments/r1_teleop/quest3_sim_v1/T001/runs/t001_b_<run-id>/raw_commands.jsonl \
  --output-dir experiments/r1_teleop/quest3_sim_v1/T001/runs/t001_b_<run-id>/analysis/telemetry_kinematics
```

The output PNG has position (m), velocity (m/s), and acceleration (m/s²) rows,
with head, left wrist and right wrist columns; each panel has x/y/z traces. The
derived CSV and summary preserve source provenance. Central finite differences
are applied only inside command segments separated by no more than 0.2 s;
derivatives in shorter post-reconnect segments remain `NaN` rather than
misrepresenting a transport gap as motion.
