# R1 MuJoCo C++ policy runner

This directory contains the C++ policy process used with the Unitree MuJoCo R1
simulator. It supports the existing single-policy workflow and the recurrent
`slope_meta` package without changing `config/tuning.yaml` during a meta run.

## Quick commands

Run the two-expert slope meta-policy on the mixed scene:

```bash
./run_sim.sh meta mixed
```

Run it on a specific slope or ramp/platform scene:

```bash
./run_sim.sh meta 15
./run_sim.sh meta 30-platform
```

Run the single policy selected by `config/tuning.yaml` exactly as before:

```bash
./run_sim.sh single flat
./run_sim.sh single mixed
```

Validate the build, all policy/scene contracts, package checksums and the 50 Hz
latency budget:

```bash
./run_sim.sh preflight
```

Preflight also discovers every ONNX below `policy/locomotion` outside the meta
package and performs a zero-observation load/inference smoke test, so newly
added or retained single policies are checked automatically.

List supported modes/scenes or build only:

```bash
./run_sim.sh list
./run_sim.sh build
```

The old entry points `run_flat_stack.sh`, `run_rough_stack.sh`,
`run_slope_sim.sh` and `run_mixed_terrain_sim.sh` remain available.

Mimic transition smoke test (headless MuJoCo, test-only dance trigger):

```bash
MUJOCO_HEADLESS=1 timeout --signal=INT --kill-after=10s 45s \
  ./run_rough_stack.sh default --dance-test=2
```

The log must show locomotion -> default pose -> dance warmup -> dance finished
-> cooldown -> locomotion. A `bounded fallback` handover is reported when the
strict q/dq gate is not reached but the post-timeout pose remains inside the
configured fallback bounds; it is not the same as unconditional max-time handover.

## Source layout

```text
src/
├── app/                    thin main and policy application orchestration
├── controllers/
│   ├── locomotion/         Flat, FlatPlus-history and Rough controllers
│   ├── motion/             dance and sit controllers
│   └── slope_meta/         gate, selector, expert adapter and meta controller
├── input/                  keyboard/teleoperation support
├── io/                     data readers
├── motion/                 gesture, audio and motion primitives
├── onnx/                   RAII ONNX session and metadata contract loader
├── runtime/                shared policy interface, tuning and R1 contracts
└── simulator/              scene discovery and MuJoCo ray filtering

tools/
├── r1_joint_tester/        standalone joint-test utility moved out of src
├── run_r1_policy.cpp       legacy auxiliary policy executable
└── slope_meta_preflight.cpp contract and 50 Hz latency benchmark

tests/
├── unit/                   deterministic pure-logic tests
├── contracts/              ONNX/package contract tests
├── scenes/                 flat, slope, platform and mixed-scene tests
└── integration/            application behavior tests

policy/locomotion/
├── flat/                   unchanged single-policy artifacts
├── rough/                  unchanged single-policy artifacts
├── slope/                  unchanged up/down expert artifacts
└── meta/slope_meta_v6/     self-contained gate + two experts + registry
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the runtime signal flow,
safety behavior and compatibility boundaries.

Hướng dẫn vận hành từng bước bằng tiếng Việt:
[HUONG_DAN_VAN_HANH_SLOPE_META.md](HUONG_DAN_VAN_HANH_SLOPE_META.md).

## Scene meaning

- `flat`/`0`: flat reference.
- `5` through `30`: trained slope range and primary slope-meta acceptance set.
- `35` and `40`: out-of-training-range robustness tests, not acceptance proof.
- `15-platform`, `30-platform`: flat spawn, ascending ramp, plateau and descending
  ramp in one scene.
- `mixed`: flat spawn plus 5/15/30 degree lanes, stairs, deterministic rough
  tiles and obstacles. The slope lanes are the policy acceptance portions;
  stairs/rough/obstacles are stress tests because the 83-D actor has no height
  scan.

## Safe operating notes

`slope_meta` starts in `STARTUP` for one second while its fixed 50-frame history
fills. Commands are locked at zero, while the two-expert neutral blend actively
balances the robot and its output is cross-faded in. After startup, commands are
clamped to the training range and ramped. Low confidence uses command-preserving
`NEUTRAL`; only invalid observations or non-finite inference enter fail-closed
`SAFETY_HOLD`. A continuous two-second invalid-data hold latches the controller
until policy reset. This is simulator validation, not real-robot acceptance.
