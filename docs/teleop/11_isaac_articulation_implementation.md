# Isaac articulation implementation disclosure

## Scope and requested behavior

The canonical Quest-to-Isaac workflow must construct the repository's R1 USD
without depending on the removed locomotion/training tree. This affects
`make teleop` simulation only and does not authorize or change hardware output.

## Implemented behavior and ownership

`teleop/r1/isaaclab_robot.py` is the sole active owner of `UNITREE_R1_CFG`.
`scripts/teleop/run_r1_quest3_live.py` imports it after Isaac Sim's
`AppLauncher` starts, then replaces only the prim path with `/World/Robot`.

The module resolves the USD from the current repository at `assets/R1/R1.usd`.
This direct resolution is **AI-selected** to remove the obsolete
`HAPPY_BABY_R1_ROOT`/training-tree dependency and prevent accidentally loading
the asset from another checkout. The module is deliberately not re-exported by
`teleop.r1.__init__`, so ordinary code-level tests do not import IsaacLab before
`AppLauncher`.

It preserves the articulation configuration removed from
`training/isaaclab/robot.py`: initial pose, actuator groups, effort/velocity
limits, stiffness, damping, armature, solver iterations, contact sensors and
the 0.9 soft joint-position limit factor. These values are **inherited**, not
newly selected. The existing CLI still disables self-collision by default for
the canonical baseline.

```text
run_r1_quest3_live.py
→ teleop.r1.isaaclab_robot.UNITREE_R1_CFG
→ assets/R1/R1.usd
→ IsaacLab Articulation
→ validated 12-joint upstream targets
```

If the module, IsaacLab dependency or USD is unavailable, startup fails; there
is no fallback robot. The launcher treats a process that returns zero without
creating its evidence directory as failed and reports the artifact gate as
skipped.

## Semantic regression and evidence compatibility

Observation, action, state, frame mapping, IK, control timing, metrics, evidence
schema and numerical configuration are unchanged. The physical boundary and
actuator values are unchanged because those values were copied without semantic
changes into their active capability owner. Existing simulation evidence remains
compatible; a new successful live run is still required to verify the repaired
workflow on the current environment. Hardware evidence is unaffected.

## Review and validation surface

- Articulation and actuator semantics: `teleop/r1/isaaclab_robot.py`.
- Import and provenance ownership: `_source_hashes` and `main` in
  `scripts/teleop/run_r1_quest3_live.py`.
- Incomplete-run exit behavior: `_effective_simulator_status` in
  `teleop/r1/launcher.py`.

Validation must distinguish code tests from a live Isaac workflow run. No
successful display, Quest session or scientific reproduction should be claimed
from unit tests alone.
