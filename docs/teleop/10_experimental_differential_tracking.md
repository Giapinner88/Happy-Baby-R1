# 10 — Experimental Differential Tracking

## 1. Status

```text
archived simulation-only experiment; no active launcher; no hardware claim
```

The active baseline uses the upstream vendor solver. This document preserves
the differential implementation's historical method and evidence semantics;
`r1_t007_differential_live.json` is not selectable from the current launcher.

## 2. Controller

`teleop/r1/differential_tracking.py` computes a 15-D task command from wrist
position/orientation and head-orientation errors, optionally adding target
velocity estimated from the pose sequence. It solves damped least squares with
box constraints for joint velocity, acceleration and joint position, then
integrates a bounded joint reference.

The `position_then_orientation` mode solves wrist position plus head first and
uses remaining null space for infeasible 5-DoF wrist orientation. This is not
the upstream Unitree CasADi/IPOPT solver.

## 3. Valid evidence

A differential run must record:

- the exact source pose sequence and timestamps;
- target velocity reconstruction/filtering;
- $e_p$, $e_R$ and $e_v$;
- controller compute time and achieved rate;
- velocity, acceleration, reference-lead and joint-limit activation;
- raw versus workspace-projected targets;
- controller type, task priority, body mode and controlled joint order.

It is not comparable with schema-2 independent-arm or schema-3 iterative
pose-IK evidence. A favorable replay does not replace a new live Quest run and
does not authorize robot actuation.
## 4. Solution multiplicity and temporal branch selection

A wrist **full-pose** IK request may have zero, one, or several isolated
solutions after joint limits and the five-DoF R1 arm are considered. A
**position-only** request is generically redundant: away from singularities,
its feasible set is a posture manifold rather than one joint vector, and joint
limits can split that manifold into disconnected components. Therefore a
per-frame minimum-residual solution is not a complete teleoperation policy.

The controller must carry a branch state through time. Wrist position is the
primary task; continuity from the previous accepted posture and a declared
robot-posture cost choose a point on the feasible manifold; wrist orientation
is lower priority for this underactuated arm. Multi-start candidate search is
permitted at a session boundary or in an offline trajectory pass, but a live
candidate must not replace the active solution merely because its instantaneous
Cartesian residual is smaller. It must also satisfy the declared joint-jump,
velocity and acceleration bounds. A necessary branch change is an explicit
homotopy/transition event, not an implicit solver restart.

For recorded-data validation, score a whole continuous joint path, including
Cartesian error, posture cost, `q[k]-q[k-1]`, acceleration and limit margins.
Selecting independent best frames can splice different discrete full-pose
solutions or disconnected position-manifold components into a physically
invalid trajectory.

## 5. Offline whole-trace continuation (archived)

The removed `teleop/r1/offline_continuation.py` implemented the recorded-trace
variant for T007. It selected an anchor at maximum bilateral hand separation, retained a
Pareto set of multi-start solutions, continues each branch backward and
forward, and refines interior samples with both temporal neighbours. Hard joint
step, velocity, acceleration and position limits are applied before replay.

The arms are not copied or coupled. Left and right are solved from their own
Quest wrist targets. Their only anatomical convention is side-aware: the
signed elbow pole is measured after projecting the elbow off the
shoulder-to-endpoint line, with `+y` outward for the left arm and `-y` outward
for the right. A hinge residual penalizes an inward pole independently on each
arm. This corrects the earlier failure in which both arms inherited the same
world-`y` posture prior, making the right elbow fold toward the torso even
though the wrist targets were distinct.

The executable and editable config were removed when this branch became
upstream-only. Git history preserves the exact workflow used by the historical
run; there is no active command for regenerating it in this workspace.

The historical Isaac replay dispatched the precomputed vector by the original command
sequence id. It does not invoke IK online. Consequently the reported lookup
time measures replay overhead, while the offline solver wall time is reported
separately in the continuation run.

### 5.1 Vendor parity and intentional deviations

Recorded wrist poses from `QuestCommandBridge` are already the processed
`xr_teleoperate` TeleVuer output: Unitree wrist convention in the
head-yaw-relative IK waist workspace. Offline continuation must consume these
absolute poses directly. It must not anchor them a second time to robot
neutral. Each wrist position is projected onto the conservative URDF-derived
reach sphere before solving; raw and projected waist-frame targets are both
stored in `offline_joint_trajectory.npz`.

The upstream R1-A5 reference uses a 0.20 m virtual endpoint, URDF joint bounds,
IPOPT warm start, 30 iterations, cost
`50||e_p||² + 0.5||e_R||² + 0.02||q||² + 0.1||q-q_last||²`, then FIR weights
`[0.4,0.3,0.2,0.1]`. It does not contain an elbow-pole or bilateral-symmetry
constraint. The project records every deviation explicitly. The accepted
offline pose method is a position-manifold study: orientation has zero solve
weight, posture and the natural elbow sector select a branch, and a whole-trace
neighbor reprojection can reopen a locally trapped branch. It is not presented
as an unchanged upstream solver.

Every offline solve now audits this contract directly from the vendored
`R1_A5_ArmIK` source and records its SHA-256. A change to direct wrist-pose
use, virtual endpoint, objective weights, IPOPT iteration limit or FIR weights
fails before a new evidence run is produced. This guard detects upstream drift;
it does not conceal the intentional project deviations listed above.

### 5.2 Natural sector and mirror-equivariant right solve

For a bent arm, the declared natural elbow pole is backward (`-x`), outward
(`+y` left, `-y` right) and down (`-z`). Direction is evaluated only when pole
norm is at least 10 mm and straightness is at most 165°, because pole direction
is ill-conditioned near a straight arm. A 5 mm violation tolerance separates
visible geometry from sub-millimetre numerical sign noise.

Both arms remain independent. To prevent numerical multistart ordering from
breaking mirror equivariance, the right-only target/history may be reflected
into a left-canonical problem and mapped back with R1-A5 joint signs
`[1,-1,-1,1,-1]`. No left target or left joint path enters this computation.
The full asset differs in mirrored shoulder-roll limits by 9e-5 rad due to
rounding; the discrepancy is recorded and the final full-model limiter applies
the actual right limit.

The 2026-08-22 v21 Isaac replay is the current simulation evidence. It passes
visual pose order, p95 wrist tracking and PhysX natural-sector fractions, but
retains 0.23–0.30 s peak-error events during the fastest overhead-to-lateral
transition. Therefore it is not a hardware gate pass.
