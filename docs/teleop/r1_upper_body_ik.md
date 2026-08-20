# Coupled R1-A5 upper-body inverse kinematics

Status: simulation pilot method, not accepted for robot actuation  
Implementation: `teleop/r1/upper_body_kinematics.py`,
`teleop/r1/upper_body_ik.py`, and `teleop/r1/whole_upper_body.py`

## Purpose and system boundary

This method maps one Quest head orientation and two wrist poses to one coupled
R1-A5 upper-body target. It is intended to test whether allowing torso yaw to
participate improves bilateral reach without moving the legs or floating base.
It replaces neither the locomotion controller nor a hardware safety layer.

## Body modes

Which torso joints the solver owns is selected by a declared body mode, either
in the experiment profile (`body_mode`) or at the command line (`--body-mode`,
`make teleop BODY_MODE=...`), which overrides it and is recorded in the
resolved configuration:

| Mode | DoF | Torso |
|---|---|---|
| `arms_head` | 12 | waist yaw and roll both held fixed |
| `waist_yaw` | 13 | waist yaw controlled; the hardware-common R1-A5 set |
| `full_upper_body` | 14 | adds waist roll; simulation-only deviation |

`arms_head` exists because a coupled least-squares solver will recruit any
available degree of freedom to reduce a position error it cannot eliminate.
When the operator's hands are outside the arm's reachable set the waist is
recruited to chase them, which reads as an unnatural torso twist. Freezing the
torso removes it from the task entirely rather than relying on the posture bias
to resist, which it cannot: the bias is null-space projected, so it has almost
no authority when the task is unreachable.

`full_upper_body` is not hardware-comparable: the R1-A5 controller marks the
waist-roll motor slot unused, and the pitch/yaw-only head task cannot represent
a rolled head. `nominal_joint_position_rad` must match the mode's length;
`retarget_nominal` converts a declared nominal between modes and preserves the
torso pose, so `--body-mode` does not require editing the profile.

The controlled vector for `waist_yaw`, in its authoritative order, is

```text
q = [waist_yaw,
     left shoulder pitch/roll/yaw, left elbow, left wrist roll,
     right shoulder pitch/roll/yaw, right elbow, right wrist roll,
     head_pitch, head_yaw]                         (13 rad values)
```

The head target is built from the head chain's own forward kinematics
(`R1A5UpperBodyModel.head_rotation`), so a commanded pitch/yaw pair is exactly
realizable by the two head joints. It must not be written as a conventional
`Rz(yaw) @ Ry(pitch)`: the R1 chain applies `head_pitch` before `head_yaw`, and
the two orders differ whenever both angles are non-zero.

All targets and FK outputs are expressed in `pelvis_link`. Quest wrist poses
enter the live sink in the declared neutral `waist_yaw_link` frame and are
transformed into `pelvis_link` using the selected URDF's neutral pelvis-to-waist
transform. Length is metres, angle is radians, and time is seconds.

The simulation root and all leg joints are fixed. `waist_roll_joint` is fixed by
default: the official R1-A5 reference URDF has no waist-roll joint and its motor
interface marks the corresponding slot unused. Fingers and wrist pitch/yaw are
outside this model.

### Optional 14-DoF torso-lean variant

`control_waist_roll: true` in the experiment configuration prepends
`waist_roll_joint` to the controlled vector, giving

```text
q = [waist_roll, waist_yaw, left arm[5], right arm[5], head_pitch, head_yaw]
                                                         (14 rad values)
```

This is a **declared simulation-only deviation**, not a hardware-comparable
mode. It exists to test whether torso lean improves bilateral reach. Three
consequences must stay visible when interpreting such a run:

- The R1-A5 motor interface marks the waist-roll slot unused, so a 14-DoF
  solution cannot be commanded on the real robot as-is.
- It requires an asset that has the joint. The pinned vendor R1-A5 URDF does
  not, and the loader raises rather than silently substituting a fixed joint.
- The head task is a yaw/pitch-only source rotation and therefore cannot
  represent a rolled head; with waist roll active the head orientation residual
  is expected to be non-zero and must not be read as a solver fault.

A 14-DoF run is incompatible with 13-DoF evidence and must be recorded and
compared separately.

## Model and assumptions

The reusable loader accepts an explicit URDF. The current simulation pilot uses
`assets/R1.urdf`; compatibility tests also load the pinned vendor reference
`third_party/xr_teleoperate_v1_6/assets/r1/r1_a5.urdf`. Relative arm geometry
matches, but their pelvis-to-waist transforms do not. The loader preserves that
difference instead of silently treating the assets as identical.

Each arm has five physical joints and a virtual end effector 0.20 m along the
wrist-roll link's local +x direction. Consequently, two arbitrary independent
6-DoF wrist poses are generally not exactly realizable. Wrist position is a
primary task; wrist orientation is a weighted best fit whose residual remains
in the evidence. Head roll is not commanded.

This method assumes rigid transforms from the URDF, a fixed base, and fresh,
calibrated Quest poses. It does not model collision avoidance, self-collision,
balance, actuator dynamics, latency compensation, or hardware backlash.

## Forward model and solver

For each URDF joint `j`, the transform is

```text
T_j(q_j) = T_origin,j · Rot(axis_j, q_j)
```

and transforms are multiplied along the pelvis-to-endpoint chain. The task
error stacks left/right position error, left/right rotation-vector error, and
head rotation-vector error:

```text
e(q) = [pL* - pL(q), pR* - pR(q),
        Log(RL* RL(q)^T), Log(RR* RR(q)^T),
        Log(RH* RH(q)^T)]                         (15 values)
```

The 15-by-13 error Jacobian is evaluated by central finite differences using
the configured perturbation. Each iteration solves a damped weighted
least-squares step, adds a null-space bias toward the declared nominal posture,
limits the total step norm, and clamps every joint to the selected URDF limit.
The exact numerical weights, task tolerances, damping, finite-difference step,
and iteration cap belong to the resolved experiment configuration; the
reusable solver supplies no hidden defaults. The nominal posture is a secondary
cost, not an acceptance condition: a target is complete when its declared
physical task tolerances pass.

### Targets with no exact solution

A Cartesian target outside the bounded joint workspace has no exact root. The
solver measures progress as a *relative* decrease of the weighted task score;
an absolute epsilon is unusable because that score is O(10³) for a far target
and asymptotic crawl toward an unreachable point would register as genuine
progress forever. When the relative decrease stalls for a fixed number of
iterations, the closest iterate found so far is returned with status
`projected_to_reachable_boundary`. A target still improving when the iteration
cap is reached returns the same closest iterate with status
`iteration_budget_exhausted`. Both remain non-converged.

Whether such a result is dispatched is the caller's declared policy, not the
solver's. `allow_projected_position_solution` dispatches the closest reachable
iterate and records `solver_solution_kind: "projected"` with its residual;
without it, the coupled sink holds every controlled joint. Holding was the only
behaviour before 2026-08-18, which is why an operator reaching beyond the arm's
workspace saw the arms stop extending and the whole upper body stop following
the commanded trajectory rather than extending to their limit. `singular_system`
and other genuine solver failures are never dispatched through this policy.

A projected target is a bounded best effort, never evidence of tracking: its
position residual is the distance between what was asked for and what the joint
limits physically allow.

### Rate limiter and joint limits

The online rate limiter is second order. When a target reverses direction the
acceleration limit cannot cancel the stored velocity in one step, so the emitted
command continues briefly in the old direction and can leave the joint range
even though every solver output was inside it. Replaying the 2026-08-18 run
showed a worst case of 0.133 rad (7.6°) past `left_shoulder_roll_joint`'s lower
limit. The coupled sink therefore gives the limiter the model's joint limits;
the limiter clamps its output and zeroes the velocity of any clamped joint so it
does not keep integrating into the limit. After this change the same replay
dispatches no command outside any joint limit.

This affected the coupled sink because it is the path being revised. The legacy
per-arm sink constructs its limiter without limits and retains the same latent
behaviour; changing it would alter schema-2 comparability and has not been done
here.

A live target is dispatched atomically. Otherwise every controlled joint holds
the last complete target. Reset returns all controlled joints to the declared
nominal pose and requires a new session. The evidence records solver status,
iteration count, five task residuals, limit margin, clamped joints,
pre-rate-limit solution, rate-limited target, and post-physics joint state.

## Seed restart

Successive solves are seeded from the previous solution, which is what keeps the
commanded trajectory continuous. That seed is also a trap: drawing the hands in
towards the body reaches a folded posture whose basin an extended previous
solution cannot cross, and a damped-least-squares step cannot climb out of it.
Iteration budget, step size, damping and task weights do not change the outcome;
only the seed does.

When a solve stalls with a position residual above `seed_restart_residual_m`
and both wrist targets lie inside the arms' reach bound, the solver re-solves
once from the declared nominal and keeps the better result. The dispatched
record carries `seed_restarted_from_nominal` so a run shows where this happened.

The reach bound is `ArmChain.max_reach_from_shoulder_m`: the triangle inequality
over the link translations below the shoulder plus the tool offset. It is
derived from the asset, not declared, because it is a property of the geometry;
it is conservative, so a target beyond it is definitely unreachable. Its role is
to distinguish "the solver is stuck" from "the operator reached past the arm",
and thereby to keep the extra solve off targets where a large residual is
honest.

## Checks executed

`tests/teleop/test_r1_upper_body_ik.py` covers both URDF profiles, transform and
limit loading, the asset-frame mismatch, SO(3) edge cases, finite-difference
refinement, waist coupling to both wrists and head, FK-to-IK round trips,
unreachable-target nonconvergence, boundary projection and its joint-limit
compliance, the 14-DoF waist-roll variant and its asset/ownership constraints,
joint limits, atomic dispatch, hold, reset, and ownership. The legacy arm/head
regression remains separate.

These checks establish internal kinematic and software consistency only. No
Quest/Isaac live run under schema 3 and no robot measurement has yet validated
tracking, latency, collision clearance, torque, or safe hardware limits.

### Offline replay of the 2026-08-18 refused targets

The 820 enabled targets recorded in
`experiments/r1_teleop/quest3_sim_v1/T007/runs/t007_whole_upper_body_20260818T050419Z`
were replayed through the revised solver on the workstation. Under the previous
converged-only policy 591/820 (72.1%) were dispatched; under the projection
policy all 820 are dispatched. Of those, 491 converge exactly (position residual
mean 0.7 mm) and 329 are projected, with position residual mean 366–429 mm.

That residual is the diagnostic, not a defect: the recorded Quest wrist targets
reach up to 0.954 m from the pelvis while a limit-corner sample of the arm chain
puts maximum reach near 0.750 m. The operator's absolute 1:1 hand pose is
frequently outside the R1 arm's physical workspace, so the projected solution is
the arm extended to its boundary toward an unreachable point.

This replay is a workstation offline check on immutable recorded input. It is
not a live run, and its wall-clock timings are not comparable with the live
`unitree_sim_env` environment. Mean solve time rose from 63.6 ms to 77.3 ms per
target in this harness while p95 and maximum were unchanged (≈208 ms and
≈214 ms), so a new live run must confirm the achieved control rate before any
tracking claim is made.

## Change control and hardware gate

This method is incompatible with T007 schema-2 results because waist yaw now
participates in the wrist and head tasks and orientation enters the joint
optimization. Older runs remain immutable legacy evidence.

Before mapping to the real R1, resolve the head motor index conflict between
the local high-level specification and the pinned vendor R1-A5 interface,
validate FK and joint signs against measured robot poses, replace simulation
rate settings with reviewed hardware limits, add collision and torque guards,
exercise dry-run output without motor enable, and follow the repository's
hardware safety procedure with an E-stop operator. None of those gates is
claimed complete here.
