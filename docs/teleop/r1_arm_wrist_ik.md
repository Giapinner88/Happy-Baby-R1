# Method — R1 arm/wrist inverse kinematics for Quest teleoperation

This is the method record required by "Gate M" in
[arm_wrist_simulation_study_plan.md](../../experiments/r1_teleop/quest3_sim_v1/arm_wrist_simulation_study_plan.md)
before any T002 IK run. It owns the accepted model, its frames, units, signs,
constraints and ownership rules. It owns no run parameters, no observed results
and no claim: those belong to the T002 experiment record.

Everything below marked **audited** was read from the repository asset and is
checked by `tests/teleop/test_r1_arm_wrist_ik_method.py`. Everything marked
**assumed** is a declared assumption that T002 must verify before its evidence
counts.

## 1. The arm has five degrees of freedom, so the target is not a 6-DOF pose

**Audited** from `assets/R1.urdf`, each arm is a five-joint serial chain:

| Joint | Axis | Lower (rad) | Upper (rad) | Effort (N·m) | Velocity (rad/s) |
|---|---|---|---|---|---|
| `*_shoulder_pitch_joint` | `0 1 0` | −3.1416 | 2.0944 | 60.0 | 18.8 |
| `*_shoulder_roll_joint` (left) | `1 0 0` | −0.2269 | 2.4784 | 60.0 | 18.8 |
| `*_shoulder_roll_joint` (right) | `1 0 0` | −2.4785 | 0.2268 | 60.0 | 18.8 |
| `*_shoulder_yaw_joint` | `0 0 1` | −1.9199 | 1.9199 | 33.0 | 33.4 |
| `*_elbow_joint` | `0 1 0` | −0.9756 | 2.1852 | 33.0 | 33.4 |
| `*_wrist_roll_joint` | `1 0 0` | −1.9199 | 1.9199 | 33.0 | 33.4 |

The shoulder-roll range is mirrored between arms, not symmetric: the left arm
opens positive and the right arm opens negative. A mapping that assumes a shared
range will silently push one arm to its limit.

Five joints cannot realize an arbitrary rigid-body pose, which needs six. The
task target is therefore **four scalars**: the 3-D wrist endpoint position and
the wrist-roll angle. That leaves one degree of redundancy, resolved in the null
space by a posture bias (§5).

`R1TeleopCommand` carries a full quaternion for each wrist. **That quaternion
must not be read as a 6-DOF orientation target.** Only the component that maps
onto wrist roll is used; the remaining orientation freedom is not commandable on
this asset. A future asset with a wrist pitch/yaw joint requires a new schema
version, not a reinterpretation of this field.

There is **no finger or gripper joint**: `left_hand_collision` and
`right_hand_collision` are collision geometry. This method covers arm, wrist roll
and a rigid hand endpoint. It supports no grasping claim.

## 2. Frames

| Frame | Meaning |
|---|---|
| `quest_headset` | Source frame of `R1TeleopCommand`, as delivered by the vendor wrapper. |
| `r1_base` | Robot frame after calibration, named in `robot_frame`. |
| endpoint | Vendor R1-A5 virtual EE: 0.20 m along local +x from `*_wrist_roll_joint`. |

**Audited** kinematic chain, left arm (right mirrors it):
`waist_yaw_link` → `left_shoulder_pitch_link` → `left_shoulder_roll_link` →
`left_shoulder_yaw_link` → `left_elbow_link` → `left_wrist_roll_link`.

The controlled point is not the wrist joint origin. The vendored R1-A5
implementation adds `L_ee`/`R_ee` with translation `[0.20, 0, 0]` from the
wrist-roll joint. `ArmChain.forward_kinematics()` and
`ArmChain.position_jacobian()` must use this same point; mixing the virtual EE
in FK with the wrist origin in the Jacobian makes reachable targets stall.

The chain is rooted at `waist_yaw_link`, **not** at the pelvis. Waist joints are
owned by locomotion (§6), so from the IK's point of view the arm base moves
whenever the waist moves. T002 fixes the base and must record the waist state it
assumed; T008 cannot inherit a workspace measured under a different waist pose.

**Assumed:** the vendor wrapper states that its controller pose output already
follows the Unitree humanoid-arm URDF convention with the origin translated from
head to waist, so no further basis change is applied. This is a vendor claim
carried through `teleop/r1/bridge.py` unmodified. T002 must verify it with a
zero/identity case before any reachability conclusion, because a wrong basis
produces a workspace that is internally consistent and entirely wrong.

## 3. Units, signs and time

Positions are metres, angles are radians, time is seconds on a monotonic clock,
angular velocity is rad/s. These are the `teleop/r1/schema.py` conventions and
this method introduces no others. Joint angle signs follow the URDF axes in §1.

Timestamps use `time.monotonic()`. The bridge and the simulator runner are
separate processes on one host, so `CLOCK_MONOTONIC` is shared and command age is
measured directly rather than estimated. Each run records this in
`clock_record.json`. Command age older than `command_timeout_s` fails closed
before IK is ever called (§7).

## 4. Mapping from a Quest pose to a calibrated target

Given a wrist pose in `quest_headset` and the calibration in the resolved config:

1. Apply the calibration yaw rotation about z, then the calibration translation.
   This is `_transform_pose` in `teleop/r1/mapping.py` and is already exercised
   by the mapper tests.
2. Take the transformed position as the endpoint position target.
3. Take the wrist-roll scalar from the transformed orientation.
4. Apply the experiment profile's pre-IK workspace policy. T002 uses its
   declared grid to measure reachability. The revised T007 live profile uses
   `ik_joint_limit_only`, so it does **not** clip or reject a target using a
   rectangular Cartesian box: it lets the joint-limited IK solve decide. The
   wrist-roll target remains bounded by the joint limits in §1.

**No scale factor is applied.** Operator motion maps to robot motion 1:1. If a
scale is introduced later it must be recorded here first, because a scale changes
which workspace conclusions from T002 remain valid.

T007 uses `absolute_vendor_pose`: the transformed vendor wrist position and
wrist roll are the arm targets directly, at scale 1:1. Right trigger begins a
fresh command session but does not redefine arm neutral. Left trigger resets to
the declared q=0 joint pose directly (it does not solve the neutral Cartesian
point from the current elbow branch), whose virtual-EE positions are `[0.328371, ±0.1386057447,
-0.0180351887]` m in the waist frame. The identity calibration must map an
incoming pose to the same numbers; this is the zero/identity audit case.

### Wrist-roll wrap does not occur on this asset

The study plan asks for continuity through wrist-roll wrap. **Audited:**
`*_wrist_roll_joint` is limited to ±1.9199 rad, which is strictly inside ±π, so
the joint cannot wrap and no unwrapping logic is required. A continuity
discontinuity in the *commanded* roll is still possible if the source quaternion
crosses a branch cut, so the extracted roll is unwrapped relative to the previous
commanded value before clamping. If a future asset removes the limit, this
section must be rewritten before that asset is used.

## 5. Solver

The task is four constraints on five joints, so the solver is an iterative
damped-least-squares step with a null-space posture bias:

- **Objective:** minimize squared endpoint position error plus squared
  wrist-roll error, with a damping term on the joint step.
- **Redundancy:** the remaining degree of freedom is resolved by biasing towards
  a declared nominal posture in the null space of the position task, so repeated
  identical targets converge to the same joint solution and traces stay
  comparable.
- **Convergence is on both tasks.** Iteration continues until the position
  residual is within tolerance **and** the applied joint step is below
  `posture_tolerance_rad`. Stopping on position alone is not sufficient: the
  position residual reaches tolerance within a handful of iterations while the
  posture bias is still moving the elbow, which leaves the redundant joint
  wherever the seed put it. Measured on the left arm, position-only stopping made
  two solves of the same target disagree by up to 0.13 rad; adding the settling
  criterion brought seeds in the same basin to agreement at 1e-16 rad.
- **Joint limits:** hard-clamped to §1 on every iteration, never only at the end.
- **Rate and acceleration limits:** the *form* is a per-step clamp on joint
  velocity and acceleration. The **numeric teleop limits are not set here.** The
  asset velocity limits in §1 are motor limits, not safe teleoperation limits, and
  this project has no baseline from which to derive the latter. T002 must declare
  them in its own resolved config, subject to the rule that each must be at or
  below the §1 asset limit. Inventing a number here would put an unaudited
  threshold into every downstream result.
- **Stopping:** converged when the position residual and the roll residual are
  both within the tolerances declared in the T002 config, or when the iteration
  budget is exhausted.
- **Fallback:** T002 retains non-convergence as a scientific workspace result.
  T007 simulation additionally preserves the best finite iterate when progress
  stagnates and labels it `projected_to_reachable_boundary`; the position
  residual remains explicit. Projection never moves the base.

### Repeatability is per basin, not global

The posture bias makes the solution a function of the target **for seeds within
one basin of attraction**, not globally. The chain reaches a given endpoint with
more than one elbow configuration. Measured on the left arm at one target, three
seeds near nominal converged to an identical solution while a seed far from
nominal converged to the other branch — the same endpoint with the elbow at
1.891 rad instead of 0.889 rad, both within limits and both converged.

Teleoperation avoids this by seeding each solve with the previous solution, so a
continuous input stream stays in one basin. A reset, a large target jump, or a
hold-and-resume can switch branch. T002 must traverse its target grid
continuously, record the seed used for every solve, and report branch switches as
observations rather than smoothing them away. A branch switch is a real jump in
joint space at constant endpoint, so it is also a rate-limit event.

### Self-collisions block the arm chain, so no collision-free claim is possible

The R1 USD has a self-collision defect that blocks the head joints under
sustained actuator torque. **T002 established that the arm chain is affected the
same way.** In a controlled A/B over 50 solver-converged targets, changing only
`enabled_self_collisions`, the median joint tracking error was 0.9945 rad with
collisions enabled versus 0.0205 rad with them disabled — 48× worse, and 48 of 50
targets exceeded 0.05 rad. A median error near one radian is mechanical blocking,
not settling.

Every functional run on this asset therefore uses `--disable-self-collisions`,
and **no result in this project may be described as a collision-free workspace**.
What is measurable is joint-limit reachability. Repairing the asset's collision
geometry is a separate piece of work; T002 localizes the defect to the asset and
does not fix it.

See [`experiments/r1_teleop/quest3_sim_v1/T002/T002.md`](../../experiments/r1_teleop/quest3_sim_v1/T002/T002.md).

## 6. Joint ownership

The 26 actuated joints partition into exactly two disjoint sets, and the union
must equal the asset's joint set. This is checked by the Gate M tests against
`R1JointOwnership` in `teleop/r1/mapping.py`.

| Owner | Joints | Count |
|---|---|---|
| Arm/wrist IK | 10 arm joints from §1 | 10 |
| Head scalar path (not IK) | `head_yaw_joint`, `head_pitch_joint` | 2 |
| Locomotion policy | 12 leg joints, `waist_roll_joint`, `waist_yaw_joint` | 14 |

Head targets come from the mapper's existing scalar yaw/pitch outputs and are
**not** produced by IK; they share the `upper_body` ownership group only for
dispatch. The waist belongs to locomotion. Reassigning the waist to IK would
change the arm base frame in §2 and therefore invalidate any workspace measured
before the change; it requires an explicit method change here, not a config edit.

**Audited caveat:** `assets/mujoco/unitree_robots/r1/R1.xml` has 24 actuated
joints and no head joints, while `assets/R1.urdf` and `assets/R1/R1.usd` have 26.
This ownership table describes the URDF/USD asset. MuJoCo cannot host this method
unchanged.

## 7. Hold behaviour

IK is only reached by a command that has already passed the mapper's fail-closed
checks: source-frame match, command age within `command_timeout_s`, and deadman
engaged. Any of those failing produces a hold before IK runs.

Once inside IK, a hold is issued when no finite usable solution exists, when an
enabled experiment workspace prefilter rejects a target, or when a rate clamp
would be violated by more than the declared margin. T007 may instead dispatch a
declared projected solution. A hold **freezes the last commanded joint target**
rather than returning to nominal: returning to nominal would be an uncommanded
fast motion at the moment the input became untrusted. This matches the existing
`HeadOnlyIsaacLabSink` hold semantics.

No hold path may emit a base-velocity command. Velocity stays disabled until the
separately recorded IsaacLab locomotion evaluation accepts a matching policy
signature.

The T007 simulation profile permits a converged position solution at a hard
arm-joint limit and a bounded best-effort projection for an unreachable target.
It records the solver status, residual, joint name and remaining limit margin;
wrist roll is clamped to its URDF limit. This profile is not a hardware command
limit.

## 8. Gate M audit

Gate M passes when all of the following hold, checked by
`tests/teleop/test_r1_arm_wrist_ik_method.py`:

1. **Unit and frame audit** — the schema conventions in §3 are the ones the
   mapper uses, and the endpoint link names in §2 exist in the asset.
2. **Zero/identity mapping** — an identity calibration maps a pose to itself,
   and a declared neutral maps to the declared neutral target.
3. **Joint list matches the asset** — the joint names and limits in §1 are the
   asset's actual joints and limits, and the ownership union in §6 equals the
   asset's actuated joint set exactly.
4. **Joint ownership** — the two owner sets are disjoint, and no IK-owned joint
   appears in the locomotion set.

```bash
python3 -m unittest -v tests.teleop.test_r1_arm_wrist_ik_method
```

If Gate M fails, fix the method before creating any T002 run evidence.

## 9. What this method does not settle

The numeric rate, acceleration, residual-tolerance and workspace-clip values are
deliberately absent; T002 declares them. Collision-free workspace is unresolved
(§5). The vendor basis claim in §2 is assumed, not verified. Dexterous-hand or
grasping behaviour is out of scope because the asset has no finger joints.
# Legacy method notice

This document remains authoritative for T007 schema-2 independent-arm runs.
The incompatible schema-3 coupled waist/arms/head pilot is defined separately
in [r1_upper_body_ik.md](r1_upper_body_ik.md); old evidence must not be silently
reinterpreted with that model.
