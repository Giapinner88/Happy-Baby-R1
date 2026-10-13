# D002 — Couple waist yaw, bilateral arms, and head in T007 IK

Status: selected for simulation pilot; hardware use not authorized  
Date: 2026-08-18

## Context and decision

The schema-2 controller solves each five-joint arm independently while keeping
the torso fixed. Live evidence showed large extension residuals and poses that
could benefit from shared torso yaw. The selected schema-3 pilot solves one
13-joint vector containing `waist_yaw_joint`, both arms, and both head joints.
The root, legs, and `waist_roll_joint` remain fixed.

The decision applies only to the T007 simulation path selected with
`--whole-upper-body-config`. The legacy `--arm-head-config` behavior and all old
runs are preserved.

## Alternatives considered

- Continue independent per-arm IK: simplest and already evidenced, but cannot
  allocate torso yaw to bilateral reach.
- Add simulation-only waist roll: expands the numerical workspace but does not
  match the pinned R1-A5 hardware model, where that motor slot is unused.
- Use whole-body IK including legs/root: may improve reach, but mixes the
  current upper-body question with balance and locomotion and would no longer
  be a smallest discriminating test.
- Use the upstream R1-A5 Pinocchio solver unchanged: useful reference, but its
  current implementation locks waist yaw and head joints, so it does not test
  the requested coupled upper body.

## Consequences and validation

The method changes ownership of `waist_yaw_joint` from lower body to upper-body
IK, introduces coupled bilateral failure semantics, and breaks direct numerical
comparability with schema-2 T007. It also exposes rather than hides the
pelvis-to-waist difference between the local and vendor URDFs.

The implementation must pass the pure-Python FK/Jacobian/IK and dispatch tests
for both assets, preserve the legacy test suite, and complete a new fixed-base
Quest/Isaac run whose resolved configuration and residual traces are inspected.
Revert or redesign if coupling worsens bilateral position residual, cannot
sustain the declared control rate, produces collision/limit violations, or the
real R1 kinematic/sign audit contradicts the model.

