#!/usr/bin/env python3
"""Solve bilateral wrist targets with the upstream `R1_A5_ArmIK`, unmodified.

This is a worker, not an entry point. It exists because upstream ships its own
package literally named `teleop`, which collides with this repository's
`teleop` package: whichever is imported first wins, and the other's submodules
stop resolving. Rather than rename either side or patch `third_party`, the
upstream solver is given a process where its own `teleop` is the only one on
the path, and it talks to the driver through two `.npz` files.

Input npz : left_target_pelvis (N,4,4), right_target_pelvis (N,4,4)
Output npz: joint_position_rad (N,10), ee_position_pelvis (N,2,3), solve_ms (N,)

Targets are in the pelvis (model root) frame, which is what upstream's reduced
model expects; the driver does the conversion. The ten joints come back in
upstream's own order and are echoed with the frame positions its solver
actually drives (`L_ee`/`R_ee`, the vendor's 0.20 m virtual tool frames), so
the driver can score the vendor on the vendor's own definition of the task.

Simulation-only. Grants no hardware authority.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

UPSTREAM_ROOT = Path(__file__).resolve().parents[2] / "third_party" / "xr_teleoperate_v1_6"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    payload = np.load(args.input)
    left = np.asarray(payload["left_target_pelvis"], dtype=float)
    right = np.asarray(payload["right_target_pelvis"], dtype=float)
    if left.shape != right.shape or left.ndim != 3 or left.shape[1:] != (4, 4):
        raise SystemExit(f"expected matching (N, 4, 4) targets, got {left.shape} and {right.shape}")

    # Upstream resolves its URDF, meshes and model cache relative to the
    # working directory, so the process moves there rather than editing paths.
    os.chdir(UPSTREAM_ROOT / "teleop" / "robot_control")
    sys.path.insert(0, str(UPSTREAM_ROOT))
    sys.path.insert(0, str(UPSTREAM_ROOT / "teleop"))

    import pinocchio as pin
    from robot_control.robot_arm_ik import R1_A5_ArmIK

    solver = R1_A5_ArmIK(Unit_Test=True, Visualization=False)
    model = solver.reduced_robot.model
    data = solver.reduced_robot.data
    left_frame = model.getFrameId("L_ee")
    right_frame = model.getFrameId("R_ee")

    count = len(left)
    joints = np.zeros((count, model.nq), dtype=float)
    endpoints = np.zeros((count, 2, 3), dtype=float)
    solve_ms = np.zeros(count, dtype=float)

    for index in range(count):
        started = time.perf_counter()
        solution, _torque = solver.solve_ik(left[index], right[index])
        solve_ms[index] = 1000.0 * (time.perf_counter() - started)
        q = np.asarray(solution, dtype=float)
        joints[index] = q
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        endpoints[index, 0] = np.asarray(data.oMf[left_frame].translation, dtype=float)
        endpoints[index, 1] = np.asarray(data.oMf[right_frame].translation, dtype=float)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        joint_position_rad=joints,
        ee_position_pelvis=endpoints,
        solve_ms=solve_ms,
        upstream_joint_names=np.asarray([str(name) for name in model.names]),
        upstream_lower_limits=np.asarray(model.lowerPositionLimit, dtype=float),
        upstream_upper_limits=np.asarray(model.upperPositionLimit, dtype=float),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
