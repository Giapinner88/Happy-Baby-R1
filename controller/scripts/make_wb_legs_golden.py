#!/usr/bin/env python3
"""Golden trace for contract r1_wb_legs_v1_blind (dev machine only).

Builds the 71-D actor observation directly from the Isaac Lab term definitions
used in training (Happy-Baby-R1-loco loco/r1/isaac/env_cfg.py build(), terms in
order; no obs scaling; normalizer inside the ONNX):
  base_ang_vel      = root_ang_vel_b            (IMU gyro, body frame)
  projected_gravity = projected_gravity_b
  command           = twist (vx, vy, yaw rate)
  phase             = sin/cos(2*pi*t/period), zero when |command| < 0.1 (mdp.phase)
  joint_pos         = q - default_joint_pos      (joint_pos_rel, 24 joints)
  joint_vel         = dq                         (joint_vel_rel, default vel = 0)
  actions           = last raw action (12)       (base.last_action)
runs the ONNX with onnxruntime over consecutive steps (so last_action is carried)
and writes, per step, the inputs, the observation, the raw action and the 24
joint targets (legs = default + scale * action, waist and arms = default).

controller/tests/wb_legs_golden_test.cpp replays the file through
WholeBodyLegsController and must reproduce every number.

Usage: python make_wb_legs_golden.py <policy.onnx> <output.txt>
(needs numpy + onnxruntime; noise-free; fixed seed 7).
"""
import sys

import numpy as np
import onnxruntime as ort


def main(model_path, out_path):
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    meta = session.get_modelmeta().custom_metadata_map
    assert meta["policy_contract"] == "r1_wb_legs_v1_blind"
    default = np.array([float(x) for x in meta["default_joint_pos"].split(",")], dtype=np.float32)
    scale = np.array([float(x) for x in meta["action_scale"].split(",")], dtype=np.float32)
    period = float(meta["gait_period_s"])
    assert default.shape == (24,) and scale.shape == (12,)

    rng = np.random.default_rng(7)
    previous = np.zeros(12, dtype=np.float32)
    commands = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.05), (0.5, 0.0, 0.0), (0.3, -0.2, 0.4),
                (0.0, 0.0, 0.0), (0.8, 0.1, -0.3), (0.0, 0.09, 0.0), (-0.4, 0.0, 0.2)]
    lines = [f"# r1_wb_legs_v1_blind golden, model sha-agnostic; period={period}"]
    for step, command in enumerate(commands):
        q = (default + rng.uniform(-0.15, 0.15, 24)).astype(np.float32)
        dq = rng.uniform(-1.0, 1.0, 24).astype(np.float32)
        gyro = rng.uniform(-0.5, 0.5, 3).astype(np.float32)
        g = np.array([0.0, 0.0, -1.0]) + rng.uniform(-0.1, 0.1, 3)
        gravity = (g / np.linalg.norm(g)).astype(np.float32)
        cmd = np.array(command, dtype=np.float32)
        t = 0.02 * (step + 3)
        if np.linalg.norm(cmd) < 0.1:
            phase = np.zeros(2, dtype=np.float32)
        else:
            phase = np.array([np.sin(2 * np.pi * t / period), np.cos(2 * np.pi * t / period)],
                             dtype=np.float32)
        obs = np.concatenate((gyro, gravity, cmd, phase, q - default, dq, previous)).astype(np.float32)
        assert obs.shape == (71,)
        action = session.run(None, {session.get_inputs()[0].name: obs[None]})[0][0].astype(np.float32)
        target = default.copy()
        target[:12] = default[:12] + scale * action
        previous = action.copy()

        def row(tag, values):
            return tag + " " + " ".join(f"{v:.9g}" for v in np.asarray(values).ravel())
        lines += [f"step {step}", row("q", q), row("dq", dq), row("gyro", gyro),
                  row("gravity", gravity), row("command", cmd), row("phase", phase),
                  row("obs", obs), row("action", action), row("target", target)]
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:3])
