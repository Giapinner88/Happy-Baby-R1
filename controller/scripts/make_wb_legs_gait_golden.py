#!/usr/bin/env python3
"""Golden trace for contract r1_wb_legs_gait_h4_v1_blind (dev machine only).

Reference implementation of the U4 actor input, independent of the C++ controller:
  frame (71)  = base_ang_vel, projected_gravity, command, phase, joint_pos[24],
                joint_vel[24], actions[12]   (same terms as r1_wb_legs_v1_blind,
                see make_wb_legs_golden.py)
  history     = Isaac Lab per-term CircularBuffer, history_length 4,
                flatten_history_dim: for each term, oldest -> newest; the first
                sample after reset fills every slot (repeat_first)
  gait one-hot= loco/r1/gait_fsm.py GaitFsm (the FSM U4 was trained with), one
                env, stage C2, joint mask = 12 leg joints, dt = 0.02 s, updated
                (update_filter, then update) before the observation, as
                GaitCommand.compute runs before the observation manager
  input       = concat(history (284), one-hot (3)) = 287
runs the ONNX with onnxruntime step by step (last_action carried) and writes, per
step, the inputs, the FSM mode, the observation, the raw action and the 24 targets.

The scripted trajectory visits every FSM edge: STAND at init (stop, quiet,
upright), WALK on a move command, W2S on stop, STAND after the 1.5 s settle
dwell, W2S on a push (tilt > 8 deg), hysteresis band (0.12 m/s keeps the mode).

controller/tests/wb_legs_gait_golden_test.cpp replays the file through
WholeBodyLegsGaitController and must reproduce every number.

Usage: python make_wb_legs_gait_golden.py <policy.onnx> <Happy-Baby-R1-loco root> <out.txt>
(needs numpy, torch, onnxruntime; fixed seed 11).
"""
import math
import sys

import numpy as np
import onnxruntime as ort

TERM_DIMS = (3, 3, 3, 2, 24, 24, 12)
STEPS = 4
DT = 0.02


def segment(step):
    """(command, dq amplitude, gyro amplitude, tilt deg) of the scripted trajectory."""
    if step < 10:
        return (0.0, 0.0, 0.0), 0.05, 0.05, 1.0    # init: stop, quiet -> STAND
    if step < 25:
        return (0.5, 0.0, 0.2), 1.5, 0.4, 2.0      # move -> WALK
    if step < 115:
        return (0.0, 0.0, 0.0), 0.05, 0.05, 1.0    # stop -> W2S, settle 1.5 s -> STAND
    if step < 122:
        return (0.0, 0.0, 0.0), 0.3, 0.3, 10.0     # push: tilt 10 deg -> W2S
    return (0.12, 0.0, 0.0), 0.05, 0.05, 1.0       # hysteresis band: no stop, no move


def main(model_path, loco_root, out_path):
    sys.path.insert(0, loco_root)
    import torch
    from loco.r1 import gait_fsm as G

    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    meta = session.get_modelmeta().custom_metadata_map
    assert meta["policy_contract"] == "r1_wb_legs_gait_h4_v1_blind"
    assert meta["actor_history_order"] == "term_major_oldest_to_newest"
    default = np.array([float(x) for x in meta["default_joint_pos"].split(",")], dtype=np.float32)
    scale = np.array([float(x) for x in meta["action_scale"].split(",")], dtype=np.float32)
    period = float(meta["gait_period_s"])
    params = G.FsmParams(settle_dwell_s=float(meta["gait_t_settle_s"]))
    mask = [True] * 12 + [False] * 12
    fsm = G.GaitFsm(1, "cpu", params, joint_mask=mask, num_joints=24)

    rng = np.random.default_rng(11)
    previous = np.zeros(12, dtype=np.float32)
    buffers = None
    lines = [f"# r1_wb_legs_gait_h4_v1_blind golden, model sha-agnostic; period={period}"]
    modes = []
    for step in range(130):
        command, dq_amp, gyro_amp, tilt_deg = segment(step)
        q = (default + rng.uniform(-0.05, 0.05, 24)).astype(np.float32)
        dq = rng.uniform(-dq_amp, dq_amp, 24).astype(np.float32)
        gyro = rng.uniform(-gyro_amp, gyro_amp, 3).astype(np.float32)
        yaw = rng.uniform(0, 2 * math.pi)
        tilt = math.radians(tilt_deg)
        gravity = np.array([math.sin(tilt) * math.cos(yaw), math.sin(tilt) * math.sin(yaw),
                            -math.cos(tilt)], dtype=np.float32)
        cmd = np.array(command, dtype=np.float32)
        t = DT * step
        if np.linalg.norm(cmd) < 0.1:
            phase = np.zeros(2, dtype=np.float32)
        else:
            phase = np.array([math.sin(2 * math.pi * t / period), math.cos(2 * math.pi * t / period)],
                             dtype=np.float32)

        fsm.update_filter(torch.tensor(gyro[None]), torch.tensor(dq[None]), DT)
        fsm.update("C2", torch.tensor(cmd[None]), G.tilt_from_gravity(torch.tensor(gravity[None])), DT)
        one_hot = fsm.one_hot()[0].numpy().astype(np.float32)
        modes.append(int(fsm.gait_for_action[0]))

        terms = (gyro, gravity, cmd, phase, q - default, dq, previous)
        if buffers is None:
            buffers = [[term.copy() for _ in range(STEPS)] for term in terms]
        else:
            for buf, term in zip(buffers, terms):
                buf.pop(0)
                buf.append(term.copy())
        history = np.concatenate([np.concatenate(buf) for buf in buffers])
        obs = np.concatenate((history, one_hot)).astype(np.float32)
        assert obs.shape == (287,) and sum(TERM_DIMS) == 71
        action = session.run(None, {session.get_inputs()[0].name: obs[None]})[0][0].astype(np.float32)
        target = default.copy()
        target[:12] = default[:12] + scale * action
        previous = action.copy()

        def row(tag, values):
            return tag + " " + " ".join(f"{v:.9g}" for v in np.asarray(values).ravel())
        lines += [f"step {step}", row("q", q), row("dq", dq), row("gyro", gyro),
                  row("gravity", gravity), row("command", cmd), row("phase", phase),
                  row("mode", [modes[-1]]), row("obs", obs), row("action", action),
                  row("target", target)]
    visited = {m: modes.index(m) for m in set(modes)}
    assert set(visited) == {G.STAND, G.WALK, G.W2S}, visited
    print("first step per mode", {G.MODES[m]: s for m, s in sorted(visited.items())},
          "transitions", int(fsm.transitions[0]))
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:4])
