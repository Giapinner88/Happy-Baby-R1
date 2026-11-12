#!/usr/bin/env python3
"""Golden trace for contracts r1_wb_legs_gait_h4_v1_blind (U4, 12 actions),
r1_wb_legs_waist_gait_h4_v1_blind (U7, 14 actions: legs + waist roll/yaw) and
r1_wb_legs_armres_gait_h4_v1_ref (U10, 22 actions: legs + 10 arm residuals) (dev machine only).

U10 adds, per frame, arm_q_ref[10], arm_dq_ref[10] after actions: the scripted desired arm
pose through loco/r1/reference.py reference_filter_step (the training filter) at the
hardware ceilings 4 rad/s, 14 rad/s^2, updated before the observation; the arm authority g
is appended after the one-hot; arm targets = clamp(q_ref + g * arm_residual_rad *
clip(a, -1, 1)) to the Isaac soft limits (0.9 of the URDF range, read from
<loco>/assets/R1.urdf). The desired pose pushes left shoulder roll past its soft limit so
the clamp is exercised.

Reference implementation of the U4 actor input, independent of the C++ controller:
  frame (71)  = base_ang_vel, projected_gravity, command, phase, joint_pos[24],
                joint_vel[24], actions[A]   (same terms as r1_wb_legs_v1_blind,
                see make_wb_legs_golden.py); A = 12, or 14 for the waist contract
                (frame 73)
  history     = Isaac Lab per-term CircularBuffer, history_length 4,
                flatten_history_dim: for each term, oldest -> newest; the first
                sample after reset fills every slot (repeat_first)
  gait one-hot= loco/r1/gait_fsm.py GaitFsm (the FSM U4 was trained with), one
                env, stage C2, joint mask = 12 leg joints, dt = 0.02 s, updated
                (update_filter, then update) before the observation, as
                GaitCommand.compute runs before the observation manager
  input       = concat(history (4 x frame), one-hot (3)) = 287 or 295
runs the ONNX with onnxruntime step by step (last_action carried) and writes, per
step, the inputs, the FSM mode, the observation, the raw action and the 24 targets
(joints 0..A-1 from the policy, the rest at default_joint_pos).

The scripted trajectory visits every FSM edge: STAND at init (stop, quiet,
upright), WALK on a move command, W2S on stop, STAND after the settle
dwell (metadata gait_t_settle_s: U4 1.5 s, U7 0.5 s), W2S on a push (tilt > 8 deg), hysteresis band (0.12 m/s keeps the mode).

controller/tests/wb_legs_gait_golden_test.cpp replays the file through
WholeBodyLegsGaitController and must reproduce every number.

Usage: python make_wb_legs_gait_golden.py <policy.onnx> <Happy-Baby-R1-loco root> <out.txt>
(needs numpy, torch, onnxruntime; fixed seed 11).
"""
import math
import sys

import numpy as np
import onnxruntime as ort

CONTRACTS = {"r1_wb_legs_gait_h4_v1_blind": 12, "r1_wb_legs_waist_gait_h4_v1_blind": 14,
             "r1_wb_legs_armres_gait_h4_v1_ref": 22}
SOFT_LIMIT_FACTOR = 0.9  # Isaac soft_joint_pos_limit_factor in the U10 env.yaml
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


def arm_segment(step, default_arm):
    """(desired arm pose, authority g) of the scripted U10 arm reference."""
    if step < 40:
        return default_arm, 1.0                    # idle: default pose, g = 1
    if step < 85:
        pose = default_arm.copy()                  # teleop moving: a large pose, g = 0.6
        pose[[0, 1, 3, 5, 8]] = (-1.0, 2.45, 1.5, -0.8, 1.2)  # left roll 2.45 > soft 2.343
        return pose, 0.6
    return default_arm, (0.0 if step < 110 else 1.0)  # holding (g = 0), then idle


def soft_arm_limits(loco_root, names):
    import xml.etree.ElementTree as ET
    joints = {j.get("name"): j.find("limit") for j in ET.parse(f"{loco_root}/assets/R1.urdf").getroot().iter("joint")}
    lo = np.array([float(joints[n].get("lower")) for n in names], dtype=np.float32)
    hi = np.array([float(joints[n].get("upper")) for n in names], dtype=np.float32)
    mid, half = 0.5 * (lo + hi), 0.5 * (hi - lo) * SOFT_LIMIT_FACTOR
    return mid - half, mid + half


def main(model_path, loco_root, out_path):
    sys.path.insert(0, loco_root)
    import torch
    from loco.r1 import gait_fsm as G
    from loco.r1 import reference as REF

    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    meta = session.get_modelmeta().custom_metadata_map
    contract = meta["policy_contract"]
    n_action = CONTRACTS[contract]
    assert len(meta["action_joint_names"].split(",")) == n_action
    armres = n_action == 22
    term_dims = (3, 3, 3, 2, 24, 24, n_action) + ((10, 10) if armres else ())
    assert meta["actor_history_order"] == "term_major_oldest_to_newest"
    default = np.array([float(x) for x in meta["default_joint_pos"].split(",")], dtype=np.float32)
    scale = np.array([float(x) for x in meta["action_scale"].split(",")], dtype=np.float32)
    period = float(meta["gait_period_s"])
    params = G.FsmParams(settle_dwell_s=float(meta["gait_t_settle_s"]))
    mask = [True] * 12 + [False] * 12
    fsm = G.GaitFsm(1, "cpu", params, joint_mask=mask, num_joints=24)

    if armres:
        arm_names = meta["arm_residual_joints"].split(",")
        assert meta["action_joint_names"].split(",")[12:] == arm_names
        residual_rad = float(meta["arm_residual_rad"])
        soft_lo, soft_hi = soft_arm_limits(loco_root, arm_names)
        q_ref = torch.tensor(default[14:][None])
        dq_ref = torch.zeros_like(q_ref)
        clamped = 0

    rng = np.random.default_rng(11)
    previous = np.zeros(n_action, dtype=np.float32)
    buffers = None
    lines = [f"# {contract} golden, model sha-agnostic; period={period}"]
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
        if armres:
            desired, authority = arm_segment(step, default[14:].copy())
            q_ref, dq_ref = REF.reference_filter_step(q_ref, dq_ref, torch.tensor(desired[None]),
                                                      REF.MAX_VEL_RAD_S, REF.MAX_ACC_RAD_S2, DT)
            ref_q, ref_dq = q_ref[0].numpy().astype(np.float32), dq_ref[0].numpy().astype(np.float32)
            terms += (ref_q, ref_dq)
        if buffers is None:
            buffers = [[term.copy() for _ in range(STEPS)] for term in terms]
        else:
            for buf, term in zip(buffers, terms):
                buf.pop(0)
                buf.append(term.copy())
        history = np.concatenate([np.concatenate(buf) for buf in buffers])
        tail = (one_hot, np.array([authority], dtype=np.float32)) if armres else (one_hot,)
        obs = np.concatenate((history,) + tail).astype(np.float32)
        assert obs.shape == (STEPS * sum(term_dims) + 3 + armres,)
        action = session.run(None, {session.get_inputs()[0].name: obs[None]})[0][0].astype(np.float32)
        target = default.copy()
        if armres:
            target[:12] = default[:12] + scale[:12] * action[:12]
            raw = np.clip(action[12:], -1.0, 1.0)
            arm = np.minimum(np.maximum(ref_q + authority * residual_rad * raw, soft_lo), soft_hi)
            clamped += int(np.any((arm == soft_lo) | (arm == soft_hi)))
            target[14:] = arm
        else:
            target[:n_action] = default[:n_action] + scale * action
        previous = action.copy()

        def row(tag, values):
            return tag + " " + " ".join(f"{v:.9g}" for v in np.asarray(values).ravel())
        lines += [f"step {step}", row("q", q), row("dq", dq), row("gyro", gyro),
                  row("gravity", gravity), row("command", cmd), row("phase", phase),
                  row("mode", [modes[-1]]), row("obs", obs), row("action", action),
                  row("target", target)]
        if armres:
            lines[-1:-1] = [row("arm_desired", desired), row("authority", [authority])]
    if armres:
        assert clamped > 0, "arm soft-limit clamp never engaged"
        print("steps with an arm target at its soft limit:", clamped)
    visited = {m: modes.index(m) for m in set(modes)}
    assert set(visited) == {G.STAND, G.WALK, G.W2S}, visited
    print("first step per mode", {G.MODES[m]: s for m, s in sorted(visited.items())},
          "transitions", int(fsm.transitions[0]))
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(*sys.argv[1:4])
