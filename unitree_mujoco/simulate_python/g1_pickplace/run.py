import os, sys, time
import mujoco
import mujoco.viewer
import numpy as np

# ── path setup ────────────────────────────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_SIM  = os.path.abspath(os.path.join(_HERE, ".."))
if _SIM not in sys.path:
    sys.path.insert(0, _SIM)

from g1_pickplace.kinematics import solve_ik
from g1_pickplace.pid         import G1ArmPIDTorque
from g1_pickplace.fsm         import G1PickPlaceFSM

# ─── Scene ────────────────────────────────────────────────────────────────────
SCENE_XML = os.path.join(_SIM, "../unitree_robots/g1/scene_pickplace.xml")

# ─── Right arm joint names (7-DOF) ───────────────────────────────────────────
ARM_JOINT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
ARM_ACT_NAMES = [n.replace("_joint", "") for n in ARM_JOINT_NAMES]

# ─── Control parameters ───────────────────────────────────────────────────────
MAX_VEL = 1.5   # rad/s — joint velocity saturation for rate limiter

# Q_HOME: initial joint posture — arm retracted, elbow up, well clear of table
# shoulder_pitch=-0.3 pulls the arm BACKWARD; elbow=1.5 keeps it folded high
Q_HOME  = np.array([-0.3, -0.35, 0.0, 1.5, 0.0, 0.0, 0.0])


def main():
    model = mujoco.MjModel.from_xml_path(SCENE_XML)
    data  = mujoco.MjData(model)

    # ── Resolve joint IDs ─────────────────────────────────────────────────────
    arm_jnt_obj_ids = []
    for name in ARM_JOINT_NAMES:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid == -1:
            raise RuntimeError(f"Joint '{name}' not found in model!")
        arm_jnt_obj_ids.append(jid)

    # qpos addresses  → reading/writing joint positions
    qpos_ids = [model.jnt_qposadr[j] for j in arm_jnt_obj_ids]
    # dof  addresses  → Jacobian columns & qvel
    dof_ids  = [model.jnt_dofadr[j]  for j in arm_jnt_obj_ids]

    # ── Resolve actuator IDs ──────────────────────────────────────────────────
    arm_act_ids = []
    for name in ARM_ACT_NAMES:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid == -1:
            raise RuntimeError(f"Actuator '{name}' not found!")
        arm_act_ids.append(aid)

    gripper_act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper")
    if gripper_act_id == -1:
        raise RuntimeError("Actuator 'gripper' not found!")

    tau_low  = model.actuator_ctrlrange[arm_act_ids, 0]
    tau_high = model.actuator_ctrlrange[arm_act_ids, 1]

    # ── Seed arm to home posture ──────────────────────────────────────────────
    for i, qid in enumerate(qpos_ids):
        data.qpos[qid] = Q_HOME[i]
    mujoco.mj_forward(model, data)

    # ── Warm-start: apply gravity compensation so arm doesn't fall ────────────
    tau0 = np.array([data.qfrc_bias[d] for d in dof_ids])
    for i, aid in enumerate(arm_act_ids):
        data.ctrl[aid] = float(np.clip(tau0[i], tau_low[i], tau_high[i]))
    data.ctrl[gripper_act_id] = 0.0   # open gripper

    # ── Initialise controllers ────────────────────────────────────────────────
    dt  = model.opt.timestep
    pid = G1ArmPIDTorque(dt=dt)
    fsm = G1PickPlaceFSM()

    # q_filt: rate-limited joint target  (initialised to actual qpos)
    q_filt = np.array([data.qpos[i] for i in qpos_ids], dtype=float)

    # End-effector site
    ee_id = model.site("pinch_site").id

    print("═══════════════════════════════════════")
    print("  G1 Pick-and-Place Controller (v2)    ")
    print(f"  dt = {dt*1000:.1f} ms | MAX_VEL = {MAX_VEL} rad/s")
    print("═══════════════════════════════════════")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.5, 0.0, 0.90]
        viewer.cam.distance  = 1.9
        viewer.cam.elevation = -22
        viewer.cam.azimuth   = 145

        while viewer.is_running():
            step_start = time.perf_counter()

            # ── 1. Current TCP position ───────────────────────────────────────
            tcp_pos = data.site(ee_id).xpos.copy()

            # ── 2. FSM → Cartesian target + gripper command ───────────────────
            target_xyz, gripper_cmd, state = fsm.step(data, tcp_pos)

            # ── 3. IK → absolute joint target q_raw ──────────────────────────
            q_raw, ik_info = solve_ik(
                model, data, target_xyz, qpos_ids, dof_ids, site_name="pinch_site"
            )

            # ── 4. Rate-limit: q_filt tracks q_raw at ≤ MAX_VEL ──────────────
            dq_max  = MAX_VEL * dt
            q_filt += np.clip(q_raw - q_filt, -dq_max, dq_max)

            # Enforce joint limits on q_filt
            for i, jid in enumerate(arm_jnt_obj_ids):
                lo, hi = model.jnt_range[jid]
                q_filt[i] = float(np.clip(q_filt[i], lo, hi))

            # ── 5. PID Torque + gravity compensation ──────────────────────────
            q_current  = np.array([data.qpos[i] for i in qpos_ids])
            qd_current = np.array([data.qvel[i] for i in dof_ids])
            gravity    = np.array([data.qfrc_bias[i] for i in dof_ids])

            tau = pid.compute_torque(q_filt, q_current, qd_current, gravity)
            tau = np.clip(tau, tau_low, tau_high)

            for i, aid in enumerate(arm_act_ids):
                data.ctrl[aid] = float(tau[i])

            # ── 6. Gripper ────────────────────────────────────────────────────
            data.ctrl[gripper_act_id] = float(np.clip(gripper_cmd, -0.8, 0.8))

            # ── 7. Step ───────────────────────────────────────────────────────
            mujoco.mj_step(model, data)
            viewer.sync()

            # Real-time pacing
            elapsed = time.perf_counter() - step_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)


if __name__ == "__main__":
    main()
