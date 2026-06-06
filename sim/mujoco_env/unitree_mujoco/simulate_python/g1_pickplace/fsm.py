"""
g1_pickplace/fsm.py
===================
Pick-and-place Finite State Machine for Unitree G1 right arm.

Architecture mirrors UR5e run2.py:
  - Reads body positions DIRECTLY from sim (data.body(...).xpos)
  - Uses data.time (sim clock) for phase timing — not timestep counters
  - Returns (target_xyz, gripper_cmd, state_name) per call
  - No internal accumulation of IK state

Scene layout (scene_pickplace.xml):
  - Table top:    z = 0.80 m
  - target_box:   (0.45, -0.15, 0.83)   ← red cube to pick
  - container:    (0.55,  0.20, 0.80)   ← blue tray to drop into
"""

import numpy as np

# ─── Thresholds ───────────────────────────────────────────────────────────────
XY_HOVER_THRESH = 0.020   # m  — XY error to enter HOVER phase
Z_GRASP_THRESH  = 0.022   # m  — Z error to trigger grasp
HOVER_SETTLE    = 0.15    # s  — wait time after XY align before descending
GRASP_TIME      = 0.40    # s  — gripper close duration
RELEASE_TIME    = 0.25    # s  — gripper open duration before retreat

# ─── Heights ──────────────────────────────────────────────────────────────────
# NOTE: LIFT_Z must be well above the table (z=0.80). Robot shoulder is at ~1.05m.
# During startup the arm swings through table space, so SAFE_INIT pulls it
# to a high retracted waypoint FIRST before any forward motion.
SAFE_Z          = 1.15    # m  — startup retraction height (clear of table)
LIFT_Z          = 1.08    # m  — safe carry height during pick/place
BIN_OVER_Z      = 1.08    # m  — approach height above container
DROP_Z          = 0.87    # m  — release height inside container
DESCEND_OFFSET  = -0.01   # m  — extra push below object center for secure grasp

# Safe retracted XYZ — arm pulled BACK and UP before approaching table
SAFE_RETRACT    = np.array([0.20, -0.30, SAFE_Z])

# ─── Gripper commands (Robotiq 85 position actuator) ──────────────────────────
GRIPPER_OPEN    =  0.5    # positive opens the gripper
GRIPPER_CLOSE   = -0.5    # negative/zero closes the gripper


class G1PickPlaceFSM:
    """
    One-object pick-and-place FSM for G1 right arm.

    Call `step(data, tcp_pos)` every sim step.
    Returns (target_xyz, gripper_cmd, state_name).
    """

    def __init__(self):
        # Start with SAFE_INIT to retract arm before approaching table
        self.state        = "SAFE_INIT"
        self.action_start = None
        self.hover_start  = None
        self.lift_anchor  = None
        self.gripper_cmd  = GRIPPER_OPEN
        print(">>> [FSM] SAFE_INIT — retracting arm before approach")

    def step(self, data, tcp_pos):
        """
        Args:
            data    : mujoco.MjData (live sim data)
            tcp_pos : np.ndarray (3,) — current pinch_site world position

        Returns:
            target_xyz   : np.ndarray (3,)
            gripper_cmd  : float
            state        : str
        """
        # Read object/container positions from sim (matches UR5e style)
        box_pos = data.body("target_box").xpos.copy()
        bin_pos = data.body("container").xpos.copy()
        sim_t   = float(data.time)

        # ── SAFE_INIT: retract arm HIGH and BACK before any table motion ──────
        # This prevents the arm from sweeping through the table surface during
        # the initial joint-space → Cartesian transition.
        if self.state == "SAFE_INIT":
            self.gripper_cmd = GRIPPER_OPEN
            target_xyz = SAFE_RETRACT
            # Wait until arm has reached the safe zone (z high enough, x retracted)
            if tcp_pos[2] > SAFE_Z - 0.06 and tcp_pos[0] < 0.35:
                self._go("APPROACH", sim_t)
                print(">>> [FSM] Arm retracted safely. Starting APPROACH.")

        # ── APPROACH: move to hover height above cube ─────────────────────────
        elif self.state == "APPROACH":
            self.gripper_cmd = GRIPPER_OPEN
            target_xyz = np.array([box_pos[0], box_pos[1], LIFT_Z])
            xy_err = np.linalg.norm(tcp_pos[:2] - target_xyz[:2])
            z_err  = abs(tcp_pos[2] - LIFT_Z)
            if xy_err < 0.03 and z_err < 0.04:
                self._go("HOVER", sim_t)
                self.hover_start = sim_t
                print(f">>> [FSM] Aligning above cube  (xy={xy_err*100:.1f}cm)")

        # ── HOVER: wait for XY to settle ─────────────────────────────────────
        elif self.state == "HOVER":
            target_xyz = np.array([box_pos[0], box_pos[1], LIFT_Z])
            xy_err = np.linalg.norm(tcp_pos[:2] - box_pos[:2])
            settled = (sim_t - self.hover_start) > HOVER_SETTLE
            if xy_err < XY_HOVER_THRESH and settled:
                self._go("DESCEND", sim_t)
                print(f">>> [FSM] Descending  (xy_err={xy_err*100:.1f}cm)")

        # ── DESCEND: lower onto the cube ──────────────────────────────────────
        elif self.state == "DESCEND":
            grasp_z = box_pos[2] + DESCEND_OFFSET
            target_xyz = np.array([box_pos[0], box_pos[1], grasp_z])
            xy_err = np.linalg.norm(tcp_pos[:2] - box_pos[:2])
            z_err  = abs(tcp_pos[2] - grasp_z)
            if xy_err < XY_HOVER_THRESH and z_err < Z_GRASP_THRESH:
                self._go("GRASP", sim_t)
                self.action_start = sim_t
                self.lift_anchor  = np.array([box_pos[0], box_pos[1], grasp_z])
                print(f">>> [FSM] Grasping  (xy={xy_err*100:.1f}cm  z={z_err*100:.1f}cm)")

        # ── GRASP: close gripper, hold position ───────────────────────────────
        elif self.state == "GRASP":
            self.gripper_cmd = GRIPPER_CLOSE
            target_xyz = self.lift_anchor.copy()
            if (sim_t - self.action_start) > GRASP_TIME:
                self._go("LIFT", sim_t)
                print(">>> [FSM] Lifting...")

        # ── LIFT: raise arm to carry height ──────────────────────────────────
        elif self.state == "LIFT":
            self.gripper_cmd = GRIPPER_CLOSE
            target_xyz = np.array([self.lift_anchor[0], self.lift_anchor[1], LIFT_Z])
            if tcp_pos[2] > LIFT_Z - 0.04:
                self._go("CARRY", sim_t)
                print(">>> [FSM] Carrying to container...")

        # ── CARRY: move horizontally above container ──────────────────────────
        elif self.state == "CARRY":
            self.gripper_cmd = GRIPPER_CLOSE
            target_xyz = np.array([bin_pos[0], bin_pos[1], BIN_OVER_Z])
            xy_err = np.linalg.norm(tcp_pos[:2] - target_xyz[:2])
            if xy_err < 0.035:
                self._go("DROP", sim_t)
                self.action_start = sim_t
                print(f">>> [FSM] Dropping  (xy_err={xy_err*100:.1f}cm)")

        # ── DROP: open gripper, descend slightly ─────────────────────────────
        elif self.state == "DROP":
            self.gripper_cmd = GRIPPER_OPEN
            target_xyz = np.array([bin_pos[0], bin_pos[1], DROP_Z])
            if (sim_t - self.action_start) > RELEASE_TIME:
                self._go("RETREAT", sim_t)

        # ── RETREAT: lift back up ─────────────────────────────────────────────
        elif self.state == "RETREAT":
            self.gripper_cmd = GRIPPER_OPEN
            target_xyz = np.array([bin_pos[0], bin_pos[1], LIFT_Z])
            if tcp_pos[2] > LIFT_Z - 0.04:
                self._go("HOME", sim_t)
                print(">>> [FSM] Returning home. Task complete! ✓")

        # ── HOME: retract to a safe rest position ────────────────────────────
        elif self.state == "HOME":
            self.gripper_cmd = GRIPPER_OPEN
            target_xyz = np.array([0.30, -0.25, LIFT_Z])

        else:
            target_xyz = np.array([0.30, -0.25, LIFT_Z])

        return target_xyz, self.gripper_cmd, self.state

    def _go(self, next_state, sim_t):
        self.state = next_state
        self.action_start = sim_t
