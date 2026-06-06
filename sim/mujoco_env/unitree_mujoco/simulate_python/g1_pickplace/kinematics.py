import mujoco
import numpy as np


# ─── G1-specific null-space target (right arm) ──────────────────────────────
# shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw
Q_REST = np.array([0.0, -0.35, 0.0, 1.2, 0.0, 0.0, 0.0])
# Explanation:
#   shoulder_roll = -0.35  → arm angles slightly out (avoid self-collision)
#   elbow         =  1.2   → "elbow-up" grasping posture


def solve_ik(model, data, target_pos, qpos_ids, dof_ids, site_name="pinch_site"):
    try:
        ee_id      = model.site(site_name).id
        current_pos = data.site(ee_id).xpos.copy()

        err_pos = target_pos - current_pos
        pos_norm = float(np.linalg.norm(err_pos))

        # Saturate position error to avoid huge first steps
        MAX_ERR = 0.12
        if pos_norm > MAX_ERR:
            err_pos = err_pos * (MAX_ERR / pos_norm)

        # ── Jacobian (3 × nv), extract arm DOF columns ─────────────────────
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, jacr, ee_id)
        J = jacp[:, dof_ids]        # 3 × 7

        # ── Adaptive damping via manipulability ──────────────────────────────
        JJT   = J @ J.T             # 3 × 3
        manip = float(np.sqrt(max(0.0, np.linalg.det(JJT))))
        if manip < 0.008:
            lam = 0.20
        elif manip < 0.02:
            lam = 0.08
        elif manip < 0.05:
            lam = 0.03
        else:
            lam = 0.01

        # ── DLS pseudo-inverse: J^† = Jᵀ (J Jᵀ + λ²I)⁻¹  (size 7×3) ──────
        lam2I = lam ** 2 * np.eye(3)
        J_dls = J.T @ np.linalg.solve(JJT + lam2I, np.eye(3))

        # Primary task: track target position
        dq_primary = J_dls @ err_pos   # 7,

        # ── Null-space posture control ────────────────────────────────────────
        q_current = np.array([data.qpos[i] for i in qpos_ids])
        # Disable posture when very close to target (avoids twitching during grasp)
        k_posture = 0.0 if pos_norm < 0.04 else 0.04
        dq_posture = k_posture * (Q_REST - q_current)

        I_n    = np.eye(len(dof_ids))
        N      = I_n - J_dls @ J              # null-space projector 7×7
        dq_null = N @ dq_posture               # 7,

        dq_total = dq_primary + dq_null

        # ── DLS velocity deviation diagnostic ─────────────────────────────────
        dls_dev_pos = float(np.linalg.norm(err_pos - J @ dq_primary))

        info = {
            "manip":       manip,
            "lam":         lam,
            "pos_norm":    pos_norm,
            "dq_norm":     float(np.linalg.norm(dq_total)),
            "dls_dev_pos": dls_dev_pos,
        }

        q_new = q_current + dq_total
        return q_new, info

    except Exception as exc:
        print(f"[IK] Warning: {exc}")
        q_current = np.array([data.qpos[i] for i in qpos_ids])
        info = {"manip": 0.0, "lam": 0.0, "pos_norm": 0.0,
                "dq_norm": 0.0, "dls_dev_pos": 0.0}
        return q_current, info
