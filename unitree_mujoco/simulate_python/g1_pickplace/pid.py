import numpy as np

# ─── Default PID gains (right arm, 7 joints) ─────────────────────────────────
# Order: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
#        wrist_roll, wrist_pitch, wrist_yaw
DEFAULT_KP = [60.0,  60.0,  50.0,  50.0,  15.0,  15.0,  15.0]
DEFAULT_KI = [ 0.3,   0.3,   0.2,   0.2,   0.05,  0.05,  0.05]
DEFAULT_KD = [ 6.0,   6.0,   5.0,   5.0,   2.0,   2.0,   2.0]


class G1ArmPIDTorque:
    """
    Joint-space PID torque controller + gravity feed-forward for G1 arm.

    Usage::

        pid = G1ArmPIDTorque(dt=model.opt.timestep)
        tau = pid.compute_torque(q_target, q_current, qd_current, gravity)
        data.ctrl[arm_act_ids] = np.clip(tau, tau_low, tau_high)
    """

    def __init__(self, dt,
                 kp=None, ki=None, kd=None,
                 integral_limit=30.0,
                 n_joints=7):
        self.kp = np.array(kp if kp is not None else DEFAULT_KP, dtype=float)
        self.ki = np.array(ki if ki is not None else DEFAULT_KI, dtype=float)
        self.kd = np.array(kd if kd is not None else DEFAULT_KD, dtype=float)
        self.dt = float(dt)
        self.integral_limit = float(integral_limit)
        self.error_integral = np.zeros(n_joints)

    def compute_torque(self, q_target, q_current, qd_current, gravity_terms):
        """
        Compute joint torques.

        Args:
            q_target      (7,): desired joint angles  [rad]
            q_current     (7,): actual joint angles   [rad]
            qd_current    (7,): actual joint velocities [rad/s]
            gravity_terms (7,): data.qfrc_bias[dof_ids]

        Returns:
            tau (7,): joint torques [N·m]
        """
        error = q_target - q_current

        # Integral with anti-windup clamp
        self.error_integral += error * self.dt
        self.error_integral = np.clip(
            self.error_integral, -self.integral_limit, self.integral_limit
        )

        # D-term: pure velocity damping (no noisy finite-difference)
        error_d = -qd_current

        tau_pid = (
            self.kp * error
            + self.ki * self.error_integral
            + self.kd * error_d
        )
        return tau_pid + gravity_terms

    def reset(self):
        """Reset integral state (call when changing task or after pause)."""
        self.error_integral[:] = 0.0
