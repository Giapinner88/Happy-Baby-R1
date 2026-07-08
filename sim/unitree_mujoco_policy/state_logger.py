"""
state_logger.py - Lightweight non-blocking state logger for MuJoCo R1 simulation.

Architecture:
  - Control loop: calls logger.log() → queue.put_nowait()  [~100ns, non-blocking]
  - Background thread: drains the queue and writes rows to CSV [no impact on control]
  - On exit (Ctrl+C): flush remaining queue → close file → print summary

Filename format: {script_stem}_{HH-MM-SS}_{YYYY-MM-DD}.csv
  → Sorting by name puts newest (largest hour) first within the same day,
    and files are further distinguishable by date suffix.

Replay:
  - Column "target_q_{joint}" stores the exact PD-commanded position sent to motors.
  - Load with: df = pd.read_csv(path); replay by sending df["target_q_*"] row-by-row.
"""

import atexit
import csv
import queue
import threading
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Joint name mapping (index 0-28 matches R1 motor_state order)
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    "left_hip_pitch",    "left_hip_roll",    "left_hip_yaw",
    "left_knee",         "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch",   "right_hip_roll",   "right_hip_yaw",
    "right_knee",        "right_ankle_pitch","right_ankle_roll",
    "waist_roll",        "waist_yaw",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
    "left_elbow",        "left_wrist_roll",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    "right_elbow",       "right_wrist_roll",
]  # 24 joints total

def _make_header(joint_names):
    return (
        ["step", "t_sec"]
        + [f"target_q_{n}" for n in joint_names]
        + [f"q_{n}" for n in joint_names]
        + [f"dq_{n}" for n in joint_names]
        + [f"action_{n}" for n in joint_names]
        + ["imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z"]
        + ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]
        + ["proj_grav_x", "proj_grav_y", "proj_grav_z"]
        + ["base_x", "base_y", "base_z"]
        + ["base_vx", "base_vy", "base_vz"]
        + ["cmd_vx", "cmd_vy", "cmd_yaw"]
        + ["gait_sin", "gait_cos", "gait_scale", "gait_time"]
    )


class SimStateLogger:
    """
    Non-blocking CSV logger.

    Example usage:
        logger = SimStateLogger(__file__, joint_names=config.MOTOR_JOINT_NAMES)
        try:
            while True:
                ...
                logger.log(step, t, target_q_arr, q_current, dq_current,
                           action, quat, gyro, proj_grav,
                           smoothed_commands, gait_phase, gait_scale, gait_time)
        except KeyboardInterrupt:
            logger.close()
    """

    # Where logs are saved: repo-level data directory to avoid writing into third_party
    # e.g. <repo_root>/data/sim_state_logs/
    _LOG_DIR = Path(__file__).resolve().parents[2] / "data" / "sim_state_logs"

    def __init__(self, script_path: str, joint_names=None):
        self._LOG_DIR.mkdir(parents=True, exist_ok=True)
        self._joint_names = list(joint_names or JOINT_NAMES)
        self._header = _make_header(self._joint_names)

        # ── Build filename: scriptname_HH-MM-SS_YYYY-MM-DD.csv ──────────────
        now   = datetime.now()
        stem  = Path(script_path).stem
        fname = f"{stem}_{now.strftime('%H-%M-%S_%Y-%m-%d')}.csv"
        self._path = self._LOG_DIR / fname

        # ── Internal state ───────────────────────────────────────────────────
        self._q:      queue.Queue = queue.Queue()   # thread-safe row buffer
        self._closed: bool        = False
        self._rows:   int         = 0               # written row counter
        self._legacy_step: int     = 0

        # ── Start background writer thread ────────────────────────────────────
        self._thread = threading.Thread(
            target=self._writer_loop, name="csv-writer", daemon=True
        )
        self._thread.start()
        atexit.register(self.close)

        print(f"[state-log] logging → {self._path.name}")

    # ── Public API (called from control loop) ────────────────────────────────

    def log(
        self,
        step:       int,
        t:          float,
        target_q,           # np.ndarray[29] – commanded PD target (for replay)
        q,                  # np.ndarray[29] – actual joint positions
        dq,                 # np.ndarray[29] – actual joint velocities
        action,             # np.ndarray[29] – raw RL network output
        quat,               # np.ndarray[4]  – IMU quaternion [w,x,y,z]
        gyro,               # np.ndarray[3]  – IMU gyroscope
        proj_grav,          # np.ndarray[3]  – projected gravity vector
        base_pos,           # np.ndarray[3]  – base/world position
        base_vel,           # np.ndarray[3]  – base/world velocity
        commands,           # np.ndarray[3]  – [vx, vy, yaw] smoothed
        gait_phase,         # np.ndarray[2]  – [sin, cos]
        gait_scale: float,
        gait_time:  float,
    ) -> None:
        """Put one data row onto the queue. O(1), ~100ns — safe inside control loop."""
        if self._closed:
            return
        # Build row as plain Python list (avoids numpy overhead in writer thread)
        row = (
            [step, round(t, 6)]
            + target_q.tolist()
            + q.tolist()
            + dq.tolist()
            + action.tolist()
            + quat.tolist()
            + gyro.tolist()
            + proj_grav.tolist()
            + base_pos.tolist()
            + base_vel.tolist()
            + commands.tolist()
            + gait_phase.tolist()
            + [round(gait_scale, 6), round(gait_time, 6)]
        )
        self._q.put_nowait(row)

    def log_low_state(
        self,
        robot_state,
        q,
        dq,
        imu_quat=None,
        imu_gyro=None,
        timestamp_s=None,
    ) -> None:
        """Compatibility wrapper for older controller scripts.

        Prefer log(...) in new code because it records commanded targets and
        policy outputs explicitly. This method keeps older scripts runnable.
        """
        if imu_quat is None:
            imu_quat = getattr(robot_state.imu_state, "quaternion", [1.0, 0.0, 0.0, 0.0])
        if imu_gyro is None:
            imu_gyro = getattr(robot_state.imu_state, "gyroscope", [0.0, 0.0, 0.0])

        try:
            import numpy as np

            quat = np.array(imu_quat, dtype=np.float32)
            gyro = np.array(imu_gyro, dtype=np.float32)
            zeros_3 = np.zeros(3, dtype=np.float32)
            nan_3 = np.full(3, np.nan, dtype=np.float32)
            zeros_2 = np.zeros(2, dtype=np.float32)
            motor_count = len(self._joint_names)
            zeros_motor = np.zeros(motor_count, dtype=np.float32)
            q_arr = np.array(q, dtype=np.float32)
            dq_arr = np.array(dq, dtype=np.float32)
            w, x, y, z = quat
            proj_grav = np.array([
                2 * (w * y - x * z),
                -2 * (y * z + w * x),
                2 * (x**2 + y**2) - 1,
            ], dtype=np.float32)
            self.log(
                step=self._legacy_step,
                t=float(timestamp_s or 0.0),
                target_q=q_arr,
                q=q_arr,
                dq=dq_arr,
                action=zeros_motor,
                quat=quat,
                gyro=gyro,
                proj_grav=proj_grav,
                base_pos=nan_3,
                base_vel=nan_3,
                commands=zeros_3,
                gait_phase=zeros_2,
                gait_scale=0.0,
                gait_time=0.0,
            )
            self._legacy_step += 1
        except Exception:
            return

    def close(self) -> None:
        """Flush remaining rows and close the file. Call once on program exit."""
        if self._closed:
            return
        self._closed = True
        self._q.put(None)          # sentinel → tells writer to stop
        self._thread.join(timeout=30)
        print(f"[state-log] saved {self._rows:,} rows → {self._path}")

    # ── Background writer (runs in daemon thread) ─────────────────────────────

    def _writer_loop(self) -> None:
        """Drain queue and write to CSV. Runs entirely off the control-loop thread."""
        with open(self._path, "w", newline="", buffering=65536) as f:
            writer = csv.writer(f)
            writer.writerow(self._header)
            while True:
                row = self._q.get()      # blocks here — no CPU waste
                if row is None:          # sentinel: flush OS buffer and exit
                    f.flush()
                    break
                writer.writerow(row)
                self._rows += 1
