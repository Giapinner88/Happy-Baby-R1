"""
state_logger.py — Lightweight non-blocking state logger for MuJoCo G1 simulation.

Architecture:
  - Control loop: calls logger.log() → queue.put_nowait()  [~100ns, non-blocking]
  - Background thread: drains the queue and writes rows to CSV [no impact on control]
  - On exit (Ctrl+C): flush remaining queue → close file → print summary

Filename format: {script_stem}_{HH-MM}_{YYYY-MM-DD}.csv
  → Sorting by name puts newest (largest hour) first within the same day,
    and files are further distinguishable by date suffix.

Replay:
  - Column "target_q_{joint}" stores the exact PD-commanded position sent to motors.
  - Load with: df = pd.read_csv(path); replay by sending df["target_q_*"] row-by-row.
"""

import csv
import queue
import threading
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Joint name mapping (index 0-28 matches G1 motor_state order)
# ---------------------------------------------------------------------------
JOINT_NAMES = [
    "left_hip_pitch",    "left_hip_roll",    "left_hip_yaw",
    "left_knee",         "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch",   "right_hip_roll",   "right_hip_yaw",
    "right_knee",        "right_ankle_pitch","right_ankle_roll",
    "waist_yaw",         "waist_roll",       "waist_pitch",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
    "left_elbow",        "left_wrist_roll",  "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    "right_elbow",       "right_wrist_roll", "right_wrist_pitch","right_wrist_yaw",
]  # 29 joints total

# Pre-build CSV header once at module load (avoids repeated string ops in hot loop)
_HEADER = (
    ["step", "t_sec"]
    # ── Replay columns ──────────────────────────────────────────────────────
    + [f"target_q_{n}" for n in JOINT_NAMES]   # commanded q → use for replay
    # ── Sensor feedback ─────────────────────────────────────────────────────
    + [f"q_{n}"        for n in JOINT_NAMES]   # actual joint position
    + [f"dq_{n}"       for n in JOINT_NAMES]   # actual joint velocity
    # ── RL policy output ────────────────────────────────────────────────────
    + [f"action_{n}"   for n in JOINT_NAMES]   # raw network output (pre-scale)
    # ── IMU ─────────────────────────────────────────────────────────────────
    + ["imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z"]
    + ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]
    + ["proj_grav_x", "proj_grav_y", "proj_grav_z"]
    # ── Controller state ────────────────────────────────────────────────────
    + ["cmd_vx", "cmd_vy", "cmd_yaw"]
    + ["gait_sin", "gait_cos", "gait_scale", "gait_time"]
)


class SimStateLogger:
    """
    Non-blocking CSV logger.

    Example usage in run98_2.py:
        logger = SimStateLogger(__file__)
        try:
            while True:
                ...
                logger.log(step, t, target_q_arr, q_current, dq_current,
                           action, quat, gyro, proj_grav,
                           smoothed_commands, gait_phase, gait_scale, gait_time)
        except KeyboardInterrupt:
            logger.close()
    """

    # Where logs are saved (relative to this file → simulate_python/sim_state_logs/)
    _LOG_DIR = Path(__file__).parent / "sim_state_logs"

    def __init__(self, script_path: str, joint_names: list = None):
        self._LOG_DIR.mkdir(exist_ok=True)

        if joint_names is None:
            self.joint_names = JOINT_NAMES
        else:
            self.joint_names = joint_names

        # Pre-build CSV header for this instance
        self._header = (
            ["step", "t_sec"]
            + [f"target_q_{n}" for n in self.joint_names]
            + [f"q_{n}"        for n in self.joint_names]
            + [f"dq_{n}"       for n in self.joint_names]
            + [f"action_{n}"   for n in self.joint_names]
            + ["imu_quat_w", "imu_quat_x", "imu_quat_y", "imu_quat_z"]
            + ["imu_gyro_x", "imu_gyro_y", "imu_gyro_z"]
            + ["proj_grav_x", "proj_grav_y", "proj_grav_z"]
            + ["cmd_vx", "cmd_vy", "cmd_yaw"]
            + ["gait_sin", "gait_cos", "gait_scale", "gait_time"]
        )

        # ── Build filename: scriptname_HH-MM_YYYY-MM-DD.csv ─────────────────
        now   = datetime.now()
        stem  = Path(script_path).stem
        fname = f"{stem}_{now.strftime('%H-%M_%Y-%m-%d')}.csv"
        self._path = self._LOG_DIR / fname

        # ── Internal state ───────────────────────────────────────────────────
        self._q:      queue.Queue = queue.Queue()   # thread-safe row buffer
        self._closed: bool        = False
        self._rows:   int         = 0               # written row counter

        # ── Start background writer thread ────────────────────────────────────
        self._thread = threading.Thread(
            target=self._writer_loop, name="csv-writer", daemon=True
        )
        self._thread.start()

        print(f"[state-log] logging → {self._path.name}")

    # ── Public API (called from control loop) ────────────────────────────────

    def log(
        self,
        step:       int,
        t:          float,
        target_q,           # np.ndarray – commanded PD target (for replay)
        q,                  # np.ndarray – actual joint positions
        dq,                 # np.ndarray – actual joint velocities
        action,             # np.ndarray – raw RL network output
        quat,               # np.ndarray[4]  – IMU quaternion [w,x,y,z]
        gyro,               # np.ndarray[3]  – IMU gyroscope
        proj_grav,          # np.ndarray[3]  – projected gravity vector
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
            + commands.tolist()
            + gait_phase.tolist()
            + [round(gait_scale, 6), round(gait_time, 6)]
        )
        self._q.put_nowait(row)

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
