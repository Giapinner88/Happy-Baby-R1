#!/usr/bin/env python3
"""Stream joint targets by solving Quest wrist poses with the upstream IK.

This is the online counterpart to `solve_r1_baseline_upstream_ik.py`. It reads
`R1TeleopCommand` JSON lines on stdin, solves each one with `xr_teleoperate`'s
`R1_A5_ArmIK` exactly as the vendor ships it, and writes one joint-target JSON
line per command on stdout.

Splitting IK away from the robot side is upstream's own arrangement, not an
invention here: the solver runs beside the headset reader and whatever drives
the robot receives joint angles it only has to apply. That is also what the
hardware sidecar wants, since the robot side then needs neither CasADi nor a
kinematic model. It is required in practice too -- CasADi and the Pinocchio 3
CasADi bindings live in the `tv` environment, while the Isaac environment has
Pinocchio 2.7 without them, and upgrading a working simulator environment to
suit the solver is a worse trade than passing joint angles between processes.

Commands are parsed with plain `json` rather than this repository's schema
class on purpose. Upstream ships a package also named `teleop`; importing both
in one interpreter means one of them stops resolving. Reading the wire format
directly keeps this process free of that collision, and the format is stable
enough to parse in a dozen lines.

The vendor wrapper emits wrists relative to the *current* head position and
yaw. Before solving, this process reverses that transform and expresses both
wrists relative to the session's initial head anchor. Moving the head therefore
does not move an otherwise stationary controller target. The resulting poses
remain in `neutral_waist_yaw_link`: the vendor's `r1_a5.urdf` gives
`waist_yaw_joint` a zero origin, so its root frame and the waist frame coincide.

Head pitch and yaw are taken from the recorded head pose because upstream's
reduced model locks both head joints along with `waist_yaw_joint`. The start of
the first sustained deadman press supplies both the wrist anchor and the head
angular neutral. Requiring a few consecutive samples avoids calibrating against
a one-frame startup pulse.

The robot-local launcher selects body_relative: head translation follows the
operator while the calibrated yaw defines forward. Its responsive filter uses
two samples. The default replay route retains the initial fixed head anchor
and vendor filter. Neither route solves while the deadman is released; the H4
passthrough emits explicit disabled frames, not an active hold target.

Emits joint targets only. Whether those reach a simulator or hardware is the
caller's decision, and this process opens no robot transport itself.
"""

from __future__ import annotations

import argparse
import atexit
import signal
import hashlib
import json
import os
import queue
import re
import sys
import threading
import time
import types
from collections import deque
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
UPSTREAM_ROOT = REPO_ROOT / "third_party" / "xr_teleoperate_r1_a5_845b25b"
CONTRACT_PATH = REPO_ROOT / "config" / "vendor_r1_a5_845b25b.json"

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint",
)
HEAD_JOINT_NAMES = ("head_pitch_joint", "head_yaw_joint")
DEFAULT_URDF = REPO_ROOT / "src" / "assets" / "R1.urdf"
HEAD_NEUTRAL_CONFIRM_SAMPLES = 3
VENDOR_HEAD_TO_WAIST_OFFSET_M = np.array([0.15, 0.0, 0.45])


class SolveTimingStats:
    """Exact lifetime count/mean/max with bounded recent quantile samples."""
    def __init__(self):
        self.samples = deque(maxlen=4096)
        self.count = 0
        self.total_ms = 0.0
        self.max_ms = 0.0

    def append(self, value):
        self.samples.append(value)
        self.count += 1
        self.total_ms += value
        self.max_ms = max(self.max_ms, value)


def head_limits_rad(urdf_path: Path) -> tuple[tuple[float, float], ...]:
    """Head joint bounds read from the asset rather than transcribed.

    The vendor reduced model locks both head joints, so this process is the only
    place they are bounded and a stale hand-copied pair would silently command
    past the asset. Read with a regex because importing this repository's
    kinematics would drag in the `teleop` package this process must avoid.
    """

    text = urdf_path.read_text(encoding="utf-8")
    limits: list[tuple[float, float]] = []
    for name in HEAD_JOINT_NAMES:
        block = re.search(
            rf'<joint name="{name}".*?<limit[^>]*lower="([-0-9.eE+]+)"[^>]*upper="([-0-9.eE+]+)"',
            text,
            re.DOTALL,
        )
        if block is None:
            raise SystemExit(f"{urdf_path} declares no limits for {name}")
        limits.append((float(block.group(1)), float(block.group(2))))
    return tuple(limits)


def pose_to_matrix(pose: dict) -> np.ndarray:
    orientation = pose["orientation"]
    x, y, z, w = (
        float(orientation["x"]), float(orientation["y"]),
        float(orientation["z"]), float(orientation["w"]),
    )
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm <= 0.0:
        raise ValueError("pose carries a zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    position = pose["position"]
    transform[:3, 3] = [float(position["x"]), float(position["y"]), float(position["z"])]
    return transform


def neck_angles_from_rotation(rotation: np.ndarray, previous_pitch: float | None = None) -> tuple[float, float]:
    """Aim the forward axis using R1's Ry(pitch) @ Rz(yaw) joint order.

    Pitch stays within +/-90 degrees; the sign of forward X selects the yaw
    branch beyond 90 degrees. At the singular side-facing pose use the full
    rotation for pitch. Head roll does not change the forward direction.
    """
    forward = np.asarray(rotation, dtype=float)[:, 0]
    forward = forward / np.linalg.norm(forward)
    horizontal = float(np.hypot(forward[0], forward[2]))
    if horizontal < 1e-7:
        pitch = (float(previous_pitch) if previous_pitch is not None else
                 float(np.arctan2(rotation[0, 2], rotation[2, 2])))
        yaw = float(np.copysign(np.pi / 2, forward[1]))
    else:
        branch = -1.0 if forward[0] < 0 else 1.0
        yaw = float(np.arctan2(forward[1], branch * horizontal))
        pitch = float(np.arctan2(-branch * forward[2], branch * forward[0]))
    return pitch, yaw


def _head_angles_unbounded(pose: dict) -> tuple[float, float]:
    return neck_angles_from_rotation(pose_to_matrix(pose)[:3, :3])


def head_angles(pose: dict, limits: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    pitch, yaw = _head_angles_unbounded(pose)
    return float(np.clip(pitch, *limits[0])), float(np.clip(yaw, *limits[1]))


def relative_head_angles(pose: dict, neutral_matrix: np.ndarray,
                         limits: tuple[tuple[float, float], ...],
                         previous_pitch: float | None = None) -> tuple[float, float]:
    """Remove the neutral rotation before extracting both neck joint angles."""
    rotation = np.asarray(neutral_matrix)[:3, :3].T @ pose_to_matrix(pose)[:3, :3]
    pitch, yaw = neck_angles_from_rotation(rotation, previous_pitch)
    return float(np.clip(pitch, *limits[0])), float(np.clip(yaw, *limits[1]))


def head_yaw_rotation(head_pose_matrix: np.ndarray) -> np.ndarray:
    """Mirror the vendor's horizontal-head-axis yaw extraction exactly."""

    forward = np.asarray(head_pose_matrix[:3, 0], dtype=float).copy()
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if not np.isfinite(norm) or norm <= 1e-6:
        return np.eye(3)
    forward /= norm
    up = np.array([0.0, 0.0, 1.0])
    left = np.cross(up, forward)
    left /= np.linalg.norm(left)
    return np.column_stack((forward, left, up))


def reanchor_wrist_matrix(
    wrist_pose_matrix: np.ndarray,
    current_head_pose_matrix: np.ndarray,
    anchor_head_pose_matrix: np.ndarray,
) -> np.ndarray:
    """Move a vendor current-head-relative wrist pose to the initial head anchor.

    TeleVuer emits ``A_t = yaw(H_t)^-1 * W`` plus its fixed waist offset.
    Recovering ``W`` with the current head and applying ``yaw(H_0)^-1`` removes
    head motion without changing the vendor wrist convention or workspace
    offset.
    """

    wrist = np.asarray(wrist_pose_matrix, dtype=float)
    current_head = np.asarray(current_head_pose_matrix, dtype=float)
    anchor_head = np.asarray(anchor_head_pose_matrix, dtype=float)
    current_yaw = head_yaw_rotation(current_head)
    anchor_yaw = head_yaw_rotation(anchor_head)

    world_wrist_rotation = current_yaw @ wrist[:3, :3]
    world_wrist_position = (
        current_yaw @ (wrist[:3, 3] - VENDOR_HEAD_TO_WAIST_OFFSET_M)
        + current_head[:3, 3]
    )

    anchored = np.eye(4)
    anchored[:3, :3] = anchor_yaw.T @ world_wrist_rotation
    anchored[:3, 3] = (
        anchor_yaw.T @ (world_wrist_position - anchor_head[:3, 3])
        + VENDOR_HEAD_TO_WAIST_OFFSET_M
    )
    return anchored


def body_relative_wrist_matrix(wrist: np.ndarray, head: np.ndarray,
                               anchor: np.ndarray) -> np.ndarray:
    """Follow head translation while retaining the calibrated forward direction.

    Head yaw is removed by reanchor_wrist_matrix; moving the operator through
    the room must not become an extra robot hand displacement.
    """
    moving_anchor = anchor.copy()
    moving_anchor[:3, 3] = head[:3, 3]
    return reanchor_wrist_matrix(wrist, head, moving_anchor)


class HeadNeutralCalibrator:
    """Capture one stable session neutral and hold the head with deadman off."""

    def __init__(
        self,
        limits: tuple[tuple[float, float], ...],
        confirm_samples: int = HEAD_NEUTRAL_CONFIRM_SAMPLES,
    ) -> None:
        if confirm_samples < 1:
            raise ValueError("confirm_samples must be positive")
        self.limits = limits
        self.confirm_samples = confirm_samples
        self.neutral: tuple[float, float] | None = None
        self.anchor_pose_matrix: np.ndarray | None = None
        self.candidate_count = 0
        self.last = (0.0, 0.0)

    @property
    def ready(self) -> bool:
        return self.anchor_pose_matrix is not None

    def update(self, pose: dict, enabled: bool) -> tuple[float, float]:
        if not enabled:
            if self.neutral is None:
                self.candidate_count = 0
            return self.last

        if self.neutral is None:
            self.candidate_count += 1
            self.last = (0.0, 0.0)
            if self.candidate_count >= self.confirm_samples:
                self.neutral = _head_angles_unbounded(pose)
                self.anchor_pose_matrix = pose_to_matrix(pose)
            return self.last

        self.last = relative_head_angles(pose, self.anchor_pose_matrix, self.limits, self.last[0])
        return self.last


def verify_vendor_artifacts() -> None:
    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    for artifact in contract["artifacts"].values():
        path = UPSTREAM_ROOT / artifact["path"]
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != artifact["sha256"]:
            raise SystemExit(f"vendor artifact absent or hash mismatch: {path}")
    meshes = sorted((UPSTREAM_ROOT / "assets" / "r1" / "meshes").iterdir())
    manifest = "".join(
        f"{path.name} {hashlib.sha256(path.read_bytes()).hexdigest()}\n"
        for path in meshes if path.is_file()
    )
    if (len(meshes) != contract["mesh_count"] or
            hashlib.sha256(manifest.encode()).hexdigest() != contract["mesh_tree_sha256"]):
        raise SystemExit("vendor R1-A5 mesh tree is missing or changed")


def load_solver():
    verify_vendor_artifacts()
    # The vendor cache has no source/URDF signature. Rebuild the model from the
    # checked artifacts for every session so an old pickle cannot bypass them.
    (UPSTREAM_ROOT / "teleop" / "robot_control" / "r1_a5_model_cache.pkl").unlink(missing_ok=True)
    # Upstream resolves its URDF, meshes and model cache relative to the working
    # directory, so the process moves there rather than editing vendor paths.
    os.chdir(UPSTREAM_ROOT / "teleop" / "robot_control")
    sys.path.insert(0, str(UPSTREAM_ROOT))
    sys.path.insert(0, str(UPSTREAM_ROOT / "teleop"))
    # The HB source tree also has a regular ``teleop`` package.  The pinned
    # vendor tree intentionally has no package marker, so Python would pick HB
    # first and fail at ``teleop.utils``.  Install a short-lived namespace
    # package for this process only; no vendor file is modified and the vendor
    # solver remains the only IK implementation loaded here.
    vendor_pkg = types.ModuleType("teleop")
    vendor_pkg.__path__ = [str(UPSTREAM_ROOT / "teleop")]
    vendor_pkg.__package__ = "teleop"
    sys.modules["teleop"] = vendor_pkg
    from robot_control.robot_arm_ik import R1_A5_ArmIK

    return R1_A5_ArmIK(Unit_Test=True, Visualization=False)


def compact_live_commands(lines):
    """Keep the latest pose plus the latest reset and release, in source order."""
    keep = {len(lines) - 1}
    if lines and lines[-1] is None and len(lines) > 1:
        keep.add(len(lines) - 2)
    last_reset = last_release = None
    first_invalid = None
    for index, line in enumerate(lines):
        if line is None:
            continue
        try:
            command = json.loads(line)
            if not isinstance(command, dict):
                raise ValueError('command must be an object')
        except (ValueError, TypeError):
            # Keep the first fault for the consumer, without accumulating
            # every malformed line while the solver is busy.
            if first_invalid is None:
                first_invalid = index
            continue
        if command.get("reset_requested"):
            last_reset = index
        if command.get("deadman_enabled") is False:
            last_release = index
    keep.update(i for i in (last_reset, last_release, first_invalid) if i is not None)
    return [lines[i] for i in sorted(keep)]


def latest_live_lines(source):
    """Yield only the newest live command after each IK solve.

    The vendor solver is intentionally run synchronously, and on ARM64 it is
    slower than the 30 Hz Quest sample stream.  Reading stdin directly would
    build an ever older backlog; by the time a line reached the solver it
    already violated ``max-command-age-s``.  A one-slot mailbox keeps the
    deadman state and pose current without replaying stale poses.  File replay
    remains on the ordinary iterator path so evidence tools stay deterministic.
    """

    mailbox = queue.Queue(maxsize=1)

    def offer(value: str | None) -> None:
        batch = [value]
        while True:
            try:
                mailbox.put_nowait(batch)
                return
            except queue.Full:
                try:
                    batch = compact_live_commands(mailbox.get_nowait() + batch)
                except queue.Empty:
                    pass

    def reader() -> None:
        for line in source:
            if line.strip():
                offer(line)
        offer(None)

    threading.Thread(target=reader, name="ik-live-reader", daemon=True).start()
    while True:
        for line in mailbox.get():
            if line is None:
                return
            yield line


def solve_tracked_arms(solver, left_wrist, right_wrist, tracking):
    """Fresh vendor solve for the active hand; inactive arm uses a FK hold target.

    The sidecar independently holds inactive motors at fresh encoder positions.
    The FK target here only keeps the two-effector vendor optimization well posed;
    it is not an observation from a missing Quest controller.
    """
    if (not isinstance(tracking, dict) or
            any(type(tracking.get(side)) is not bool for side in ('left', 'right')) or
            not tracking['right']):
        raise ValueError('enabled command requires fresh right-arm tracking')
    if not tracking['left']:
        import pinocchio as pin
        model = solver.reduced_robot.model
        data = model.createData()
        seed = np.asarray(solver.init_data, dtype=float).copy()
        if seed.shape != (10,) or not np.all(np.isfinite(seed)):
            raise ValueError('invalid vendor seed for inactive arm')
        pin.framesForwardKinematics(model, data, seed)
        left_wrist = np.asarray(data.oMf[solver.L_hand_id].homogeneous).copy()
    solution, torque = solver.solve_ik(left_wrist, right_wrist)
    return solution, torque, left_wrist, right_wrist


def shutdown_logging_listener():
    """The pinned logging-mp listener ignores SIGTERM: close its queue first."""
    module = sys.modules.get("logging_mp")
    manager = getattr(module, "_internal_manager", None)
    child = getattr(manager, "_listener_process", None)
    if child is None:
        return
    try:
        manager._log_queue.put_nowait(None)
        child.join(timeout=2)
    finally:
        if child.is_alive():
            child.kill()
            child.join(timeout=1)
        manager._listener_process = None
        manager._is_started = False


def terminate_ik(signum, frame):
    # SystemExit reaches finally and atexit, including while stdin is idle.
    raise SystemExit(0)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--input", type=Path, default=None,
        help="Read commands from this file instead of stdin; useful for replaying a trace.",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Write joint targets here instead of stdout.",
    )
    parser.add_argument(
        "--stats-path", type=Path, default=None,
        help="Write per-run solve timing here on exit.",
    )
    parser.add_argument(
        "--passthrough", action="store_true",
        help=(
            "Emit each original command with the solved joints attached, instead of "
            "joint targets alone. The simulator runner needs the wrist poses to keep "
            "writing its usual evidence, and `R1TeleopCommand` ignores the extra keys, "
            "so the augmented line stays a valid command stream."
        ),
    )
    parser.add_argument("--urdf-path", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--preflight", action="store_true", help="Validate pinned local artifacts and exit.")
    parser.add_argument("--max-command-age-s", type=float, default=0.3)
    parser.add_argument("--wrist-reference", choices=("initial_head", "body_relative"),
                        default="initial_head")
    parser.add_argument("--arm-filter", choices=("vendor", "responsive"), default="vendor",
                        help="responsive uses two samples (0.7/0.3) instead of four.")
    args = parser.parse_args()
    if not np.isfinite(args.max_command_age_s) or args.max_command_age_s <= 0:
        raise SystemExit("--max-command-age-s must be finite and positive")
    if args.preflight:
        verify_vendor_artifacts()
        print("[VENDOR IK PREFLIGHT] pinned artifacts present", file=sys.stderr, flush=True)
        return 0

    # Resolved before the solver loads, because loading it moves the working
    # directory into the vendor tree and would strand any relative path here.
    for name in ("input", "output", "stats_path", "urdf_path"):
        value = getattr(args, name)
        if value is not None:
            setattr(args, name, value.expanduser().resolve())

    head_bounds = head_limits_rad(args.urdf_path)
    signal.signal(signal.SIGTERM, terminate_ik)
    atexit.register(shutdown_logging_listener)
    solver = load_solver()
    if args.arm_filter == "responsive":
        solver.smooth_filter = type(solver.smooth_filter)(np.array([0.7, 0.3]), 10)
    print("[VENDOR IK READY] pinned R1_A5_ArmIK loaded", file=sys.stderr, flush=True)
    joint_names = list(ARM_JOINT_NAMES) + list(HEAD_JOINT_NAMES)

    source = args.input.open("r", encoding="utf-8") if args.input else sys.stdin
    sink = args.output.open("w", encoding="utf-8") if args.output else sys.stdout
    solve_ms = SolveTimingStats()
    held = 0
    calibration_samples_skipped = 0
    input_line_count = 0
    stale_samples_skipped = 0
    last_sequence_id: int | None = None
    last_arms: np.ndarray | None = None
    last_wrist_targets: tuple[np.ndarray, np.ndarray] | None = None
    head_calibrator = HeadNeutralCalibrator(head_bounds)
    reset_requires_release = True

    downstream_closed = False
    failure: str | None = None
    try:
        line_iter = source if args.input else latest_live_lines(source)
        for line in line_iter:
            line = line.strip()
            if not line:
                continue
            command = json.loads(line)
            input_line_count += 1
            source_at = float(command["timestamp_monotonic_s"])
            if not np.isfinite(source_at):
                raise ValueError("Quest source timestamp is not finite")
            if (args.input is None and time.monotonic() - source_at > args.max_command_age_s
                    and command.get("deadman_enabled") and not command.get("reset_requested")):
                stale_samples_skipped += 1
                continue
            sequence_value = command.get("sequence_id")
            last_sequence_id = int(sequence_value) if sequence_value is not None else None
            enabled = bool(command.get("deadman_enabled", False))
            if command.get("reset_requested"):
                head_calibrator = HeadNeutralCalibrator(head_bounds)
                reset_requires_release = True
            elif not enabled:
                reset_requires_release = False
            enabled = enabled and not reset_requires_release
            pitch, yaw = head_calibrator.update(command["head_pose"], enabled)
            if enabled and not head_calibrator.ready:
                calibration_samples_skipped += 1
                enabled = False
            # Carry STOP even before the first solve and throughout reset and
            # calibration. No fabricated zero joint target is needed.
            if not enabled and args.passthrough:
                record = dict(command)
                record["deadman_enabled"] = False
                record["upstream_solved_this_sample"] = False
                sink.write(json.dumps(record) + "\n")
                sink.flush()
                held += 1
                continue
            if enabled:
                assert head_calibrator.anchor_pose_matrix is not None
                current_head = pose_to_matrix(command["head_pose"])
                transform_wrist = (body_relative_wrist_matrix
                                   if args.wrist_reference == "body_relative"
                                   else reanchor_wrist_matrix)
                left_wrist = transform_wrist(
                    pose_to_matrix(command["left_wrist_pose"]),
                    current_head,
                    head_calibrator.anchor_pose_matrix,
                )
                right_wrist = transform_wrist(
                    pose_to_matrix(command["right_wrist_pose"]),
                    current_head,
                    head_calibrator.anchor_pose_matrix,
                )
                started = time.perf_counter()
                arm_tracking = command.get("arm_tracking", {"left": True, "right": True})
                solution, _torque, left_wrist, right_wrist = solve_tracked_arms(
                    solver, left_wrist, right_wrist, arm_tracking)
                solve_ms.append(1000.0 * (time.perf_counter() - started))
                last_arms = np.asarray(solution, dtype=float)
                if last_arms.shape != (10,) or not np.all(np.isfinite(last_arms)):
                    raise ValueError("vendor IK returned an invalid 10-joint solution")
                last_wrist_targets = (left_wrist, right_wrist)
            elif last_arms is None:
                # Nothing solved yet, so there is no pose to hold and emitting a
                # zero vector would be a command, not a hold.
                continue
            else:
                held += 1
            solved = [*(float(v) for v in last_arms), pitch, yaw]
            if args.passthrough:
                # The original command is preserved verbatim so the consumer's
                # own evidence stays exactly what it would have been, and the
                # solved vector rides along under its own keys.
                record = dict(command)
                record["upstream_arm_tracking"] = command.get("arm_tracking", {"left": True, "right": True})
                record["upstream_solver"] = "upstream_xr_teleoperate_R1_A5_ArmIK"
                record["upstream_joint_names"] = joint_names
                record["upstream_joint_position_rad"] = solved
                record["upstream_solved_this_sample"] = enabled
                if enabled:
                    # The Quest timestamp measures capture time, while the
                    # downstream watchdog needs the time the vendor solve
                    # actually became available.  On ARM64 a valid solve can
                    # take close to the 300 ms source lease even after stale
                    # input has already been dropped above.
                    record["upstream_solved_timestamp_monotonic_s"] = time.monotonic()
                record["upstream_wrist_reference_mode"] = args.wrist_reference
                assert last_wrist_targets is not None
                record["upstream_left_wrist_target_matrix"] = last_wrist_targets[0].tolist()
                record["upstream_right_wrist_target_matrix"] = last_wrist_targets[1].tolist()
            else:
                record = {
                    "schema_version": 1,
                    "sequence_id": command.get("sequence_id"),
                    "timestamp_monotonic_s": command.get("timestamp_monotonic_s"),
                    "deadman_enabled": enabled,
                    "solver": "upstream_xr_teleoperate_R1_A5_ArmIK",
                    "joint_names": joint_names,
                    "joint_position_rad": solved,
                }
            try:
                sink.write(json.dumps(record) + "\n")
                sink.flush()
            except BrokenPipeError:
                # Isaac đóng ống trước khi solver kịp nhận ra phiên đã kết thúc.
                # Đó là trình tự tắt máy bình thường của pipeline ba tiến trình,
                # không phải solver hỏng. `quest_bridge.py:313` xử lý y hệt. Đổi
                # stdout sang /dev/null để lượt flush lúc Python thoát không ném
                # thêm một lỗi thứ hai, rồi vẫn ghi thống kê ra file.
                sys.stdout = open(os.devnull, "w", encoding="utf-8")
                downstream_closed = True
                break
    except SystemExit:
        raise
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        shutdown_logging_listener()
        if args.input:
            source.close()
        if args.output:
            sink.close()
        if args.stats_path:
            values = np.asarray(solve_ms.samples, dtype=float)
            solve_summary = None
            implied_rate = None
            if len(values):
                solve_summary = {
                    "mean": solve_ms.total_ms / solve_ms.count,
                    "median": float(np.median(values)),
                    "p95": float(np.quantile(values, 0.95)),
                    "max": solve_ms.max_ms,
                    "quantile_window_sample_count": len(values),
                    "quantile_scope": "most_recent_4096_solves",
                }
                implied_rate = float(1000.0 / solve_summary['mean'])
            args.stats_path.write_text(
                json.dumps(
                    {
                        "status": "failed" if failure else "completed",
                        "failure": failure,
                        "input_line_count": input_line_count,
                        "last_sequence_id": last_sequence_id,
                        "solved_sample_count": solve_ms.count,
                        "held_sample_count": int(held),
                        "calibration_samples_skipped": int(calibration_samples_skipped),
                        "stale_samples_skipped": int(stale_samples_skipped),
                        "wrist_reference_mode": args.wrist_reference,
                        "arm_filter": args.arm_filter,
                        "solve_ms": solve_summary,
                        "implied_rate_ceiling_hz": implied_rate,
                        "head_neutral_pitch_yaw_rad": (
                            list(head_calibrator.neutral) if head_calibrator.neutral else None
                        ),
                        "head_anchor_pose_matrix": (
                            head_calibrator.anchor_pose_matrix.tolist()
                            if head_calibrator.anchor_pose_matrix is not None
                            else None
                        ),
                        "stop_reason": (
                            "exception" if failure
                            else "downstream_closed" if downstream_closed
                            else "input_exhausted"
                        ),
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
