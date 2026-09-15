#!/usr/bin/env python3
"""Quest live commands into an IsaacLab R1 simulator.

This process runs in the `unitree_sim_env` environment and consumes the
newline-delimited `R1TeleopCommand` stream produced by `quest_bridge.py`. It has
no DDS, ROS, Unitree SDK, `LowCmd`, or `hardware/high_level/` import, and the
`HeadOnlyIsaacLabSink` raises if a base-velocity dispatch ever reaches it.

The default is the T001 head-only connectivity pilot. ``--arm-head-config``
selects the legacy T007 independent-arm controller. ``--whole-upper-body-config``
selects the coupled R1-A5 controller (waist yaw, both arms, and head). Both T007
modes fix the root and prohibit locomotion; they are simulation evidence only.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("HAPPY_BABY_R1_ROOT", str(ROOT))

from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_json,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T001/config/r1_quest3_sim_v1.json",
    )
    parser.add_argument("--output-dir", type=Path, required=True, help="New evidence directory; never overwritten.")
    parser.add_argument("--duration-s", type=float, default=180.0, help="Wall-clock limit for the simulator loop.")
    parser.add_argument("--control-hz", type=float, default=50.0, help="Rate at which commands are mapped and applied.")
    parser.add_argument("--physics-hz", type=float, default=200.0, help="Simulation physics rate.")
    parser.add_argument("--video-fps", type=float, default=15.0, help="Frame rate of the recorded evidence video.")
    parser.add_argument("--video-width", type=int, default=640, help="Evidence video width in pixels.")
    parser.add_argument("--video-height", type=int, default=360, help="Evidence video height in pixels.")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--arm-head-config", type=Path,
        help="Enable legacy T007 independent bilateral arm+head simulation.",
    )
    mode.add_argument(
        "--whole-upper-body-config", type=Path,
        help="Enable coupled T007 R1-A5 waist-yaw + bilateral arms + head simulation.",
    )
    mode.add_argument(
        "--offline-joint-trajectory",
        type=Path,
        help="Replay a sequence-indexed offline T007 joint trajectory in Isaac Lab.",
    )
    mode.add_argument(
        "--upstream-joint-stream-config",
        type=Path,
        help=(
            "Apply joint targets solved by the vendor xr_teleoperate IK running upstream "
            "of this process. The command stream must carry upstream_joint_position_rad, "
            "which run_r1_upstream_ik_stream.py --passthrough adds."
        ),
    )
    parser.add_argument(
        "--offline-continuation-config",
        type=Path,
        help="Configuration that produced --offline-joint-trajectory.",
    )
    parser.add_argument(
        "--body-mode",
        choices=("arms_head", "waist_yaw", "full_upper_body"),
        help=(
            "Which torso joints the coupled solver drives, overriding the profile: "
            "'arms_head' freezes the torso and drives both arms + head only (12 joints); "
            "'waist_yaw' adds waist yaw, the hardware-common set (13); "
            "'full_upper_body' also adds waist roll, a simulation-only deviation (14). "
            "Freezing the torso stops it being recruited to chase out-of-reach hand targets."
        ),
    )
    parser.add_argument("--no-video", action="store_true", help="Skip video capture; recorded as missing evidence.")
    parser.add_argument(
        "--dual-view",
        action="store_true",
        help=(
            "Record a second fixed evidence camera on the opposite side and store both views "
            "side by side in one synchronized video. Doubles the recorded frame width."
        ),
    )
    parser.add_argument(
        "--dataset-scene-config",
        type=Path,
        help=(
            "Profile dataset khai báo vật thể trong cảnh và camera gắn trên đầu robot. "
            "Không truyền thì cảnh giữ nguyên như T007: mặt đất, đèn, robot, và không có "
            "camera nào gắn trên robot."
        ),
    )
    parser.add_argument(
        "--strict-dataset-fps",
        action="store_true",
        help=(
            "Dừng sớm một phiên dataset nếu FPS wall-clock không đạt gate trong "
            "profile sau số frame warm-up. Frame camera cũng phải có source id tăng "
            "nghiêm ngặt khi profile yêu cầu; không lặp ảnh để giả đủ tần số."
        ),
    )
    parser.add_argument(
        "--target-digit-priority",
        help=(
            "Danh sách chữ số ngăn cách bởi dấu phẩy phải làm mục tiêu trước, theo đúng thứ tự. "
            "Dùng để bù những chữ số còn thiếu trong dataset đã thu; lấy gợi ý bằng "
            "tools/dataset_report.py --suggest-priority."
        ),
    )
    parser.add_argument(
        "--fixed-digit-layout",
        help=(
            "Bố cục tấm số cố định, ngăn cách bởi dấu phẩy, theo thứ tự trái sang "
            "phải trong góc nhìn robot; ví dụ 5,4,8,2. Bỏ trống để random như cũ."
        ),
    )
    parser.add_argument(
        "--policy-prompt",
        help=(
            "Prompt gửi cho policy. Prompt literal phải chứa đúng một chữ số 1-9 "
            "để tự suy target. Prompt ngữ nghĩa không ghi số, hoặc template {n}, "
            "cần một --target-digit-priority tường minh để lưu ground truth. Bỏ "
            "trống để dùng template trong dataset config."
        ),
    )
    parser.add_argument(
        "--viewport-camera",
        choices=("perspective", "head"),
        default="perspective",
        help=(
            "Camera nào chiếm cửa sổ Isaac. 'head' gán camera gắn trên đầu robot, tức "
            "góc nhìn thứ nhất. Chỉ có tác dụng khi chạy có cửa sổ và có camera đầu."
        ),
    )
    parser.add_argument(
        "--head-view-port",
        type=int,
        help=(
            "Phát khung camera đầu robot qua ZMQ tới cổng này để bridge hiện trong kính. "
            "Không truyền thì không có ảnh nào rời khỏi tiến trình này."
        ),
    )
    parser.add_argument(
        "--policy-obs-port",
        type=int,
        help=(
            "Phát quan sát (ảnh camera đầu + state 12 chiều + câu lệnh) qua ZMQ tới cổng "
            "này cho một policy tự lái. Tách khỏi --head-view-port vì đó là ảnh cho người "
            "vận hành, đã vẽ cue lên; ảnh ở đây là tensor sensor gốc mà dataset đã dạy."
        ),
    )
    parser.add_argument(
        "--render-every-control-step",
        action="store_true",
        help=(
            "Refresh the viewport at every control tick. By default rendering is "
            "decoupled to the evidence-video rate so RTX work does not reduce the "
            "differential-controller update rate."
        ),
    )
    parser.add_argument(
        "--disable-self-collisions",
        action="store_true",
        help=(
            "Spawn the R1 with articulation self-collisions off. The project asset config enables them, "
            "but with them enabled the head joints are mechanically blocked and cannot follow a target. "
            "This is a declared deviation and is recorded in resolved_config.json."
        ),
    )
    parser.add_argument(
        "--idle-stop-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds with no command once the stream has started; 0 disables.",
    )
    parser.add_argument(
        "--stop-file",
        type=Path,
        help="A path that must not exist at startup; create it to request a graceful live stop.",
    )
    parser.add_argument(
        "--replay-command-file",
        type=Path,
        help=(
            "Replay a recorded raw_commands.jsonl stream instead of stdin. "
            "Source timing is preserved while monotonic timestamps are retimed "
            "to this process so watchdog and latency semantics remain valid."
        ),
    )
    parser.add_argument(
        "--replay-speed",
        type=float,
        default=1.0,
        help="Wall-clock replay speed multiplier; 1.0 preserves source timing.",
    )
    parser.add_argument(
        "--position-scale",
        type=float,
        help=(
            "Override differential relative-session wrist translation scale. "
            "The effective value is recorded in resolved_config.json."
        ),
    )
    parser.add_argument(
        "--velocity-feedforward-task",
        choices=("none", "head_only", "wrist_position_head", "all"),
        help=(
            "Override which differential Cartesian tasks use measured target "
            "velocity feedforward; recorded in resolved_config.json."
        ),
    )
    parser.add_argument(
        "--max-joint-velocity-rad-s",
        type=float,
        help="Override the differential controller velocity box limit.",
    )
    parser.add_argument(
        "--max-joint-acceleration-rad-s2",
        type=float,
        help="Override the differential controller acceleration box limit.",
    )
    return parser


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _source_hashes() -> dict[str, str]:
    files = (
        ROOT / "scripts" / "teleop" / "run_r1_quest3_live.py",
        ROOT / "scripts" / "teleop" / "quest_bridge.py",
        ROOT / "teleop" / "r1" / "bridge.py",
        ROOT / "teleop" / "r1" / "isaaclab_sink.py",
        ROOT / "teleop" / "r1" / "mapping.py",
        ROOT / "teleop" / "r1" / "schema.py",
        ROOT / "teleop" / "r1" / "simulator.py",
        ROOT / "teleop" / "r1" / "live_arm_head.py",
        ROOT / "teleop" / "r1" / "rate_limit.py",
        ROOT / "teleop" / "r1" / "upper_body_kinematics.py",
        ROOT / "teleop" / "r1" / "offline_replay.py",
        ROOT / "teleop" / "r1" / "upstream_joint_stream.py",
        ROOT / "teleop" / "r1" / "upper_body_ik.py",
        ROOT / "teleop" / "r1" / "whole_upper_body.py",
        ROOT / "teleop" / "r1" / "differential_tracking.py",
        ROOT / "teleop" / "r1" / "differential_live.py",
        ROOT / "teleop" / "r1" / "workspace_projection.py",
        ROOT / "teleop" / "r1" / "dataset_episode.py",
        ROOT / "teleop" / "r1" / "dataset_randomizer.py",
        ROOT / "teleop" / "r1" / "dataset_scene.py",
        ROOT / "teleop" / "r1" / "operator_cue.py",
    )
    return {str(path.relative_to(ROOT)): _sha256(path) for path in files}


def _head_tracking_error(target_records: list[dict[str, object]]) -> dict[str, object]:
    """Absolute yaw/pitch error between commanded head targets and observed joints.

    A large error with a healthy command path is the signature of the simulator
    refusing the target rather than of a broken bridge, so it is recorded as a
    first-class metric instead of being inferred from the video.
    """

    def dispatched(record: dict[str, object]) -> bool:
        for key in ("whole_upper_body", "arm_head"):
            if key in record:
                return bool(dict(record[key]).get("accepted"))
        return True

    errors = [
        (
            abs(float(record.get("applied_head_target_rad", [record["head_yaw_rad"], record["head_pitch_rad"]])[0]) - float(record["post_physics_head_position_rad"][0])),
            abs(float(record.get("applied_head_target_rad", [record["head_yaw_rad"], record["head_pitch_rad"]])[1]) - float(record["post_physics_head_position_rad"][1])),
        )
        for record in target_records
        if record["enabled"] and dispatched(record) and "post_physics_head_position_rad" in record
    ]
    if not errors:
        return {"count": 0, "max_yaw": None, "max_pitch": None, "mean_yaw": None, "mean_pitch": None}
    return {
        "count": len(errors),
        "max_yaw": max(yaw for yaw, _ in errors),
        "max_pitch": max(pitch for _, pitch in errors),
        "mean_yaw": sum(yaw for yaw, _ in errors) / len(errors),
        "mean_pitch": sum(pitch for _, pitch in errors) / len(errors),
    }


def _numeric_summary(values: list[float]) -> dict[str, object]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if len(finite) == 0:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": int(len(finite)),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.quantile(finite, 0.95)),
        "max": float(np.max(finite)),
    }


def _arm_straightness_deg(model: object, q: np.ndarray, side: str) -> float:
    """Return the shoulder-elbow-endpoint angle (180 deg is a straight arm)."""

    if side not in {"left", "right"}:
        raise ValueError(f"Unsupported arm side: {side}")
    values = np.asarray(q, dtype=float)
    chain = getattr(model, f"{side}_arm")
    arm_slice = getattr(model, f"{side}_arm_slice")
    waist = model.waist_transform_from_q(values)
    arm_q = values[arm_slice]
    shoulder = (waist @ np.append(chain.shoulder_origin(), 1.0))[:3]
    # Joint four is the elbow in the R1-A5 five-joint arm chain.  Its
    # cumulative transform has the same origin before and after its rotation.
    elbow = (waist @ chain.link_transforms(arm_q)[3])[:3, 3]
    endpoint = (waist @ chain.forward_kinematics(arm_q))[:3, 3]
    upper = shoulder - elbow
    lower = endpoint - elbow
    denominator = float(np.linalg.norm(upper) * np.linalg.norm(lower))
    if denominator <= 1.0e-12:
        return float("nan")
    cosine = float(np.clip(np.dot(upper, lower) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _differential_tracking_metrics(
    target_records: list[dict[str, object]], model: object
) -> dict[str, object] | None:
    rows = []
    for record in target_records:
        application = dict(record.get("whole_upper_body", {}))
        if application.get("accepted") and application.get("controller_type") == "differential_dls":
            rows.append((record, application))
    if not rows:
        return None
    times = np.asarray([float(record["elapsed_s"]) for record, _ in rows])
    control_steps = np.asarray(
        [int(record["control_step"]) for record, _ in rows], dtype=int
    )
    # Deadman release and reconnects intentionally create gaps between accepted
    # samples.  Those gaps are not controller periods and would under-report the
    # live update rate by folding operator inactivity into the timing metric.
    consecutive = np.diff(control_steps) == 1
    periods = np.diff(times)[consecutive]
    compute_ms = [float(application["controller_compute_ms"]) for _, application in rows]
    joint_error: list[float] = []
    left_error: list[float] = []
    right_error: list[float] = []
    straightness = {"left": [], "right": []}
    target_reach = {"left": [], "right": []}
    projection_left = 0
    projection_right = 0
    velocity_saturated = 0
    acceleration_saturated = 0
    lead_clamped = 0
    joint_limit_active = 0
    joint_samples = 0
    branch_recovery_states: list[np.ndarray] = []
    for record, application in rows:
        actual_q = np.asarray(record["post_physics_whole_upper_body_position_rad"], dtype=float)
        reference_q = np.asarray(application["joint_position_reference_rad"], dtype=float)
        joint_error.extend(np.abs(reference_q - actual_q).tolist())
        state = model.forward_kinematics(actual_q)
        left_target = np.asarray(application["left_target_position_pelvis_m"], dtype=float)
        right_target = np.asarray(application["right_target_position_pelvis_m"], dtype=float)
        left_error.append(float(np.linalg.norm(left_target - state.left_end_effector[:3, 3])))
        right_error.append(float(np.linalg.norm(right_target - state.right_end_effector[:3, 3])))
        waist = model.waist_transform_from_q(actual_q)
        for side, target in (("left", left_target), ("right", right_target)):
            chain = getattr(model, f"{side}_arm")
            shoulder = (waist @ np.append(chain.shoulder_origin(), 1.0))[:3]
            straightness[side].append(_arm_straightness_deg(model, actual_q, side))
            target_reach[side].append(float(np.linalg.norm(target - shoulder)))
        projection = dict(application["workspace_projection"])
        projection_left += int(bool(projection["left_projected"]))
        projection_right += int(bool(projection["right_projected"]))
        for key, accumulator in (
            ("velocity_saturated", "velocity"),
            ("acceleration_saturated", "acceleration"),
            ("reference_lead_clamped", "lead"),
            ("joint_limit_active", "limit"),
        ):
            count = int(np.count_nonzero(np.asarray(application[key], dtype=bool)))
            if accumulator == "velocity":
                velocity_saturated += count
            elif accumulator == "acceleration":
                acceleration_saturated += count
            elif accumulator == "lead":
                lead_clamped += count
            else:
                joint_limit_active += count
        joint_samples += len(reference_q)
        branch_recovery_states.append(
            np.asarray(
                application.get("branch_recovery_active", [False, False]),
                dtype=bool,
            )
        )
    sample_count = len(rows)
    branch_recovery = np.asarray(branch_recovery_states, dtype=bool)
    recovery_activations = (
        np.count_nonzero(branch_recovery[1:] & ~branch_recovery[:-1], axis=0)
        if sample_count > 1
        else np.zeros(2, dtype=int)
    )
    if sample_count and branch_recovery[0].any():
        recovery_activations = recovery_activations + branch_recovery[0].astype(int)
    extension_metrics: dict[str, object] = {}
    for side in ("left", "right"):
        reach = np.asarray(target_reach[side], dtype=float)
        angles = np.asarray(straightness[side], dtype=float)
        farthest_index = int(np.nanargmax(reach))
        farthest_threshold = float(np.nanquantile(reach, 0.95))
        farthest_angles = angles[reach >= farthest_threshold]
        extension_metrics[side] = {
            "straightness_deg": _numeric_summary(angles.tolist()),
            "target_shoulder_distance_m": _numeric_summary(reach.tolist()),
            "straightness_at_max_target_reach_deg": float(angles[farthest_index]),
            "max_target_shoulder_distance_m": float(reach[farthest_index]),
            "straightness_over_farthest_5pct_deg": _numeric_summary(farthest_angles.tolist()),
            "definition": "shoulder-elbow-endpoint angle; 180 deg is straight",
        }
    return {
        "controller_type": "differential_dls",
        "jacobian_backend": rows[0][1]["jacobian_backend"],
        "accepted_sample_count": sample_count,
        "effective_update_hz": (float(1.0 / np.mean(periods)) if len(periods) else None),
        "update_period_ms": _numeric_summary((1000.0 * periods).tolist()),
        "controller_compute_ms": _numeric_summary(compute_ms),
        "wrist_position_error_m": {
            "left": _numeric_summary(left_error),
            "right": _numeric_summary(right_error),
        },
        "arm_extension": extension_metrics,
        "joint_reference_tracking_error_rad": _numeric_summary(joint_error),
        "workspace_projection_fraction": {
            "left": projection_left / sample_count,
            "right": projection_right / sample_count,
            "either": sum(
                int(bool(dict(application["workspace_projection"])["left_projected"]))
                or int(bool(dict(application["workspace_projection"])["right_projected"]))
                for _, application in rows
            )
            / sample_count,
        },
        "constraint_activation_fraction": {
            "velocity": velocity_saturated / joint_samples,
            "acceleration": acceleration_saturated / joint_samples,
            "reference_lead": lead_clamped / joint_samples,
            "joint_limit": joint_limit_active / joint_samples,
        },
        "branch_recovery": {
            "left_active_fraction": float(np.mean(branch_recovery[:, 0])),
            "right_active_fraction": float(np.mean(branch_recovery[:, 1])),
            "left_activation_count": int(recovery_activations[0]),
            "right_activation_count": int(recovery_activations[1]),
            "independent_per_arm": True,
        },
    }


def _offline_trajectory_tracking_metrics(
    target_records: list[dict[str, object]], model: object
) -> dict[str, object] | None:
    rows = []
    for record in target_records:
        application = dict(record.get("whole_upper_body", {}))
        if (
            application.get("accepted")
            and application.get("controller_type") == "offline_trajectory_continuation"
            and record.get("post_physics_whole_upper_body_position_rad") is not None
        ):
            rows.append((record, application))
    if not rows:
        return None
    joint_error: list[float] = []
    command_to_observed = {"left": [], "right": []}
    source_to_observed = {"left": [], "right": []}
    compute_ms: list[float] = []
    age_ms: list[float] = []
    times: list[float] = []
    steps: list[int] = []
    for record, application in rows:
        reference = np.asarray(application["limited_joint_target_rad"], dtype=float)
        observed = np.asarray(
            record["post_physics_whole_upper_body_position_rad"], dtype=float
        )
        joint_error.extend(np.abs(reference - observed).tolist())
        reference_state = model.forward_kinematics(reference)
        observed_state = model.forward_kinematics(observed)
        for side in ("left", "right"):
            reference_endpoint = getattr(reference_state, f"{side}_end_effector")[:3, 3]
            observed_endpoint = getattr(observed_state, f"{side}_end_effector")[:3, 3]
            source_target = np.asarray(
                application[f"{side}_target_position_pelvis_m"], dtype=float
            )
            command_to_observed[side].append(
                float(np.linalg.norm(reference_endpoint - observed_endpoint))
            )
            source_to_observed[side].append(
                float(np.linalg.norm(source_target - observed_endpoint))
            )
        compute_ms.append(float(application["controller_compute_ms"]))
        age_ms.append(1000.0 * float(record["age_s"]))
        times.append(float(record["elapsed_s"]))
        steps.append(int(record["control_step"]))
    consecutive = np.diff(np.asarray(steps, dtype=int)) == 1
    periods = np.diff(np.asarray(times, dtype=float))[consecutive]
    return {
        "sample_count": len(rows),
        "joint_reference_to_physx_abs_error_rad": _numeric_summary(joint_error),
        "wrist_command_fk_to_physx_fk_error_m": {
            side: _numeric_summary(command_to_observed[side]) for side in ("left", "right")
        },
        "source_wrist_target_to_physx_fk_error_m": {
            side: _numeric_summary(source_to_observed[side]) for side in ("left", "right")
        },
        "trajectory_lookup_compute_ms": _numeric_summary(compute_ms),
        "fresh_or_held_command_age_ms": _numeric_summary(age_ms),
        "consecutive_update_period_s": _numeric_summary(periods.tolist()),
        "effective_update_hz_from_median_period": (
            None if len(periods) == 0 else float(1.0 / np.median(periods))
        ),
        "error_separation": (
            "command_fk_to_physx_fk isolates simulator tracking; source_target_to_physx_fk "
            "also includes offline retargeting error."
        ),
    }


def _stdin_reader(sink: "queue.Queue[str | None]") -> None:
    """Feed stdin lines to the control loop without blocking the simulator."""

    for line in sys.stdin:
        line = line.strip()
        if line:
            sink.put(line)
    sink.put(None)


def _load_replay_payloads(path: Path) -> list[dict[str, object]]:
    """Load and validate a recorded command stream before Isaac starts."""

    payloads: list[dict[str, object]] = []
    previous_timestamp: float | None = None
    previous_sequence: int | None = None
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            payload = json.loads(line)
            timestamp = float(payload["timestamp_monotonic_s"])
            sequence = int(payload["sequence_id"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid replay command at line {line_number}: {exc}") from exc
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError(
                f"replay timestamps must increase strictly (line {line_number})"
            )
        if previous_sequence is not None and sequence <= previous_sequence:
            raise ValueError(
                f"replay sequence_id must increase strictly (line {line_number})"
            )
        payloads.append(payload)
        previous_timestamp = timestamp
        previous_sequence = sequence
    if not payloads:
        raise ValueError("replay command file contains no commands")
    return payloads


def _replay_reader(
    payloads: list[dict[str, object]],
    sink: "queue.Queue[str | None]",
    replay_speed: float,
) -> None:
    """Emit recorded commands in real time with valid local monotonic stamps."""

    source_origin = float(payloads[0]["timestamp_monotonic_s"])
    replay_origin = time.monotonic()
    for source in payloads:
        source_offset = (
            float(source["timestamp_monotonic_s"]) - source_origin
        ) / replay_speed
        due = replay_origin + source_offset
        while True:
            remaining = due - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.002, remaining))
        payload = dict(source)
        payload["timestamp_monotonic_s"] = due
        sink.put(json.dumps(payload, sort_keys=True))
    sink.put(None)


def _install_stop_handlers() -> tuple[threading.Event, dict[str, str], dict[int, object]]:
    """Turn SIGINT/SIGTERM into a loop-level stop so evidence can be finalized."""

    requested = threading.Event()
    detail: dict[str, str] = {"reason": ""}
    previous: dict[int, object] = {}

    def request_stop(signum: int, _frame: object) -> None:
        detail["reason"] = f"signal_{signal.Signals(signum).name}"
        requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return requested, detail, previous


def _restore_stop_handlers(previous: dict[int, object]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def _close_simulation_app_and_exit(
    simulation_app: object,
    *,
    timeout_s: float = 30.0,
    exit_code: int = 0,
    exit_fn: Callable[[int], None] = os._exit,
) -> None:
    """Bound Kit shutdown and end the process even if worker threads survive."""

    closer = threading.Thread(target=simulation_app.close, daemon=True)
    closer.start()
    closer.join(timeout=timeout_s)
    if closer.is_alive():
        print(
            f"Isaac Sim shutdown did not return within {timeout_s:g} s; forcing exit.",
            file=sys.stderr,
            flush=True,
        )
    exit_fn(exit_code)


def main() -> int:
    parser = build_parser()
    if "-h" in sys.argv or "--help" in sys.argv:
        parser.print_help()
        return 0
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    if args.duration_s <= 0.0 or args.control_hz <= 0.0 or args.physics_hz <= 0.0:
        raise SystemExit("--duration-s, --control-hz and --physics-hz must be positive.")
    if not np.isfinite(args.replay_speed) or args.replay_speed <= 0.0:
        raise SystemExit("--replay-speed must be finite and positive.")
    if args.position_scale is not None and (
        not np.isfinite(args.position_scale) or args.position_scale <= 0.0
    ):
        raise SystemExit("--position-scale must be finite and positive.")
    for name in ("max_joint_velocity_rad_s", "max_joint_acceleration_rad_s2"):
        value = getattr(args, name)
        if value is not None and (not np.isfinite(value) or value <= 0.0):
            raise SystemExit(f"--{name.replace('_', '-')} must be finite and positive.")
    if args.physics_hz < args.control_hz:
        raise SystemExit("--physics-hz must be at least --control-hz.")
    if args.stop_file is not None and args.stop_file.expanduser().exists():
        raise SystemExit(f"Refusing to start: --stop-file already exists: {args.stop_file}")
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T001 evidence: {output_dir}")
    replay_path = (
        args.replay_command_file.expanduser().resolve()
        if args.replay_command_file is not None
        else None
    )
    replay_payloads: list[dict[str, object]] | None = None
    if replay_path is not None:
        try:
            replay_payloads = _load_replay_payloads(replay_path)
        except (OSError, ValueError) as exc:
            raise SystemExit(f"Cannot load replay command file {replay_path}: {exc}") from exc

    dataset_scene = None
    if args.dataset_scene_config is not None:
        from teleop.r1.dataset_scene import load_dataset_scene_config  # noqa: E402

        try:
            dataset_scene = load_dataset_scene_config(args.dataset_scene_config)
        except ValueError as exc:
            raise SystemExit(str(exc)) from exc

    try:
        config = json.loads(args.config.expanduser().resolve().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load teleop JSON config {args.config}: {exc}") from exc
    if config.get("mode") != "simulation_only":
        raise SystemExit("R1 teleop v1 only permits mode='simulation_only'.")
    velocity = config.get("velocity") or {}
    if bool(velocity.get("enabled", False)):
        raise SystemExit("T001 requires base velocity disabled; refusing to run with velocity enabled.")

    # Camera đầu robot vẫn là một camera: Isaac từ chối spawn nó nếu rendering
    # chưa bật, kể cả khi --no-video đã tắt video bằng chứng.
    needs_head_camera = dataset_scene is not None and dataset_scene.head_camera is not None
    args.enable_cameras = (not args.no_video) or needs_head_camera
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils  # noqa: E402
    import torch  # noqa: E402
    from isaaclab.assets import Articulation  # noqa: E402
    from isaaclab.sensors import Camera, CameraCfg  # noqa: E402

    from teleop.r1 import (  # noqa: E402
        HeadOnlyIsaacLabSink,
        IsaacLabArticulationHandle,
        R1TeleopCommand,
        R1TeleopMapper,
        SimulationOnlyAdapter,
        TeleopCalibration,
        TeleopLimits,
        Vector3,
    )
    from teleop.r1.live_arm_head import ArmHeadIsaacLabSink, ArmHeadLiveConfig  # noqa: E402
    from teleop.r1.ik import ArmIKConfig  # noqa: E402
    from teleop.r1.mapping import R1A5WholeUpperBodyOwnership  # noqa: E402
    from teleop.r1.upper_body_ik import UpperBodyIKConfig  # noqa: E402
    from teleop.r1.kinematics import KinematicsError  # noqa: E402
    from teleop.r1.upper_body_kinematics import retarget_nominal  # noqa: E402
    from teleop.r1.whole_upper_body import (  # noqa: E402
        WholeUpperBodyIsaacLabSink,
        WholeUpperBodyLiveConfig,
    )
    from teleop.r1.differential_tracking import DifferentialTrackingConfig  # noqa: E402
    from teleop.r1.differential_live import (  # noqa: E402
        DifferentialWholeUpperBodyIsaacLabSink,
        DifferentialWholeUpperBodyLiveConfig,
    )
    from teleop.r1.offline_replay import (  # noqa: E402
        OfflineTrajectoryIsaacLabSink,
        OfflineTrajectoryReplayConfig,
    )
    from teleop.r1.upstream_joint_stream import (  # noqa: E402
        UpstreamJointStreamConfig,
        UpstreamJointStreamSink,
    )
    from training.isaaclab.robot import UNITREE_R1_CFG  # noqa: E402

    calibration_config = config.get("calibration") or {}
    translation = calibration_config.get("translation_m") or [0.0, 0.0, 0.0]
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(calibration_config.get("yaw_rad", 0.0)),
            source_frame=str(config.get("source_frame", "quest_headset")),
            robot_frame=str(config.get("robot_frame", "neutral_waist_yaw_link")),
        ),
        TeleopLimits(command_timeout_s=float(config.get("command_timeout_s", 0.5)), allow_velocity=False),
    )

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / args.physics_hz, device=args.device)
    )
    sim.set_camera_view([2.2, 1.6, 1.5], [0.0, 0.0, 0.9])
    # Môi trường chuẩn mang sẵn sàn của nó; trải thêm một mặt đất phẳng lên trên
    # sẽ che mất sàn đó và làm robot đứng trên một tấm bạt xám giữa căn phòng.
    scene_has_environment = (
        dataset_scene is not None and dataset_scene.environment is not None
    )
    if not scene_has_environment:
        sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
        sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9)).func(
            "/World/Light", sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9))
        )
    scene_props: dict[str, object] = {"spawned": [], "rigid_objects": []}
    if dataset_scene is not None:
        from teleop.r1.dataset_scene import spawn_scene_props  # noqa: E402

        scene_props = spawn_scene_props(dataset_scene)

    robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
    if args.disable_self_collisions:
        robot_cfg.spawn.articulation_props.enabled_self_collisions = False
    upper_body_mode = (
        args.arm_head_config is not None
        or args.whole_upper_body_config is not None
        or args.offline_joint_trajectory is not None
        or args.upstream_joint_stream_config is not None
    )
    if upper_body_mode:
        robot_cfg.spawn.articulation_props.fix_root_link = True
    robot = Articulation(robot_cfg)

    # Mirrored across the robot's y axis so the two views cover the left and
    # right side of the workspace; an arm occluded in one is visible in the other.
    evidence_camera_look_at = (0.0, 0.0, 0.9)
    evidence_camera_positions = [(2.2, 1.6, 1.5)]
    if args.dual_view:
        evidence_camera_positions.append((2.2, -1.6, 1.5))

    cameras: list[object] = []
    if not args.no_video:
        for index, position in enumerate(evidence_camera_positions):
            cameras.append(
                Camera(
                    CameraCfg(
                        prim_path=f"/World/EvidenceCamera_{index}",
                        update_period=0.0,
                        height=args.video_height,
                        width=args.video_width,
                        data_types=["rgb"],
                        spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.1, 1.0e5)),
                        offset=CameraCfg.OffsetCfg(pos=position, rot=(0.0, 0.0, 0.0, 1.0), convention="world"),
                    )
                )
            )

    head_camera = None
    if dataset_scene is not None and dataset_scene.head_camera is not None:
        from teleop.r1.dataset_scene import build_head_camera  # noqa: E402

        head_camera = build_head_camera(dataset_scene.head_camera)

    sim.reset()
    for camera_instance, position in zip(cameras, evidence_camera_positions):
        camera_instance.set_world_poses_from_view(
            torch.tensor([list(position)], device=sim.device),
            torch.tensor([list(evidence_camera_look_at)], device=sim.device),
        )

    handle = IsaacLabArticulationHandle(robot)
    legacy_arm_head_mode = args.arm_head_config is not None
    offline_trajectory_mode = args.offline_joint_trajectory is not None
    upstream_stream_mode = args.upstream_joint_stream_config is not None
    whole_upper_body_mode = (
        args.whole_upper_body_config is not None
        or offline_trajectory_mode
        or upstream_stream_mode
    )
    arm_head_mode = legacy_arm_head_mode or whole_upper_body_mode
    experiment_config_payload: dict[str, object] | None = None
    arm_config = None
    whole_config = None
    if legacy_arm_head_mode:
        try:
            experiment_config_payload = json.loads(args.arm_head_config.expanduser().resolve().read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SystemExit(f"Cannot load arm/head config {args.arm_head_config}: {exc}") from exc
        if experiment_config_payload.get("experiment_id") != "t007" or experiment_config_payload.get("mode") != "simulation_only":
            raise SystemExit("--arm-head-config must be a T007 simulation-only configuration.")
        declared = dict(experiment_config_payload["arm_head"])
        ik_declared = dict(declared["ik"])
        arm_config = ArmHeadLiveConfig(
            left_neutral_position_m=np.asarray(declared["left_neutral_position_m"], dtype=float),
            right_neutral_position_m=np.asarray(declared["right_neutral_position_m"], dtype=float),
            left_lower_m=np.asarray(declared["left_workspace"]["lower_m"], dtype=float),
            left_upper_m=np.asarray(declared["left_workspace"]["upper_m"], dtype=float),
            right_lower_m=np.asarray(declared["right_workspace"]["lower_m"], dtype=float),
            right_upper_m=np.asarray(declared["right_workspace"]["upper_m"], dtype=float),
            position_scale=float(declared["position_scale"]),
            max_joint_velocity_rad_s=float(declared["max_joint_velocity_rad_s"]),
            max_joint_acceleration_rad_s2=float(declared["max_joint_acceleration_rad_s2"]),
            control_dt_s=1.0 / args.control_hz,
            ik=ArmIKConfig(**{key: ik_declared[key] for key in ("position_tolerance_m", "roll_tolerance_rad", "max_iterations", "damping", "posture_weight", "max_joint_step_rad", "posture_tolerance_rad")}),
            enforce_workspace=bool(declared.get("enforce_workspace", True)),
            allow_converged_joint_limit_solution=bool(declared.get("allow_converged_joint_limit_solution", False)),
            mapping_mode=str(declared.get("mapping_mode", "relative_session")),
            allow_projected_position_solution=bool(declared.get("allow_projected_position_solution", False)),
            allow_clamped_roll_solution=bool(declared.get("allow_clamped_roll_solution", False)),
            neutral_joint_position_rad=tuple(float(value) for value in declared["neutral_joint_position_rad"]),
        )
        sink = ArmHeadIsaacLabSink(handle, arm_config)
        ownership = mapper.ownership
    elif offline_trajectory_mode:
        if args.offline_continuation_config is None:
            raise SystemExit(
                "--offline-continuation-config is required with --offline-joint-trajectory."
            )
        try:
            experiment_config_payload = json.loads(
                args.offline_continuation_config.expanduser()
                .resolve()
                .read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            raise SystemExit(
                f"Cannot load offline continuation config {args.offline_continuation_config}: {exc}"
            ) from exc
        if (
            experiment_config_payload.get("experiment_id") != "t007"
            or experiment_config_payload.get("mode") != "simulation_only"
        ):
            raise SystemExit(
                "--offline-continuation-config must be a T007 simulation-only configuration."
            )
        model_declared = dict(experiment_config_payload["model"])
        if str(model_declared.get("body_mode")) != "arms_head":
            raise SystemExit("Offline continuation replay currently requires body_mode='arms_head'.")
        whole_config = OfflineTrajectoryReplayConfig(
            trajectory_path=args.offline_joint_trajectory.expanduser().resolve(),
            urdf_path=(ROOT / str(model_declared["urdf_path"])).resolve(),
            body_mode="arms_head",
            fixed_waist_yaw_rad=float(model_declared.get("fixed_waist_yaw_rad", 0.0)),
            hold_uncontrolled_waist_joints=bool(
                model_declared.get("hold_uncontrolled_waist_joints", False)
            ),
            waist_roll_hold_rad=float(model_declared.get("waist_roll_hold_rad", 0.0)),
            held_joint_stiffness=float(model_declared.get("held_joint_stiffness", 10000.0)),
            held_joint_damping=float(model_declared.get("held_joint_damping", 200.0)),
        )
        sink = OfflineTrajectoryIsaacLabSink(handle, whole_config)
        ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head")
    elif upstream_stream_mode:
        try:
            experiment_config_payload = json.loads(
                args.upstream_joint_stream_config.expanduser()
                .resolve()
                .read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            raise SystemExit(
                f"Cannot load upstream joint stream config "
                f"{args.upstream_joint_stream_config}: {exc}"
            ) from exc
        if (
            experiment_config_payload.get("experiment_id") != "t007"
            or experiment_config_payload.get("mode") != "simulation_only"
        ):
            raise SystemExit(
                "--upstream-joint-stream-config must be a T007 simulation-only configuration."
            )
        model_declared = dict(experiment_config_payload["model"])
        if str(model_declared.get("body_mode")) != "arms_head":
            raise SystemExit(
                "The vendor solver locks waist yaw and both head joints, so the upstream "
                "stream path requires body_mode='arms_head'."
            )
        whole_config = UpstreamJointStreamConfig(
            urdf_path=(ROOT / str(model_declared["urdf_path"])).resolve(),
            body_mode="arms_head",
            fixed_waist_yaw_rad=float(model_declared.get("fixed_waist_yaw_rad", 0.0)),
            hold_uncontrolled_waist_joints=bool(
                model_declared.get("hold_uncontrolled_waist_joints", True)
            ),
            waist_roll_hold_rad=float(model_declared.get("waist_roll_hold_rad", 0.0)),
            held_joint_stiffness=float(model_declared.get("held_joint_stiffness", 10000.0)),
            held_joint_damping=float(model_declared.get("held_joint_damping", 200.0)),
        )
        sink = UpstreamJointStreamSink(handle, whole_config)
        ownership = R1A5WholeUpperBodyOwnership(body_mode="arms_head")
    elif whole_upper_body_mode:
        try:
            experiment_config_payload = json.loads(
                args.whole_upper_body_config.expanduser().resolve().read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            raise SystemExit(
                f"Cannot load whole-upper-body config {args.whole_upper_body_config}: {exc}"
            ) from exc
        if experiment_config_payload.get("experiment_id") != "t007" or experiment_config_payload.get("mode") != "simulation_only":
            raise SystemExit("--whole-upper-body-config must be a T007 simulation-only configuration.")
        declared = dict(experiment_config_payload["whole_upper_body"])
        urdf_path = (ROOT / str(declared["urdf_path"])).resolve()
        profile_body_mode = str(declared.get("body_mode", "waist_yaw"))
        body_mode = str(args.body_mode or profile_body_mode)
        nominal = tuple(float(value) for value in declared["nominal_joint_position_rad"])
        fixed_waist_yaw_rad = 0.0
        if body_mode != profile_body_mode:
            # --body-mode lets an operator freeze or free the torso without
            # editing the profile, so the declared nominal is converted to the
            # selected mode's length rather than rejected. The torso pose the
            # profile declared is preserved across the conversion. The effective
            # mode is recorded in the resolved config either way.
            try:
                nominal, fixed_waist_yaw_rad = retarget_nominal(
                    nominal, profile_body_mode, body_mode
                )
            except KinematicsError as exc:
                raise SystemExit(f"Cannot apply --body-mode {body_mode}: {exc}") from exc
        controller_declared = dict(declared.get("controller") or {})
        controller_type = str(controller_declared.get("type", "iterative_pose_ik"))
        if controller_type == "differential_dls":
            projection_declared = dict(declared.get("workspace_projection") or {})
            calibration_declared = dict(declared.get("calibration") or {})
            tracker_fields = {
                key: float(controller_declared[key])
                for key in (
                    "position_gain_s",
                    "wrist_orientation_gain_s",
                    "head_orientation_gain_s",
                    "damping",
                    "finite_difference_rad",
                    "position_weight",
                    "wrist_orientation_weight",
                    "head_orientation_weight",
                    "max_joint_velocity_rad_s",
                    "max_joint_acceleration_rad_s2",
                    "max_reference_lead_rad",
                    "posture_gain_s",
                    "wide_elbow_pole_gain_s",
                    "wide_elbow_pole_weight",
                    "wide_hand_separation_start_m",
                    "wide_hand_separation_full_m",
                    "branch_recovery_position_error_m",
                    "branch_recovery_exit_error_m",
                    "branch_recovery_limit_margin_rad",
                    "branch_recovery_gain_s",
                    "branch_recovery_weight",
                )
            }
            if args.max_joint_velocity_rad_s is not None:
                tracker_fields["max_joint_velocity_rad_s"] = float(
                    args.max_joint_velocity_rad_s
                )
            if args.max_joint_acceleration_rad_s2 is not None:
                tracker_fields["max_joint_acceleration_rad_s2"] = float(
                    args.max_joint_acceleration_rad_s2
                )
            whole_config = DifferentialWholeUpperBodyLiveConfig(
                urdf_path=urdf_path,
                nominal_joint_position_rad=nominal,
                tracker=DifferentialTrackingConfig(
                    dt_s=1.0 / args.control_hz,
                    task_priority=str(
                        controller_declared.get("task_priority", "weighted")
                    ),
                    **tracker_fields,
                ),
                source_target_frame=str(declared["source_target_frame"]),
                body_mode=body_mode,
                fixed_waist_yaw_rad=fixed_waist_yaw_rad,
                workspace_projection_margin_m=float(projection_declared["margin_m"]),
                mapping_mode=str(calibration_declared.get("mapping_mode", "relative_session")),
                position_scale=float(
                    args.position_scale
                    if args.position_scale is not None
                    else calibration_declared.get("position_scale", 0.85)
                ),
                velocity_feedforward_task=str(
                    args.velocity_feedforward_task
                    if args.velocity_feedforward_task is not None
                    else controller_declared.get("velocity_feedforward_task", "none")
                ),
                source_rate_hz=float(controller_declared.get("source_rate_hz", 30.0)),
                velocity_filter_alpha=float(
                    controller_declared.get("velocity_filter_alpha", 0.35)
                ),
            )
            sink = DifferentialWholeUpperBodyIsaacLabSink(handle, whole_config)
        elif controller_type == "iterative_pose_ik":
            ik_declared = dict(declared["ik"])
            whole_config = WholeUpperBodyLiveConfig(
                urdf_path=urdf_path,
                nominal_joint_position_rad=nominal,
                max_joint_velocity_rad_s=float(declared["max_joint_velocity_rad_s"]),
                max_joint_acceleration_rad_s2=float(declared["max_joint_acceleration_rad_s2"]),
                control_dt_s=1.0 / args.control_hz,
                ik=UpperBodyIKConfig(**ik_declared),
                source_target_frame=str(declared["source_target_frame"]),
                allow_nonconverged_solution=bool(declared.get("allow_nonconverged_solution", False)),
                body_mode=body_mode,
                fixed_waist_yaw_rad=fixed_waist_yaw_rad,
                seed_restart_residual_m=(
                    float(declared["seed_restart_residual_m"])
                    if declared.get("seed_restart_residual_m") is not None
                    else None
                ),
                hold_uncontrolled_waist_joints=bool(
                    declared.get("hold_uncontrolled_waist_joints", False)
                ),
                waist_roll_hold_rad=float(declared.get("waist_roll_hold_rad", 0.0)),
                held_joint_stiffness=float(declared.get("held_joint_stiffness", 10000.0)),
                held_joint_damping=float(declared.get("held_joint_damping", 200.0)),
                allow_projected_position_solution=bool(
                    declared.get("allow_projected_position_solution", False)
                ),
            )
            sink = WholeUpperBodyIsaacLabSink(handle, whole_config)
        else:
            raise SystemExit(f"Unsupported whole-upper-body controller type: {controller_type}")
        ownership = R1A5WholeUpperBodyOwnership(body_mode=whole_config.body_mode)
    else:
        sink = HeadOnlyIsaacLabSink(handle)
        ownership = mapper.ownership
    adapter = SimulationOnlyAdapter(sink, ownership)

    default_joint_pos = robot.data.default_joint_pos.clone()
    default_joint_vel = robot.data.default_joint_vel.clone()
    startup_joint_pos = default_joint_pos.clone()
    if legacy_arm_head_mode:
        assert arm_config is not None
        neutral_q = torch.tensor(
            arm_config.neutral_joint_position_rad,
            device=robot.device,
            dtype=startup_joint_pos.dtype,
        )
        startup_joint_pos[:, handle.left_arm_joint_ids] = neutral_q
        startup_joint_pos[:, handle.right_arm_joint_ids] = neutral_q
        startup_joint_pos[:, handle.joint_ids] = 0.0
    elif whole_upper_body_mode:
        assert whole_config is not None
        ids = [robot.data.joint_names.index(name) for name in sink.model.joint_names]
        startup_nominal = (
            sink.nominal_joint_position_rad
            if offline_trajectory_mode or upstream_stream_mode
            else whole_config.nominal_joint_position_rad
        )
        startup_joint_pos[:, ids] = torch.tensor(
            startup_nominal,
            device=robot.device,
            dtype=startup_joint_pos.dtype,
        )
    robot.write_joint_state_to_sim(startup_joint_pos, default_joint_vel)
    robot.set_joint_position_target(startup_joint_pos)
    held_waist_stiffness = None
    if arm_head_mode and getattr(whole_config, "hold_uncontrolled_waist_joints", False):
        # Commanding a zero target is not enough: the asset's waist actuator is
        # 100 N m/rad, and the reaction torque of the swinging arms bends it by
        # up to 22.4 deg anyway. The startup target is already zero, so writing
        # zero every step changes nothing measurable. The offline model treats
        # the waist as rigid, so the simulation has to be stiff enough to match
        # the model it is compared against.
        held_ids = [robot.data.joint_names.index(name) for name in sink.held_joint_names]
        if held_ids:
            held_waist_stiffness = float(whole_config.held_joint_stiffness)
            robot.write_joint_stiffness_to_sim(
                held_waist_stiffness, joint_ids=held_ids
            )
            robot.write_joint_damping_to_sim(
                float(whole_config.held_joint_damping), joint_ids=held_ids
            )
            print(
                f"[hold] waist joints {sink.held_joint_names} held at "
                f"stiffness={held_waist_stiffness} damping={whole_config.held_joint_damping}",
                flush=True,
            )
    if arm_head_mode:
        # Keep the sink's continuous seeds and rate limiters consistent with
        # the state written above.  Starting from Isaac's curled default pose
        # creates a large, unrelated transient before the first Quest target.
        sink.reset_session()
    pinned_root_state = robot.data.default_root_state.clone()

    writer = None
    if cameras:
        import imageio.v2 as imageio

        writer = imageio.get_writer(
            str(Path(str(output_dir) + ".video.tmp.mp4")), fps=args.video_fps, macro_block_size=None
        )

    if args.viewport_camera == "head" and not getattr(args, "headless", False):
        if head_camera is None:
            raise SystemExit(
                "--viewport-camera head cần một camera đầu; hãy truyền --dataset-scene-config "
                "có khai head_camera."
            )
        # Gán camera đầu vào viewport đang hoạt động. Bọc lại vì đây là API của
        # Kit UI: chạy ẩn hoặc bản Kit khác thì không có viewport nào để gán, và
        # đó không phải lý do làm hỏng cả phiên.
        try:
            from omni.kit.viewport.utility import get_active_viewport  # noqa: E402

            viewport = get_active_viewport()
            if viewport is None:
                print("[viewport] không tìm thấy viewport nào; giữ camera perspective.", flush=True)
            else:
                viewport.camera_path = dataset_scene.head_camera.prim_path
                print(
                    f"[viewport] cửa sổ Isaac dùng {dataset_scene.head_camera.prim_path}",
                    flush=True,
                )
        except Exception as exc:  # noqa: BLE001 - đổi góc nhìn là tiện ích, không phải bằng chứng
            print(f"[viewport] không gán được camera đầu: {exc}", flush=True)

    marker_randomizer = None
    marker_prim_paths: dict[int, str] = {}
    if dataset_scene is not None and dataset_scene.marker_pool is not None:
        from teleop.r1.dataset_randomizer import MarkerRandomizer  # noqa: E402

        pool = dataset_scene.marker_pool
        marker_randomizer = MarkerRandomizer(
            digits=list(pool.digits),
            slot_x_m=pool.slot_x_m,
            slot_y_m=list(pool.slot_y_m),
            slot_z_m=pool.slot_z_m,
            slot_y_jitter_m=pool.slot_y_jitter_m,
            prompt_templates=list(pool.prompt_templates),
            seed=pool.seed,
            priority_digits=[
                int(v) for v in (args.target_digit_priority or "").split(",") if v.strip()
            ],
            fixed_layout_digits=[
                int(v) for v in (args.fixed_digit_layout or "").split(",") if v.strip()
            ],
            prompt_override=args.policy_prompt,
        )
        marker_prim_paths = {d: pool.prim_path(d) for d in pool.digits}

    head_view_publisher = None
    if args.head_view_port is not None and head_camera is not None:
        from teleop.r1.head_view_stream import HeadViewPublisher  # noqa: E402

        head_view_publisher = HeadViewPublisher(port=args.head_view_port)
        print(f"[head-view] phát khung tới tcp://127.0.0.1:{args.head_view_port}", flush=True)
    elif args.head_view_port is not None:
        raise SystemExit(
            "--head-view-port cần một camera đầu; hãy truyền --dataset-scene-config có khai head_camera."
        )

    policy_obs_publisher = None
    if args.policy_obs_port is not None and head_camera is not None:
        from teleop.r1.policy_obs_stream import PolicyObsPublisher  # noqa: E402

        policy_obs_publisher = PolicyObsPublisher(port=args.policy_obs_port)
        print(f"[policy-obs] phát quan sát tới tcp://127.0.0.1:{args.policy_obs_port}", flush=True)
    elif args.policy_obs_port is not None:
        raise SystemExit(
            "--policy-obs-port cần một camera đầu; hãy truyền --dataset-scene-config có khai head_camera."
        )

    head_recorder = None
    episode_recorder = None
    if head_camera is not None:
        # Simulator từ chối một thư mục bằng chứng đã tồn tại, nên ảnh được ghi
        # tạm cạnh nó rồi mới chuyển vào — đúng cách video đang làm.
        staging = Path(str(output_dir) + ".colors.tmp")
        if dataset_scene.episode is not None and whole_upper_body_mode:
            from teleop.r1.dataset_episode import EpisodeRecorder  # noqa: E402

            episode_recorder = EpisodeRecorder(
                staging_dir=staging,
                camera=head_camera,
                physics_dt_s=1.0 / args.physics_hz,
                config=dataset_scene.episode,
                controlled_joint_names=list(sink.model.joint_names),
            )
        else:
            from teleop.r1.dataset_scene import HeadCameraRecorder  # noqa: E402

            head_recorder = HeadCameraRecorder(staging, head_camera, 1.0 / args.physics_hz)

    operator_cue = None
    current_marker_layout = None
    cue_config = dataset_scene.operator_cue if dataset_scene is not None else None
    if cue_config is not None and cue_config.enabled:
        from teleop.r1.operator_cue import OperatorCue  # noqa: E402

        operator_cue = OperatorCue(
            cue_config.text_template,
            desktop_hud=cue_config.desktop_hud and not getattr(args, "headless", False),
        )

    def prepare_next_marker_episode() -> None:
        """Chọn/hiện mục tiêu trước khi người vận hành bóp cò phải."""

        nonlocal current_marker_layout
        if marker_randomizer is None or episode_recorder is None:
            return
        if not episode_recorder.is_between_episodes and not episode_recorder.has_items:
            # Bố cục đã dàn sẵn và chưa ai chạm tới. Nhả cò phải rồi bóp cò trái
            # là HAI sự kiện nhưng chỉ mở MỘT episode, nên chỉ được bốc số một
            # lần: bốc lại làm người vận hành vừa đọc xong mục tiêu thì nó đổi.
            return
        from teleop.r1.dataset_randomizer import apply_layout  # noqa: E402

        current_marker_layout = marker_randomizer.next_layout()
        apply_layout(current_marker_layout, marker_prim_paths, pool.parked_position_m)
        episode_recorder.begin_episode(current_marker_layout.as_task_record())
        if operator_cue is not None:
            operator_cue.show(current_marker_layout)
        print(f"[policy-prompt] {current_marker_layout.prompt}", flush=True)

    # Trước đây mục tiêu chỉ được chọn sau frame accepted đầu tiên, tức người
    # vận hành phải bắt đầu chuyển động khi chưa biết cần chạm số nào.
    prepare_next_marker_episode()

    commands: "queue.Queue[str | None]" = queue.Queue()
    if replay_payloads is None:
        threading.Thread(target=_stdin_reader, args=(commands,), daemon=True).start()
    else:
        threading.Thread(
            target=_replay_reader,
            args=(replay_payloads, commands, args.replay_speed),
            daemon=True,
        ).start()

    raw_lines: list[str] = []
    target_records: list[dict[str, object]] = []
    loop_records: list[dict[str, object]] = []
    dynamics_time_s: list[float] = []
    dynamics_root_position_m: list[list[float]] = []
    dynamics_root_linear_velocity_mps: list[list[float]] = []
    dynamics_joint_position_rad: list[list[float]] = []
    dynamics_joint_velocity_radps: list[list[float]] = []
    dynamics_body_com_position_m: list[list[list[float]]] = []
    invalid_lines: list[dict[str, object]] = []
    head_render_time_s: list[float] = []
    head_sensor_update_time_s: list[float] = []
    head_gpu_readback_time_s: list[float] = []
    head_capture_wall_time_s: list[float] = []
    head_source_frame_ids: list[int] = []
    dataset_fps_gate_failure: dict[str, object] | None = None
    previous_sequence = -1
    last_command: R1TeleopCommand | None = None
    last_command_wall_s: float | None = None
    stream_started = False
    stream_closed = False
    steps_per_control = max(1, int(round(args.physics_hz / args.control_hz)))
    max_catchup_steps = steps_per_control * 4
    sim_time_s = 0.0
    physics_step_count = 0
    # Rendering is intentionally decoupled from control. The video timestamps
    # remain wall-clock scheduled, while an expensive viewport refresh cannot
    # silently turn a requested 30 Hz controller into a 20 Hz controller.
    render_each_control_step = bool(args.render_every_control_step) and not bool(
        getattr(args, "headless", False)
    )
    video_period_s = 1.0 / args.video_fps
    head_period_s = (
        1.0 / episode_recorder.config.fps
        if episode_recorder is not None
        else video_period_s
    )
    next_video_time = 0.0
    next_head_time = 0.0
    scene_rigid_objects = list(scene_props.get("rigid_objects") or [])
    stop_reason = "duration_elapsed"
    stop_requested, stop_detail, previous_handlers = _install_stop_handlers()

    start_monotonic = time.monotonic()
    start_utc = datetime.now(timezone.utc).isoformat()
    control_step = 0
    next_control_elapsed_s = 0.0
    try:
        while simulation_app.is_running():
            if stop_requested.is_set():
                stop_reason = stop_detail["reason"]
                break
            if args.stop_file is not None and args.stop_file.expanduser().exists():
                stop_reason = "stop_file_requested"
                break
            now = time.monotonic()
            elapsed = now - start_monotonic
            if elapsed >= args.duration_s:
                break
            if elapsed < next_control_elapsed_s:
                time.sleep(min(0.002, next_control_elapsed_s - elapsed))
                continue
            next_control_elapsed_s += 1.0 / args.control_hz

            newest: R1TeleopCommand | None = None
            while True:
                try:
                    line = commands.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    stream_closed = True
                    break
                raw_lines.append(line)
                try:
                    payload = json.loads(line)
                    candidate = R1TeleopCommand.from_dict(payload)
                except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                    invalid_lines.append({"line_number": len(raw_lines), "error": str(exc)})
                    continue
                if candidate.sequence_id <= previous_sequence:
                    invalid_lines.append(
                        {"line_number": len(raw_lines), "error": "sequence_id did not increase strictly"}
                    )
                    continue
                previous_sequence = candidate.sequence_id
                if upstream_stream_mode:
                    # Every line is handed to the sink, not only the one this
                    # cycle applies: the stream runs at headset rate and the
                    # loop at control rate, so the sample eventually applied may
                    # have arrived in an earlier batch.
                    solved = payload.get("upstream_joint_position_rad")
                    names = payload.get("upstream_joint_names")
                    if solved is None or names is None:
                        invalid_lines.append(
                            {
                                "line_number": len(raw_lines),
                                "error": (
                                    "upstream stream mode requires upstream_joint_position_rad "
                                    "and upstream_joint_names on every line"
                                ),
                            }
                        )
                    else:
                        sink.ingest(candidate.sequence_id, names, solved)
                if arm_head_mode and candidate.reset_requested:
                    # Cò trái vốn đã có nghĩa "reset session"; ở đây nó đồng thời
                    # là ranh giới episode. Không thêm nút điều khiển nào mới.
                    if episode_recorder is not None and episode_recorder.has_items:
                        episode_recorder.end_episode(wait=False)
                    sink.reset_session()
                    # A reset is a session boundary. Do not let the previously
                    # held right-trigger command be re-applied later in this
                    # same control cycle or after a temporary input gap.
                    last_command = None
                    last_command_wall_s = None
                    stream_started = True
                    prepare_next_marker_episode()
                    continue
                newest = candidate
                stream_started = True

            if newest is not None:
                last_command = newest
                last_command_wall_s = time.monotonic()

            if last_command is None:
                sink_event_index = len(sink.events)
                adapter_applied = None
            else:
                received_monotonic_s = time.monotonic()
                target = mapper.map(last_command, received_monotonic_s)
                sink_event_index = len(sink.events)
                adapter.apply(target)
                record = asdict(target)
                record["control_step"] = control_step
                record["elapsed_s"] = elapsed
                record["command_timestamp_monotonic_s"] = last_command.timestamp_monotonic_s
                record["received_monotonic_s"] = received_monotonic_s
                record["age_s"] = received_monotonic_s - last_command.timestamp_monotonic_s
                record["is_fresh_command"] = newest is not None
                target_records.append(record)
                if arm_head_mode:
                    application_key = "whole_upper_body" if whole_upper_body_mode else "arm_head"
                    record[application_key] = dict(sink.last_application or {})
                    if record[application_key].get("accepted"):
                        record["applied_head_target_rad"] = list(record[application_key]["head_target_rad"])
                adapter_applied = record

            loop_record = {
                "control_step": control_step,
                "elapsed_s": elapsed,
                "had_command": adapter_applied is not None,
                "fresh_command": newest is not None,
                "sink_events": [event["event"] for event in sink.events[sink_event_index:]],
                "pre_physics_head_position_rad": list(handle.head_joint_positions()),
            }
            loop_records.append(loop_record)

            if not arm_head_mode:
                robot.write_root_state_to_sim(pinned_root_state)
            robot.write_data_to_sim()
            # Physics advances to catch up with wall-clock time rather than by a
            # fixed count, so a slow render or a busy GPU makes the loop coarser
            # instead of putting the operator in slow motion. The catch-up is
            # capped so a long stall cannot trigger an unbounded step burst.
            physics_dt = 1.0 / args.physics_hz
            behind_s = max(0.0, elapsed - sim_time_s)
            steps_this_cycle = min(max_catchup_steps, max(1, int(behind_s / physics_dt)))
            for _ in range(steps_this_cycle):
                for scene_object in scene_rigid_objects:
                    scene_object.write_data_to_sim()
                sim.step(render=False)
            sim_time_s += steps_this_cycle * physics_dt
            physics_step_count += steps_this_cycle
            robot.update(physics_dt)
            # Vật trong cảnh phải được làm mới rõ ràng; thiếu bước này thì
            # `data.root_pos_w` trả về đúng giá trị khởi tạo và một vật đã rơi
            # vẫn trông như đang đứng yên.
            for scene_object in scene_rigid_objects:
                scene_object.update(physics_dt)
            if arm_head_mode:
                dynamics_time_s.append(elapsed)
                dynamics_root_position_m.append(robot.data.root_pos_w[0].detach().cpu().tolist())
                dynamics_root_linear_velocity_mps.append(robot.data.root_lin_vel_w[0].detach().cpu().tolist())
                dynamics_joint_position_rad.append(robot.data.joint_pos[0].detach().cpu().tolist())
                dynamics_joint_velocity_radps.append(robot.data.joint_vel[0].detach().cpu().tolist())
                dynamics_body_com_position_m.append(robot.data.body_com_pos_w[0].detach().cpu().tolist())
            post_physics_head_position = list(handle.head_joint_positions())
            loop_record["post_physics_head_position_rad"] = post_physics_head_position
            loop_record["simulated_time_s"] = sim_time_s
            if adapter_applied is not None:
                adapter_applied["post_physics_head_position_rad"] = post_physics_head_position
                if adapter_applied["enabled"]:
                    adapter_applied["post_physics_tracking_error_rad"] = [
                        abs(float(adapter_applied["head_yaw_rad"]) - post_physics_head_position[0]),
                        abs(float(adapter_applied["head_pitch_rad"]) - post_physics_head_position[1]),
                    ]
                if arm_head_mode:
                    post_left, post_right = handle.arm_joint_positions()
                    adapter_applied["post_physics_arm_position_rad"] = {"left": list(post_left), "right": list(post_right)}
                if whole_upper_body_mode:
                    adapter_applied["post_physics_whole_upper_body_position_rad"] = list(
                        handle.joint_positions(sink.model.joint_names)
                    )
            capture_frame = bool(cameras) and elapsed >= next_video_time
            application = (adapter_applied or {}).get("whole_upper_body") or {}
            episode_step_accepted = bool(application.get("accepted"))
            if episode_recorder is not None:
                if episode_step_accepted:
                    if marker_randomizer is not None and episode_recorder.is_between_episodes:
                        prepare_next_marker_episode()
                elif episode_recorder.has_items:
                    # Đóng bằng một job nằm sau toàn bộ ảnh của episode trong
                    # cùng queue. Main thread đổi cue ngay, không chờ ổ đĩa.
                    episode_recorder.end_episode(wait=False)
                    prepare_next_marker_episode()

            # Camera/render có nhịp riêng với control. Trước đây episode_recorder
            # làm biểu thức này luôn True, khiến RTX + JPEG chạy ở mọi control
            # tick và kéo GUI xuống ~5 Hz. Một deadline bị lỡ được bỏ qua thay vì
            # "catch up" bằng nhiều render liên tiếp.
            head_due = head_camera is not None and elapsed >= next_head_time
            record_episode_frame = bool(
                episode_recorder is not None and episode_step_accepted and head_due
            )
            capture_plain_head = bool(head_recorder is not None and head_due)
            publish_head = bool(head_view_publisher is not None and head_due)
            publish_policy_obs = bool(policy_obs_publisher is not None and head_due)
            refresh_head_view = bool(
                head_due
                and (
                    episode_recorder is not None
                    or head_recorder is not None
                    or publish_head
                    or publish_policy_obs
                )
            )
            if head_due:
                missed_periods = max(1, int((elapsed - next_head_time) / head_period_s) + 1)
                next_head_time += missed_periods * head_period_s

            if render_each_control_step or capture_frame or refresh_head_view:
                render_started = time.monotonic()
                sim.render()
                if refresh_head_view:
                    head_render_time_s.append(time.monotonic() - render_started)

            raw_head_rgb = None
            source_camera_frame = None
            need_head_pixels = bool(
                publish_head
                or publish_policy_obs
                or (
                    record_episode_frame
                    and episode_recorder is not None
                    and episode_recorder.can_accept_frame
                )
            )
            if need_head_pixels:
                # Một GPU→CPU copy dùng chung cho dataset và bản operator. Cue
                # chỉ được vẽ lên bản sao ở nhánh publish bên dưới.
                # Camera chỉ được gọi ở deadline ảnh, không phải mỗi physics
                # tick. Truyền physics_dt ở đây làm đồng hồ nội bộ của sensor
                # tiến 0.005 s cho mỗi lần capture và có thể trả lại cùng buffer
                # nhiều lần. force_recompute tạo đúng một source frame mới cho
                # mỗi capture, tương đương latest-frame producer của teleimager.
                update_started = time.monotonic()
                head_camera.update(head_period_s, force_recompute=True)
                head_sensor_update_time_s.append(time.monotonic() - update_started)
                readback_started = time.monotonic()
                raw_head_rgb = head_camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
                source_camera_frame = int(head_camera.frame[0].item())
                head_gpu_readback_time_s.append(time.monotonic() - readback_started)
                head_capture_wall_time_s.append(time.monotonic() - start_monotonic)
                head_source_frame_ids.append(source_camera_frame)

            if publish_policy_obs and raw_head_rgb is not None:
                # State và ảnh phải đến từ CÙNG một bước, và state phải là
                # `post_physics_whole_upper_body_position_rad` — đúng trường mà
                # converter đã ghi vào `observation.state`. Lấy nguồn khác là
                # đưa cho policy một quan sát nó chưa từng thấy lúc train.
                policy_state = (adapter_applied or {}).get(
                    "post_physics_whole_upper_body_position_rad"
                )
                if policy_state is None and whole_upper_body_mode:
                    # Tick ĐẦU TIÊN chưa có lệnh nào được áp, nên `adapter_applied`
                    # còn rỗng. Nếu chờ nó thì hai bên khoá chết nhau: policy chờ
                    # quan sát để sinh lệnh, còn quan sát lại chờ một lệnh đã áp.
                    # Đọc thẳng tư thế hiện tại của robot cắt vòng đó, và đây đúng
                    # là nguồn mà `post_physics_whole_upper_body_position_rad` dùng.
                    policy_state = list(handle.joint_positions(sink.model.joint_names))
                if policy_state is not None:
                    policy_obs_publisher.publish(
                        raw_head_rgb,
                        policy_state,
                        current_marker_layout.prompt if current_marker_layout is not None else "",
                    )

            if publish_head:
                operator_rgb = raw_head_rgb
                assert operator_rgb is not None
                if (
                    operator_cue is not None
                    and cue_config is not None
                    and cue_config.head_view_overlay
                ):
                    operator_rgb = operator_cue.decorate_head_view(operator_rgb)
                head_view_publisher.publish(operator_rgb)

            if record_episode_frame and episode_recorder is not None:
                # Khi queue đầy, add_item(None) chỉ ghi nhận overflow và trả về;
                # nó không gọi camera.update hay tạo một JSON item thiếu ảnh.
                queued = episode_recorder.add_item(
                    control_step,
                    elapsed,
                    (adapter_applied or {}).get("post_physics_whole_upper_body_position_rad"),
                    application.get("limited_joint_target_rad"),
                    application.get("solver_solution_kind"),
                    rgb=raw_head_rgb,
                    source_camera_frame=source_camera_frame,
                )
                if queued and args.strict_dataset_fps:
                    gate = episode_recorder.config.rejection
                    enough_samples = episode_recorder.current_item_count >= gate.fps_gate_warmup_frames
                    measured = episode_recorder.current_measured_fps
                    below = gate.min_measured_fps is not None and measured < gate.min_measured_fps
                    above = gate.max_measured_fps is not None and measured > gate.max_measured_fps
                    duplicates = episode_recorder.current_duplicate_camera_frame_count
                    if enough_samples and (below or above or duplicates):
                        dataset_fps_gate_failure = {
                            "measured_fps": measured,
                            "min_measured_fps": gate.min_measured_fps,
                            "max_measured_fps": gate.max_measured_fps,
                            "warmup_frames": gate.fps_gate_warmup_frames,
                            "duplicate_camera_frame_count": duplicates,
                        }
                        stop_reason = "dataset_camera_fps_gate_failed"
                        print(
                            "[dataset-fps][FAIL] "
                            f"measured={measured:.3f} FPS, expected="
                            f"[{gate.min_measured_fps}, {gate.max_measured_fps}], "
                            f"duplicates={duplicates}; stopping before collecting more invalid data.",
                            file=sys.stderr,
                            flush=True,
                        )
                        break
            elif capture_plain_head:
                head_recorder.capture(control_step, elapsed)

            # Rendering is the dominant cost per control step, so the cameras
            # are only updated on the steps that actually contribute a frame.
            if capture_frame:
                views = []
                for camera_instance in cameras:
                    camera_instance.update(1.0 / args.physics_hz)
                    views.append(
                        camera_instance.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
                    )
                # Both views come from the same rendered step, so one composite
                # frame keeps them synchronized in the evidence by construction.
                frame = views[0] if len(views) == 1 else np.concatenate(views, axis=1)
                writer.append_data(frame.astype("uint8"))
                next_video_time += video_period_s

            control_step += 1
            if stream_closed and commands.empty():
                stop_reason = "command_stream_closed"
                break
            if (
                args.idle_stop_s > 0.0
                and stream_started
                and last_command_wall_s is not None
                and time.monotonic() - last_command_wall_s > args.idle_stop_s
            ):
                stop_reason = "idle_timeout"
                break
    finally:
        if writer is not None:
            writer.close()
        _restore_stop_handlers(previous_handlers)

    # Evidence is written before the simulator is closed on purpose. Isaac Sim's
    # `SimulationApp.close()` has been observed on this workstation not to return,
    # and anything written after it would be lost with the whole run.
    output_dir.mkdir(parents=True)
    video_path = Path(str(output_dir) + ".video.tmp.mp4")
    video_recorded = False
    if video_path.is_file() and video_path.stat().st_size > 0:
        video_path.rename(output_dir / "simulator_view.mp4")
        video_recorded = True
    elif video_path.exists():
        video_path.unlink()

    head_camera_evidence: dict[str, object] = {}
    capture_span_s = (
        head_capture_wall_time_s[-1] - head_capture_wall_time_s[0]
        if len(head_capture_wall_time_s) >= 2
        else 0.0
    )
    head_camera_evidence.update(
        {
            "camera_pipeline": "fresh_latest_frame_bounded_async_writer",
            "camera_render_time_s": _numeric_summary(head_render_time_s),
            "camera_sensor_update_time_s": _numeric_summary(head_sensor_update_time_s),
            "camera_gpu_readback_time_s": _numeric_summary(head_gpu_readback_time_s),
            "camera_capture_count": len(head_capture_wall_time_s),
            "camera_capture_fps_wall_clock": (
                (len(head_capture_wall_time_s) - 1) / capture_span_s
                if capture_span_s > 0.0
                else None
            ),
            "camera_source_frame_count": len(set(head_source_frame_ids)),
            "camera_duplicate_source_frame_count": (
                len(head_source_frame_ids) - len(set(head_source_frame_ids))
            ),
            "dataset_fps_gate_strict": bool(args.strict_dataset_fps),
            "dataset_fps_gate_failure": dataset_fps_gate_failure,
        }
    )
    if head_view_publisher is not None:
        head_camera_evidence.update(head_view_publisher.stats())
        head_view_publisher.close()
    if policy_obs_publisher is not None:
        head_camera_evidence.update(policy_obs_publisher.stats())
        policy_obs_publisher.close()
    if operator_cue is not None:
        operator_cue.close()
    staging_root = Path(str(output_dir) + ".colors.tmp")
    if head_recorder is not None:
        staged_colors = staging_root / "colors"
        if staged_colors.is_dir():
            staged_colors.rename(output_dir / "colors")
            staged_colors.parent.rmdir()
        head_camera_evidence.update(head_recorder.write_manifest(output_dir))
    elif episode_recorder is not None:
        head_camera_evidence.update(episode_recorder.finalize(output_dir))
        if staging_root.is_dir():
            staging_root.rename(output_dir / "episodes")
        rejected_root = Path(str(staging_root) + ".rejected")
        if rejected_root.is_dir():
            rejected_root.rename(output_dir / "episodes_rejected")

    (output_dir / "raw_commands.jsonl").write_text(
        "".join(line + "\n" for line in raw_lines), encoding="utf-8"
    )
    (output_dir / "targets.json").write_text(json.dumps(target_records, indent=2) + "\n", encoding="utf-8")
    (output_dir / "sink_events.json").write_text(json.dumps(sink.events, indent=2) + "\n", encoding="utf-8")
    (output_dir / "sink_acknowledgements.json").write_text(
        json.dumps(sink.acknowledgements, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "control_loop.json").write_text(json.dumps(loop_records, indent=2) + "\n", encoding="utf-8")
    if arm_head_mode:
        np.savez_compressed(
            output_dir / "dynamics_trace.npz",
            elapsed_s=np.asarray(dynamics_time_s, dtype=float),
            root_position_m=np.asarray(dynamics_root_position_m, dtype=float),
            root_linear_velocity_mps=np.asarray(dynamics_root_linear_velocity_mps, dtype=float),
            joint_position_rad=np.asarray(dynamics_joint_position_rad, dtype=float),
            joint_velocity_radps=np.asarray(dynamics_joint_velocity_radps, dtype=float),
            body_com_position_m=np.asarray(dynamics_body_com_position_m, dtype=float),
            joint_names=np.asarray(robot.data.joint_names),
            body_names=np.asarray(robot.data.body_names),
        )

    resolved = dict(config)
    resolved["t007_runtime" if arm_head_mode else "t001_runtime"] = {
        "control_hz": args.control_hz,
        "physics_hz": args.physics_hz,
        "physics_steps_per_control_step": steps_per_control,
        "duration_s": args.duration_s,
        "device": args.device,
        "driven_joints": (list(sink.acknowledgements[-1]["accepted_joints"]) if arm_head_mode and sink.acknowledgements else ["head_yaw_joint", "head_pitch_joint"]),
        "withheld_joints_reason": None if arm_head_mode else "arm_wrist_ik_method_gate",
        "arm_head_config": str(args.arm_head_config) if legacy_arm_head_mode else None,
        "whole_upper_body_config": (
            str(args.whole_upper_body_config)
            if args.whole_upper_body_config is not None
            else None
        ),
        "offline_joint_trajectory": (
            str(args.offline_joint_trajectory) if offline_trajectory_mode else None
        ),
        "offline_joint_trajectory_sha256": (
            _sha256(args.offline_joint_trajectory.expanduser().resolve())
            if offline_trajectory_mode
            else None
        ),
        "offline_continuation_config": (
            str(args.offline_continuation_config) if offline_trajectory_mode else None
        ),
        "upstream_joint_stream_config": (
            str(args.upstream_joint_stream_config) if upstream_stream_mode else None
        ),
        "upstream_solver": (
            "third_party/xr_teleoperate_v1_6 R1_A5_ArmIK, unmodified, solved in a "
            "separate process before this one"
            if upstream_stream_mode
            else None
        ),
        # The effective mode after any --body-mode override, plus the joints it
        # actually drove, so a run is readable without re-deriving them.
        "body_mode": whole_config.body_mode if whole_upper_body_mode else None,
        "held_waist_stiffness": held_waist_stiffness,
        "body_mode_cli_override": args.body_mode if whole_upper_body_mode else None,
        "controlled_joint_names": list(sink.model.joint_names) if whole_upper_body_mode else None,
        "controller_type": (
            "offline_trajectory_continuation"
            if offline_trajectory_mode
            else "upstream_xr_teleoperate_R1_A5_ArmIK"
            if upstream_stream_mode
            else str(dict(experiment_config_payload["whole_upper_body"]).get("controller", {}).get("type", "iterative_pose_ik"))
            if whole_upper_body_mode
            else None
        ),
        "effective_position_scale": (
            whole_config.position_scale
            if whole_upper_body_mode
            and isinstance(whole_config, DifferentialWholeUpperBodyLiveConfig)
            else None
        ),
        "position_scale_cli_override": args.position_scale,
        "effective_velocity_feedforward_task": (
            whole_config.velocity_feedforward_task
            if whole_upper_body_mode
            and isinstance(whole_config, DifferentialWholeUpperBodyLiveConfig)
            else None
        ),
        "velocity_feedforward_task_cli_override": args.velocity_feedforward_task,
        "max_joint_velocity_rad_s_cli_override": args.max_joint_velocity_rad_s,
        "max_joint_acceleration_rad_s2_cli_override": (
            args.max_joint_acceleration_rad_s2
        ),
        "jax_available_in_runtime": importlib.util.find_spec("jax") is not None,
        "command_source": "recorded_replay" if replay_path is not None else "stdin_live",
        "replay_command_file": str(replay_path) if replay_path is not None else None,
        "replay_command_sha256": _sha256(replay_path) if replay_path is not None else None,
        "replay_speed": args.replay_speed if replay_path is not None else None,
        "replay_timestamp_policy": (
            "source intervals preserved; timestamps retimed to local monotonic clock"
            if replay_path is not None
            else None
        ),
        "pelvis_pinned": True,
        "video_fps": args.video_fps if not args.no_video else None,
        "rendering_mode": args.rendering_mode,
        "strict_dataset_fps": bool(args.strict_dataset_fps),
        "render_every_control_step": render_each_control_step,
        "video_resolution": (
            None
            if args.no_video
            else [args.video_width * len(evidence_camera_positions), args.video_height]
        ),
        "evidence_camera_count": 0 if args.no_video else len(evidence_camera_positions),
        "evidence_camera_positions_m": None if args.no_video else [list(p) for p in evidence_camera_positions],
        "evidence_camera_look_at_m": None if args.no_video else list(evidence_camera_look_at),
        "evidence_video_layout": (
            None
            if args.no_video
            else ("single_view" if len(evidence_camera_positions) == 1 else "side_by_side_horizontal")
        ),
        "self_collisions_enabled": not args.disable_self_collisions,
        "fixed_base": arm_head_mode,
        "upper_body_startup_pose": "declared_neutral_joint_position" if arm_head_mode else None,
        "self_collisions_note": (
            "Project asset config enables self-collisions. With them enabled the R1 head joints are "
            "mechanically blocked and hold at zero regardless of the commanded target."
        ),
    }
    if arm_head_mode:
        assert experiment_config_payload is not None
        resolved[
            "t007_offline_continuation_profile"
            if offline_trajectory_mode
            else "t007_whole_upper_body_profile"
            if whole_upper_body_mode
            else "t007_arm_head_profile"
        ] = experiment_config_payload
    write_resolved_config(output_dir, resolved)
    # A T007 run consumes two editable configs.  The shared bridge config is
    # retained separately, while the contract's primary experiment snapshot is
    # the T007 profile that determines arm/IK semantics.
    write_experiment_config(output_dir, experiment_config_payload if arm_head_mode else config)
    if arm_head_mode:
        write_json(output_dir / "bridge_config.json", config)
    if dataset_scene is not None:
        write_json(
            output_dir / "dataset_scene_config.json",
            json.loads(dataset_scene.source_path.read_text(encoding="utf-8")),
        )
    write_runner_command(output_dir)

    holds = [event for event in sink.events if event["event"] == "hold"]
    hold_reasons = sorted({str(event["reason"]) for event in holds})
    latencies = [float(record["age_s"]) for record in target_records if record["is_fresh_command"]]
    metrics = {
        "schema_version": 1,
        "mode": "simulation_only",
        "dataset_scene_config": (
            str(dataset_scene.source_path) if dataset_scene is not None else None
        ),
        "dataset_scene_props": list(scene_props.get("spawned") or []),
        "operator_cue": (
            {
                "enabled": bool(cue_config.enabled),
                "desktop_hud": bool(cue_config.desktop_hud),
                "head_view_overlay": bool(cue_config.head_view_overlay),
                "recorded_in_sensor_images": False,
            }
            if cue_config is not None
            else None
        ),
        **head_camera_evidence,
        "control_step_count": control_step,
        "raw_line_count": len(raw_lines),
        "invalid_line_count": len(invalid_lines),
        "invalid_lines": invalid_lines,
        "accepted_command_count": previous_sequence + 1 if previous_sequence >= 0 else 0,
        "enabled_target_count": sum(1 for record in target_records if record["enabled"]),
        "hold_event_count": len(holds),
        "hold_reasons": hold_reasons,
        "upper_body_dispatch_count": sum(
            1 for event in sink.events if event["event"] in ("upper_body", "whole_upper_body")
        ),
        "base_velocity_dispatch_count": 0,
        "arm_targets_withheld_count": sink.arm_targets_withheld if not arm_head_mode else 0,
        "arm_head_mode": arm_head_mode,
        "whole_upper_body_mode": whole_upper_body_mode,
        "upper_body_accepted_target_count": sum(
            1
            for record in target_records
            if bool(
                dict(record.get("whole_upper_body", record.get("arm_head", {}))).get("accepted")
            )
        ),
        "arm_head_accepted_target_count": sum(
            1
            for record in target_records
            if bool(dict(record.get("arm_head", {})).get("accepted"))
        ),
        "fresh_command_latency_s": {
            "count": len(latencies),
            "min": min(latencies) if latencies else None,
            "max": max(latencies) if latencies else None,
            "mean": sum(latencies) / len(latencies) if latencies else None,
        },
        "stop_reason": stop_reason,
        "achieved_control_hz": (
            control_step / loop_records[-1]["elapsed_s"] if loop_records and loop_records[-1]["elapsed_s"] > 0 else None
        ),
        "requested_control_hz": args.control_hz,
        "physics_step_count": physics_step_count,
        "simulated_time_s": sim_time_s,
        "wall_time_s": loop_records[-1]["elapsed_s"] if loop_records else 0.0,
        "sim_to_wall_ratio": (
            sim_time_s / loop_records[-1]["elapsed_s"]
            if loop_records and loop_records[-1]["elapsed_s"] > 0
            else None
        ),
        "head_tracking_error_rad": _head_tracking_error(target_records),
    }
    if arm_head_mode and dynamics_root_position_m:
        root_positions = np.asarray(dynamics_root_position_m, dtype=float)
        root_velocities = np.asarray(dynamics_root_linear_velocity_mps, dtype=float)
        metrics["fixed_base_dynamics"] = {
            "root_fixed_by_articulation_joint": True,
            "root_initial_position_m": root_positions[0].tolist(),
            "root_max_displacement_m": float(np.max(np.linalg.norm(root_positions - root_positions[0], axis=1))),
            "root_max_linear_velocity_mps": float(np.max(np.linalg.norm(root_velocities, axis=1))),
            "body_com_trace_definition": "Per-link center-of-mass positions in world frame; not a mass-weighted whole-robot COM.",
            "body_count": len(robot.data.body_names),
        }
    if whole_upper_body_mode:
        differential_metrics = _differential_tracking_metrics(target_records, sink.model)
        if differential_metrics is not None:
            metrics["differential_tracking"] = differential_metrics
        offline_metrics = _offline_trajectory_tracking_metrics(target_records, sink.model)
        if offline_metrics is not None:
            metrics["offline_trajectory_tracking"] = offline_metrics
    if upstream_stream_mode:
        # The sink applies vectors it did not solve, so its own accounting --
        # what it clamped, what it never received, how fast it drove the joints
        # with no limiter in the path -- is the only record of that half.
        metrics["upstream_joint_stream"] = sink.summary()
    (output_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    (output_dir / "clock_record.json").write_text(
        json.dumps(
            {
                "clock": "time.monotonic",
                "posix_clock": "CLOCK_MONOTONIC",
                "shared_across_processes": True,
                "basis": (
                    "The bridge and this runner are separate processes on one host, so "
                    "CLOCK_MONOTONIC is a common timebase and command age is measured "
                    "directly rather than estimated from an offset handshake."
                ),
                "runner_start_monotonic_s": start_monotonic,
                "runner_start_utc": start_utc,
                "runner_stop_utc": datetime.now(timezone.utc).isoformat(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    write_metadata(
        output_dir,
        ROOT,
        {
            "record_type": "experiment_run_provenance",
            "protocol_id": (
                dataset_scene.experiment_id
                if dataset_scene is not None and dataset_scene.experiment_id is not None
                else "t007"
                if arm_head_mode
                else "t001_b"
            ),
            "created_at": start_utc,
            "run": {
                    "id": output_dir.name,
                    "output_scope": (
                        "experiment_evidence"
                        if output_dir.is_relative_to(ROOT / "experiments")
                        else "smoke_or_external"
                    ),
                    "path": str(output_dir),
                },
            "execution": {
                "working_directory": str(ROOT),
                "python_executable": sys.executable,
            },
            "source": {"teleop_source_sha256": _source_hashes()},
            "configuration": {
                "bridge_config_path": str(args.config) if arm_head_mode else None,
                "bridge_config_sha256": _sha256(args.config.expanduser().resolve()) if arm_head_mode else None,
                "dataset_scene_config_path": (
                    str(dataset_scene.source_path) if dataset_scene is not None else None
                ),
                "dataset_scene_config_sha256": (
                    _sha256(dataset_scene.source_path) if dataset_scene is not None else None
                ),
                "arm_head_config_path": str(args.arm_head_config) if legacy_arm_head_mode else None,
                "arm_head_config_sha256": _sha256(args.arm_head_config.expanduser().resolve()) if legacy_arm_head_mode else None,
                "whole_upper_body_config_path": (
                    str(args.whole_upper_body_config)
                    if args.whole_upper_body_config is not None
                    else None
                ),
                "whole_upper_body_config_sha256": (
                    _sha256(args.whole_upper_body_config.expanduser().resolve())
                    if args.whole_upper_body_config is not None
                    else None
                ),
                "offline_continuation_config_path": (
                    str(args.offline_continuation_config)
                    if offline_trajectory_mode
                    else None
                ),
                "offline_continuation_config_sha256": (
                    _sha256(args.offline_continuation_config.expanduser().resolve())
                    if offline_trajectory_mode
                    else None
                ),
                "offline_joint_trajectory_path": (
                    str(args.offline_joint_trajectory)
                    if offline_trajectory_mode
                    else None
                ),
                "offline_joint_trajectory_sha256": (
                    _sha256(args.offline_joint_trajectory.expanduser().resolve())
                    if offline_trajectory_mode
                    else None
                ),
            },
            "assets": {
                "r1_usd": {
                    "path": "assets/R1/R1.usd",
                    "sha256": _sha256(ROOT / "assets" / "R1" / "R1.usd"),
                }
            },
        },
    )

    write_evidence_completeness(
        output_dir,
        {
            "clock_record": True,
            "control_loop": True,
            "metrics": True,
            "raw_commands": True,
            "sink_acknowledgements": True,
            "sink_events": True,
            "targets": True,
            "dynamics_trace": arm_head_mode,
            "video": video_recorded,
            "video_reason": None if video_recorded else "Video capture disabled or produced no frames.",
            "arm_wrist_targets_applied": arm_head_mode,
            "arm_wrist_reason": None if arm_head_mode else "Withheld pending the arm/wrist IK method gate; T001 drives head joints only.",
            "arm_head_config_snapshot": legacy_arm_head_mode,
            "whole_upper_body_config_snapshot": args.whole_upper_body_config is not None,
            "offline_continuation_config_snapshot": offline_trajectory_mode,
            "bridge_config_snapshot": arm_head_mode,
            "dataset_scene_config_snapshot": dataset_scene is not None,
        },
    )

    run_failed = dataset_fps_gate_failure is not None
    write_status(
        output_dir,
        "failed" if run_failed else "completed",
        extra={
            "stop_reason": stop_reason,
            "dataset_fps_gate_failure": dataset_fps_gate_failure,
            "dds_or_hardware_called": False,
            "base_velocity_dispatched": False,
        },
    )
    print(json.dumps(metrics, sort_keys=True), flush=True)
    evidence_protocol = (
        dataset_scene.experiment_id.upper()
        if dataset_scene is not None and dataset_scene.experiment_id is not None
        else "T007"
        if arm_head_mode
        else "T001"
    )
    print(f"{evidence_protocol} evidence written to: {output_dir}", flush=True)

    # `SimulationApp.close()` can block indefinitely on this workstation and,
    # even when it returns, Kit may leave non-daemon threads alive. Every
    # evidence file is already on disk, so give close a bounded chance and then
    # terminate without running Python's thread finalizers. Returning from main
    # is insufficient: completed runs have otherwise retained GPU for days.
    sys.stdout.flush()
    sys.stderr.flush()
    _close_simulation_app_and_exit(simulation_app, exit_code=2 if run_failed else 0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
