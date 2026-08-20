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
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path


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
        ROOT / "teleop" / "r1" / "upper_body_ik.py",
        ROOT / "teleop" / "r1" / "whole_upper_body.py",
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


def _stdin_reader(sink: "queue.Queue[str | None]") -> None:
    """Feed stdin lines to the control loop without blocking the simulator."""

    for line in sys.stdin:
        line = line.strip()
        if line:
            sink.put(line)
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
    if args.physics_hz < args.control_hz:
        raise SystemExit("--physics-hz must be at least --control-hz.")
    if args.stop_file is not None and args.stop_file.expanduser().exists():
        raise SystemExit(f"Refusing to start: --stop-file already exists: {args.stop_file}")
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T001 evidence: {output_dir}")

    try:
        config = json.loads(args.config.expanduser().resolve().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load teleop JSON config {args.config}: {exc}") from exc
    if config.get("mode") != "simulation_only":
        raise SystemExit("R1 teleop v1 only permits mode='simulation_only'.")
    velocity = config.get("velocity") or {}
    if bool(velocity.get("enabled", False)):
        raise SystemExit("T001 requires base velocity disabled; refusing to run with velocity enabled.")

    args.enable_cameras = not args.no_video
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import isaaclab.sim as sim_utils  # noqa: E402
    import numpy as np  # noqa: E402
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
    from training.isaaclab.robot import UNITREE_R1_CFG  # noqa: E402

    calibration_config = config.get("calibration") or {}
    translation = calibration_config.get("translation_m") or [0.0, 0.0, 0.0]
    mapper = R1TeleopMapper(
        TeleopCalibration(
            translation_m=Vector3(*(float(value) for value in translation)),
            yaw_rad=float(calibration_config.get("yaw_rad", 0.0)),
            source_frame=str(config.get("source_frame", "quest_headset")),
            robot_frame=str(config.get("robot_frame", "r1_base")),
        ),
        TeleopLimits(command_timeout_s=float(config.get("command_timeout_s", 0.5)), allow_velocity=False),
    )

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / args.physics_hz, device=args.device)
    )
    sim.set_camera_view([2.2, 1.6, 1.5], [0.0, 0.0, 0.9])
    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9)).func(
        "/World/Light", sim_utils.DomeLightCfg(intensity=3000.0, color=(0.9, 0.9, 0.9))
    )
    robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
    if args.disable_self_collisions:
        robot_cfg.spawn.articulation_props.enabled_self_collisions = False
    upper_body_mode = args.arm_head_config is not None or args.whole_upper_body_config is not None
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

    sim.reset()
    for camera_instance, position in zip(cameras, evidence_camera_positions):
        camera_instance.set_world_poses_from_view(
            torch.tensor([list(position)], device=sim.device),
            torch.tensor([list(evidence_camera_look_at)], device=sim.device),
        )

    handle = IsaacLabArticulationHandle(robot)
    legacy_arm_head_mode = args.arm_head_config is not None
    whole_upper_body_mode = args.whole_upper_body_config is not None
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
        ik_declared = dict(declared["ik"])
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
            allow_projected_position_solution=bool(
                declared.get("allow_projected_position_solution", False)
            ),
        )
        sink = WholeUpperBodyIsaacLabSink(handle, whole_config)
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
        startup_joint_pos[:, ids] = torch.tensor(
            whole_config.nominal_joint_position_rad,
            device=robot.device,
            dtype=startup_joint_pos.dtype,
        )
    robot.write_joint_state_to_sim(startup_joint_pos, default_joint_vel)
    robot.set_joint_position_target(startup_joint_pos)
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

    commands: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(commands,), daemon=True).start()

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
    previous_sequence = -1
    last_command: R1TeleopCommand | None = None
    last_command_wall_s: float | None = None
    stream_started = False
    stream_closed = False
    steps_per_control = max(1, int(round(args.physics_hz / args.control_hz)))
    max_catchup_steps = steps_per_control * 4
    sim_time_s = 0.0
    physics_step_count = 0
    # A GUI run needs its viewport refreshed every control step; a headless run
    # only renders when it is about to grab an evidence frame.
    render_each_control_step = not bool(getattr(args, "headless", False))
    video_period_s = 1.0 / args.video_fps
    next_video_time = 0.0
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
                    candidate = R1TeleopCommand.from_dict(json.loads(line))
                except (ValueError, TypeError, KeyError, json.JSONDecodeError) as exc:
                    invalid_lines.append({"line_number": len(raw_lines), "error": str(exc)})
                    continue
                if candidate.sequence_id <= previous_sequence:
                    invalid_lines.append(
                        {"line_number": len(raw_lines), "error": "sequence_id did not increase strictly"}
                    )
                    continue
                previous_sequence = candidate.sequence_id
                if arm_head_mode and candidate.reset_requested:
                    sink.reset_session()
                    # A reset is a session boundary. Do not let the previously
                    # held right-trigger command be re-applied later in this
                    # same control cycle or after a temporary input gap.
                    last_command = None
                    last_command_wall_s = None
                    stream_started = True
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
                sim.step(render=False)
            sim_time_s += steps_this_cycle * physics_dt
            physics_step_count += steps_this_cycle
            robot.update(physics_dt)
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
            if render_each_control_step or capture_frame:
                sim.render()

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
        "whole_upper_body_config": str(args.whole_upper_body_config) if whole_upper_body_mode else None,
        # The effective mode after any --body-mode override, plus the joints it
        # actually drove, so a run is readable without re-deriving them.
        "body_mode": whole_config.body_mode if whole_upper_body_mode else None,
        "body_mode_cli_override": args.body_mode if whole_upper_body_mode else None,
        "controlled_joint_names": list(sink.model.joint_names) if whole_upper_body_mode else None,
        "pelvis_pinned": True,
        "video_fps": args.video_fps if not args.no_video else None,
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
            "t007_whole_upper_body_profile" if whole_upper_body_mode else "t007_arm_head_profile"
        ] = experiment_config_payload
    write_resolved_config(output_dir, resolved)
    # A T007 run consumes two editable configs.  The shared bridge config is
    # retained separately, while the contract's primary experiment snapshot is
    # the T007 profile that determines arm/IK semantics.
    write_experiment_config(output_dir, experiment_config_payload if arm_head_mode else config)
    if arm_head_mode:
        write_json(output_dir / "bridge_config.json", config)
    write_runner_command(output_dir)

    holds = [event for event in sink.events if event["event"] == "hold"]
    hold_reasons = sorted({str(event["reason"]) for event in holds})
    latencies = [float(record["age_s"]) for record in target_records if record["is_fresh_command"]]
    metrics = {
        "schema_version": 1,
        "mode": "simulation_only",
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
            "protocol_id": "t007" if arm_head_mode else "t001_b",
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
                "arm_head_config_path": str(args.arm_head_config) if legacy_arm_head_mode else None,
                "arm_head_config_sha256": _sha256(args.arm_head_config.expanduser().resolve()) if legacy_arm_head_mode else None,
                "whole_upper_body_config_path": str(args.whole_upper_body_config) if whole_upper_body_mode else None,
                "whole_upper_body_config_sha256": _sha256(args.whole_upper_body_config.expanduser().resolve()) if whole_upper_body_mode else None,
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
            "whole_upper_body_config_snapshot": whole_upper_body_mode,
            "bridge_config_snapshot": arm_head_mode,
        },
    )

    write_status(
        output_dir,
        "completed",
        extra={
            "stop_reason": stop_reason,
            "dds_or_hardware_called": False,
            "base_velocity_dispatched": False,
        },
    )
    print(json.dumps(metrics, sort_keys=True), flush=True)
    print(f"{'T007' if arm_head_mode else 'T001'} evidence written to: {output_dir}", flush=True)

    # `SimulationApp.close()` can block indefinitely on this workstation. Every
    # evidence file is already on disk, so give the clean shutdown a bounded
    # chance and then force process exit rather than hanging the pipeline.
    sys.stdout.flush()
    sys.stderr.flush()
    closer = threading.Thread(target=simulation_app.close, daemon=True)
    closer.start()
    closer.join(timeout=30.0)
    if closer.is_alive():
        print("Isaac Sim shutdown did not return within 30 s; forcing exit.", file=sys.stderr, flush=True)
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
