#!/usr/bin/env python3
"""Run the T003 rate-limited arm/wrist trajectory pilot in Isaac Lab.

This runner is simulation-only.  It derives rest-to-rest minimum-jerk joint
segments from the experiment configuration, replays them through the R1
position targets, and freezes the last emitted target for each declared mapper
safety injection.  It imports no DDS, ROS, Unitree SDK, or hardware module.
"""

from __future__ import annotations

import argparse
import csv
import copy
import json
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("HAPPY_BABY_R1_ROOT", str(ROOT))

from evidence.run_id import allocate_run_id  # noqa: E402
from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_json,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)


RUN_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T003" / "runs"


@dataclass(frozen=True)
class ProgramSegment:
    name: str
    duration_s: float
    trajectory: object | None
    goal_position_m: list[float] | None
    goal_roll_rad: float | None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T003/config/r1_t003_case_matrix.json",
    )
    parser.add_argument(
        "--case-id",
        required=True,
        help="One declared T003 case from the config (for example nominal, deadman, timeout, sequence).",
    )
    parser.add_argument("--output-dir", type=Path, help="New evidence directory; allocated when omitted.")
    parser.add_argument("--no-video", action="store_true", help="Skip video only for a diagnostic run.")
    parser.add_argument("--video-width", type=int, default=640)
    parser.add_argument("--video-height", type=int, default=360)
    return parser


def load_config(path: Path) -> dict:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load T003 config {path}: {exc}") from exc
    if config.get("mode") != "simulation_only" or config.get("experiment_family") != "t003":
        raise SystemExit("T003 requires mode='simulation_only' and experiment_family='t003'.")
    if not isinstance(config.get("cases"), list) or not config["cases"]:
        raise SystemExit("T003 configuration must declare one or more atomic cases.")
    return config


def select_case(config: dict, case_id: str) -> dict:
    """Return one immutable run configuration from the declared case matrix.

    A nominal trajectory and a safety transition answer different questions.
    Keeping them as separate selected configurations prevents a one-run maximum
    from silently mixing normal tracking with an intentional emergency hold.
    """

    matches = [item for item in config["cases"] if item.get("case_id") == case_id]
    if len(matches) != 1:
        known = [str(item.get("case_id")) for item in config["cases"]]
        raise SystemExit(f"Unknown or duplicate T003 case {case_id!r}; choose one of {known}.")
    case = copy.deepcopy(matches[0])
    protocol_id = str(case.get("protocol_id", ""))
    run_prefix = str(case.get("run_prefix", ""))
    if not protocol_id.startswith("t003_") or not run_prefix.startswith(protocol_id):
        raise SystemExit("Each T003 case must declare a t003_* protocol_id and matching run_prefix.")
    selected = copy.deepcopy(config)
    selected.pop("cases", None)
    selected["case"] = case
    selected["protocol_id"] = protocol_id
    selected["run_prefix"] = run_prefix
    selected["safety_injections"] = case.get("safety_injections", [])
    validity = dict(selected["validity"])
    validity["required_safety_events"] = list(case.get("required_safety_events", []))
    selected["validity"] = validity
    return selected


def _ik_config(config: dict):
    from teleop.r1.ik import ArmIKConfig

    declared = config["ik"]
    result = ArmIKConfig(
        position_tolerance_m=float(declared["position_tolerance_m"]),
        roll_tolerance_rad=float(declared["roll_tolerance_rad"]),
        max_iterations=int(declared["max_iterations"]),
        damping=float(declared["damping"]),
        posture_weight=float(declared["posture_weight"]),
        max_joint_step_rad=float(declared["max_joint_step_rad"]),
        posture_tolerance_rad=float(declared["posture_tolerance_rad"]),
    )
    result.validate()
    return result


def build_program(
    config: dict, chain: object, initial_joint_positions_rad: object | None = None
) -> tuple[list[ProgramSegment], list[dict[str, object]]]:
    """Solve configured Cartesian waypoints and time-scale their joint moves."""

    import numpy as np

    from teleop.r1.ik import solve_arm_ik
    from teleop.r1.trajectory import JointTrajectoryLimits, MinimumJerkSegment

    limit_values = config["trajectory_limiter"]
    limits = JointTrajectoryLimits(
        max_velocity_rad_s=np.full(chain.dof, float(limit_values["max_joint_velocity_rad_s"])),
        max_acceleration_rad_s2=np.full(chain.dof, float(limit_values["max_joint_acceleration_rad_s2"])),
        max_jerk_rad_s3=np.full(chain.dof, float(limit_values["max_joint_jerk_rad_s3"])),
    )
    limits.validate()
    ik_config = _ik_config(config)
    nominal = np.zeros(chain.dof)
    seed = np.zeros(chain.dof)
    if initial_joint_positions_rad is None:
        current = np.zeros(chain.dof)
    else:
        current = np.asarray(initial_joint_positions_rad, dtype=float).copy()
        if current.shape != (chain.dof,) or not np.all(np.isfinite(current)):
            raise SystemExit(f"T003 initial arm state must be finite with shape ({chain.dof},), got {current.shape}.")
    segments: list[ProgramSegment] = []
    waypoint_records: list[dict[str, object]] = []
    waypoint_specs = [{"name": "initial", **config["trajectory"]["initial_target"]}, *config["trajectory"]["waypoints"]]
    for declared in waypoint_specs:
        name = str(declared["name"])
        if "hold_s" in declared:
            duration = float(declared["hold_s"])
            if duration <= 0.0:
                raise SystemExit(f"T003 hold {name!r} must have positive hold_s.")
            segments.append(ProgramSegment(name, duration, None, None, None))
            waypoint_records.append({"name": name, "kind": "hold", "duration_s": duration})
            continue
        position = np.asarray(declared["position_m"], dtype=float)
        roll = float(declared["wrist_roll_rad"])
        result = solve_arm_ik(chain, position, roll, seed, nominal, ik_config)
        record = {
            "name": name,
            "kind": "move",
            "target_position_m": position.tolist(),
            "target_wrist_roll_rad": roll,
            "solver_status": result.status,
            "converged": bool(result.converged),
            "iterations": result.iterations,
            "position_residual_m": result.position_residual_m,
            "roll_residual_rad": result.roll_residual_rad,
            "limit_margin_rad": result.limit_margin_rad,
            "clamped_joints": list(result.clamped_joints),
            "joint_goal_rad": result.joint_positions.tolist(),
        }
        if not result.converged:
            raise SystemExit(f"T003 waypoint {name!r} did not converge: {result.status}")
        segment = MinimumJerkSegment.from_limits(current, result.joint_positions, limits)
        segments.append(ProgramSegment(name, segment.duration_s, segment, position.tolist(), roll))
        record["duration_s"] = segment.duration_s
        waypoint_records.append(record)
        current = result.joint_positions.copy()
        seed = result.joint_positions.copy()
    return segments, waypoint_records


def _program_sample(segments: list[ProgramSegment], active_time_s: float):
    """Return the current segment and its commanded derivative sample."""

    elapsed = max(0.0, active_time_s)
    last = segments[-1]
    for segment in segments:
        if elapsed <= segment.duration_s or segment is last:
            if segment.trajectory is None:
                return segment, None
            return segment, segment.trajectory.sample(elapsed)
        elapsed -= segment.duration_s
    return last, None


def _write_csv(path: Path, rows: list[dict[str, object]], joint_names: list[str]) -> None:
    base = ["control_step", "sim_time_s", "active_trajectory_s", "segment", "enabled", "reason"]
    joint_fields = [
        f"{joint}_{quantity}"
        for joint in joint_names
        for quantity in ("target_rad", "target_vel_rad_s", "target_acc_rad_s2", "target_jerk_rad_s3", "observed_rad", "observed_vel_rad_s", "observed_acc_rad_s2", "tracking_error_rad", "torque_nm")
    ]
    endpoint = ["target_endpoint_x_m", "target_endpoint_y_m", "target_endpoint_z_m", "observed_endpoint_x_m", "observed_endpoint_y_m", "observed_endpoint_z_m", "endpoint_error_m"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[*base, *joint_fields, *endpoint])
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = build_parser()
    if "-h" in sys.argv or "--help" in sys.argv:
        parser.print_help()
        return 0
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    config_path = args.config.expanduser().resolve()
    config = select_case(load_config(config_path), args.case_id)
    if args.output_dir is None:
        RUN_ROOT.mkdir(parents=True, exist_ok=True)
        output_dir = RUN_ROOT / allocate_run_id(RUN_ROOT, str(config["run_prefix"]))
    else:
        output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T003 evidence: {output_dir}")

    args.enable_cameras = not args.no_video
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import numpy as np  # noqa: E402
    import isaaclab.sim as sim_utils  # noqa: E402
    import torch  # noqa: E402
    from isaaclab.assets import Articulation  # noqa: E402
    from isaaclab.sensors import Camera, CameraCfg  # noqa: E402

    from teleop.r1.kinematics import load_arm_chain  # noqa: E402
    from teleop.r1.mapping import R1JointOwnership, R1TeleopMapper, TeleopCalibration, TeleopLimits  # noqa: E402
    from teleop.r1.schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3  # noqa: E402
    from training.isaaclab.robot import UNITREE_R1_CFG  # noqa: E402

    trajectory_config = config["trajectory"]
    side = str(trajectory_config["side"])
    offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
    chain = load_arm_chain(side, end_effector_offset_m=offset)
    physics_hz = float(config["simulation"]["physics_hz"])
    control_hz = float(config["simulation"]["control_hz"])
    steps_per_control = int(round(physics_hz / control_hz))
    if steps_per_control < 1 or abs(steps_per_control * control_hz - physics_hz) > 1e-9:
        raise SystemExit("simulation.physics_hz must be an integer multiple of simulation.control_hz.")
    physics_dt = 1.0 / physics_hz
    control_dt = 1.0 / control_hz

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=physics_dt, device=args.device))
    sim.set_camera_view([1.8, -1.6, 1.3], [0.0, 0.0, 0.8])
    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg(intensity=3000.0))
    robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
    robot_cfg.spawn.articulation_props.enabled_self_collisions = bool(config["simulation"]["self_collisions_enabled"])
    robot = Articulation(robot_cfg)

    camera = None
    if not args.no_video:
        camera = Camera(CameraCfg(
            prim_path="/World/EvidenceCamera", update_period=0.0, height=args.video_height, width=args.video_width,
            data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.1, 1.0e5)),
            offset=CameraCfg.OffsetCfg(pos=(1.8, -1.6, 1.3), rot=(0.0, 0.0, 0.0, 1.0), convention="world"),
        ))
    sim.reset()
    if camera is not None:
        camera.set_world_poses_from_view(torch.tensor([[1.8, -1.6, 1.3]], device=sim.device), torch.tensor([[0.0, 0.0, 0.8]], device=sim.device))

    ownership = R1JointOwnership()
    ownership.validate()
    joint_names = list(robot.data.joint_names)
    missing = [name for name in chain.joint_names if name not in joint_names]
    if missing:
        raise SystemExit(f"Articulation is missing T003 arm joints: {missing}")
    if set(chain.joint_names) & set(ownership.lower_body):
        raise SystemExit("Ownership breach: a T003 arm joint is owned by locomotion.")
    joint_ids = [joint_names.index(name) for name in chain.joint_names]
    default_q = robot.data.default_joint_pos.clone()
    robot.write_joint_state_to_sim(default_q, robot.data.default_joint_vel.clone())
    robot.set_joint_position_target(default_q)
    pinned_root_state = robot.data.default_root_state.clone()
    starting_joint_positions = np.array([float(default_q[0, index]) for index in joint_ids])
    # The first segment must start at the *actual* spawn target.  Starting the
    # analytic planner at a hard-coded zero vector creates an uncommanded
    # transient whenever the USD default posture is non-zero and corrupts the
    # dynamic tracking metric before the replay has begun.
    segments, waypoints = build_program(config, chain, starting_joint_positions)
    total_active_duration_s = sum(segment.duration_s for segment in segments)

    mapper = R1TeleopMapper(
        TeleopCalibration(source_frame=str(config["mapper"]["source_frame"])),
        TeleopLimits(command_timeout_s=float(config["mapper"]["command_timeout_s"])),
    )
    injections = [dict(item) for item in config["safety_injections"]]
    started_injections: set[str] = set()
    active_injection: dict[str, object] | None = None
    injection_remaining_s = 0.0
    previous_sequence = -1
    next_sequence = 0
    last_target = starting_joint_positions.copy()
    previous_observed_velocity = np.zeros(chain.dof)
    active_time_s = 0.0
    sim_time_s = 0.0
    records: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    stop_reason = "trajectory_complete"
    start_utc = datetime.now(timezone.utc).isoformat()
    writer = None
    video_path = Path(str(output_dir) + ".video.tmp.mp4")
    if camera is not None:
        import imageio.v2 as imageio

        writer = imageio.get_writer(str(video_path), fps=float(config["simulation"]["video_fps"]), macro_block_size=None)
    next_video_time_s = 0.0
    control_step = 0

    try:
        while simulation_app.is_running() and active_time_s < total_active_duration_s:
            segment, sample = _program_sample(segments, active_time_s)
            due = next((item for item in injections if item["name"] not in started_injections and active_time_s >= float(item["at_active_trajectory_s"])), None)
            if active_injection is None and due is not None:
                active_injection = due
                started_injections.add(str(due["name"]))
                injection_remaining_s = float(due["duration_s"])
                events.append({"event": "injection_start", "sim_time_s": sim_time_s, "active_trajectory_s": active_time_s, **due, "held_target_rad": last_target.tolist()})

            position = Vector3(0.0, 0.0, 0.0)
            pose = Pose(position, Quaternion(0.0, 0.0, 0.0, 1.0))
            reason = None
            if active_injection is not None:
                reason = str(active_injection["reason"])
                candidate_sequence = previous_sequence if reason == "sequence_id_not_increasing" else next_sequence
                timestamp = sim_time_s - 2.0 * float(config["mapper"]["command_timeout_s"]) if reason == "command_timeout" else sim_time_s
                deadman = reason != "deadman_released"
            else:
                candidate_sequence, timestamp, deadman = next_sequence, sim_time_s, True
            command = R1TeleopCommand(candidate_sequence, timestamp, deadman, pose, pose, pose, BaseVelocity.zero(), str(config["mapper"]["source_frame"]))
            if command.sequence_id <= previous_sequence:
                enabled = False
                reason = "sequence_id_not_increasing"
            else:
                mapped = mapper.map(command, sim_time_s)
                enabled = mapped.enabled
                reason = mapped.reason
                previous_sequence = command.sequence_id
                next_sequence = command.sequence_id + 1
            if enabled:
                if sample is None:
                    target = last_target.copy()
                    target_vel = np.zeros(chain.dof)
                    target_acc = np.zeros(chain.dof)
                    target_jerk = np.zeros(chain.dof)
                else:
                    target = sample.position_rad
                    target_vel = sample.velocity_rad_s
                    target_acc = sample.acceleration_rad_s2
                    target_jerk = sample.jerk_rad_s3
                last_target = target.copy()
                active_time_s += control_dt
            else:
                target = last_target.copy()
                target_vel = np.zeros(chain.dof)
                target_acc = np.zeros(chain.dof)
                target_jerk = np.zeros(chain.dof)
                events.append({"event": "hold", "sim_time_s": sim_time_s, "active_trajectory_s": active_time_s, "reason": reason, "held_target_rad": target.tolist()})
                injection_remaining_s -= control_dt
                if injection_remaining_s <= 1e-12:
                    events.append({"event": "injection_end", "sim_time_s": sim_time_s, "active_trajectory_s": active_time_s, "name": active_injection["name"]})
                    active_injection = None

            robot.set_joint_position_target(torch.tensor([target.tolist()], device=robot.device, dtype=torch.float32), joint_ids=joint_ids)
            for _ in range(steps_per_control):
                robot.write_root_state_to_sim(pinned_root_state)
                robot.write_data_to_sim()
                sim.step(render=False)
                sim_time_s += physics_dt
            robot.update(physics_dt)
            observed = np.array([float(robot.data.joint_pos[0, index]) for index in joint_ids])
            observed_velocity = np.array([float(robot.data.joint_vel[0, index]) for index in joint_ids])
            observed_acceleration = (observed_velocity - previous_observed_velocity) / control_dt
            previous_observed_velocity = observed_velocity
            torque_data = getattr(robot.data, "applied_torque", None)
            torques = np.array([float(torque_data[0, index]) for index in joint_ids]) if torque_data is not None else np.full(chain.dof, np.nan)
            target_endpoint = chain.endpoint_position(target)
            observed_endpoint = chain.endpoint_position(observed)
            row: dict[str, object] = {"control_step": control_step, "sim_time_s": sim_time_s, "active_trajectory_s": active_time_s, "segment": segment.name, "enabled": enabled, "reason": reason or ""}
            for index, name in enumerate(chain.joint_names):
                row.update({
                    f"{name}_target_rad": float(target[index]), f"{name}_target_vel_rad_s": float(target_vel[index]),
                    f"{name}_target_acc_rad_s2": float(target_acc[index]), f"{name}_target_jerk_rad_s3": float(target_jerk[index]),
                    f"{name}_observed_rad": float(observed[index]), f"{name}_observed_vel_rad_s": float(observed_velocity[index]),
                    f"{name}_observed_acc_rad_s2": float(observed_acceleration[index]), f"{name}_tracking_error_rad": float(observed[index] - target[index]), f"{name}_torque_nm": float(torques[index]),
                })
            row.update({"target_endpoint_x_m": float(target_endpoint[0]), "target_endpoint_y_m": float(target_endpoint[1]), "target_endpoint_z_m": float(target_endpoint[2]), "observed_endpoint_x_m": float(observed_endpoint[0]), "observed_endpoint_y_m": float(observed_endpoint[1]), "observed_endpoint_z_m": float(observed_endpoint[2]), "endpoint_error_m": float(np.linalg.norm(observed_endpoint - target_endpoint))})
            records.append(row)
            if camera is not None and sim_time_s >= next_video_time_s:
                sim.render()
                camera.update(physics_dt)
                writer.append_data(camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype("uint8"))
                next_video_time_s += 1.0 / float(config["simulation"]["video_fps"])
            control_step += 1
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    finally:
        if writer is not None:
            writer.close()

    output_dir.mkdir(parents=True)
    video_recorded = False
    if video_path.is_file() and video_path.stat().st_size > 0:
        video_path.rename(output_dir / "trajectory.mp4")
        video_recorded = True
    elif video_path.exists():
        video_path.unlink()
    _write_csv(output_dir / "trajectory.csv", records, list(chain.joint_names))
    write_json(output_dir / "waypoints.json", waypoints)
    write_json(output_dir / "events.json", events)
    maxima = {
        "target_velocity_rad_s": max((abs(float(row[f"{name}_target_vel_rad_s"])) for row in records for name in chain.joint_names), default=0.0),
        "target_acceleration_rad_s2": max((abs(float(row[f"{name}_target_acc_rad_s2"])) for row in records for name in chain.joint_names), default=0.0),
        "target_jerk_rad_s3": max((abs(float(row[f"{name}_target_jerk_rad_s3"])) for row in records for name in chain.joint_names), default=0.0),
        "observed_tracking_error_rad": max((abs(float(row[f"{name}_tracking_error_rad"])) for row in records for name in chain.joint_names), default=0.0),
        "observed_endpoint_error_m": max((float(row["endpoint_error_m"]) for row in records), default=0.0),
    }
    # Observed derivatives are kept separate from the analytic command envelope.
    # A safety hold freezes position targets by design and may produce a physical
    # acceleration transient; hiding it behind the planner's zero derivative
    # would incorrectly turn a fail-closed event into a smoothness claim.
    observed_summary: dict[str, dict[str, float | None]] = {}
    for label, suffix in (
        ("velocity_rad_s", "observed_vel_rad_s"),
        ("acceleration_rad_s2", "observed_acc_rad_s2"),
        ("torque_nm", "torque_nm"),
    ):
        values = [
            float(row[f"{name}_{suffix}"])
            for row in records
            for name in chain.joint_names
            if np.isfinite(float(row[f"{name}_{suffix}"]))
        ]
        observed_summary[label] = {
            "peak_absolute": max((abs(value) for value in values), default=None),
            "rms": float(np.sqrt(np.mean(np.square(values)))) if values else None,
        }
    required_events = set(config["validity"]["required_safety_events"])
    hold_reasons = {str(event["reason"]) for event in events if event["event"] == "hold"}
    declared = config["trajectory_limiter"]
    envelope_ok = maxima["target_velocity_rad_s"] <= float(declared["max_joint_velocity_rad_s"]) + 1e-9 and maxima["target_acceleration_rad_s2"] <= float(declared["max_joint_acceleration_rad_s2"]) + 1e-9 and maxima["target_jerk_rad_s3"] <= float(declared["max_joint_jerk_rad_s3"]) + 1e-9
    verification = {"all_waypoints_converged": all(bool(item.get("converged", True)) for item in waypoints), "required_hold_reasons_observed": sorted(required_events) == sorted(hold_reasons), "hold_reasons": sorted(hold_reasons), "lower_body_dispatch_count": 0, "base_velocity_dispatch_count": 0, "target_envelope_ok": envelope_ok, "video_recorded": video_recorded}
    metrics = {"schema_version": 1, "mode": "simulation_only", "side": side, "control_sample_count": len(records), "sim_time_s": sim_time_s, "active_trajectory_duration_s": total_active_duration_s, "stop_reason": stop_reason, "maxima": maxima, "observed_joint_summary": observed_summary, "verification": verification}
    write_json(output_dir / "metrics.json", metrics)
    write_json(output_dir / "verification.json", verification)
    resolved = dict(config)
    resolved["t003_runtime"] = {"device": args.device, "physics_dt_s": physics_dt, "control_dt_s": control_dt, "self_collisions_enabled": bool(config["simulation"]["self_collisions_enabled"]), "starting_arm_joint_positions_rad": starting_joint_positions.tolist(), "video": video_recorded}
    write_resolved_config(output_dir, resolved)
    write_experiment_config(output_dir, config)
    write_runner_command(output_dir)
    write_metadata(output_dir, ROOT, {"record_type": "experiment_run_provenance", "created_at": start_utc, "protocol_id": config["protocol_id"], "case_id": config["case"]["case_id"], "run": {"id": output_dir.name, "path": str(output_dir)}})
    write_evidence_completeness(output_dir, {"waypoints": True, "trajectory_csv": True, "events": True, "metrics": True, "verification": True, "video": video_recorded, "video_reason": None if video_recorded else "Video was disabled or produced no frames."})
    completed = stop_reason == "trajectory_complete" and active_time_s >= total_active_duration_s
    write_status(output_dir, "completed" if completed else "aborted", scientific_outcome="unassessed", reason=None if completed else stop_reason, extra={"dds_or_hardware_called": False, "base_velocity_dispatched": False})
    print(json.dumps(metrics, sort_keys=True), flush=True)
    print(f"T003 evidence written to: {output_dir}", flush=True)
    import threading
    closer = threading.Thread(target=simulation_app.close, daemon=True)
    closer.start()
    closer.join(timeout=30.0)
    if closer.is_alive():
        print("Isaac Sim shutdown did not return within 30 s; forcing exit.", file=sys.stderr, flush=True)
        os._exit(0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
