#!/usr/bin/env python3
"""T002: sweep a declared target grid through IK into the IsaacLab R1 simulator.

Answers two questions in one sweep:

1. Which wrist endpoints are reachable inside the joint limits, and does the
   solver place them at the commanded position in the simulator?
2. Does articulation self-collision block the arm chain the way it blocks the
   head joints? The probe is a controlled A/B — the identical grid is swept twice
   changing only `enabled_self_collisions` — so the two runs are compared, not a
   single run interpreted.

Runs in `unitree_sim_env`. No DDS, ROS, Unitree SDK, `LowCmd`, or
`hardware/high_level/` import, and no base-velocity command is ever produced.

Every numeric threshold comes from the configuration file; this script declares
none of its own.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
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

RUN_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T002" / "runs"
PROTOCOL = "t002"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "experiments/r1_teleop/quest3_sim_v1/T002/config/r1_t002_workspace.json",
    )
    parser.add_argument("--output-dir", type=Path, help="Evidence directory; allocated when omitted.")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--self-collisions",
        dest="self_collisions",
        action="store_true",
        help="Spawn with the project asset's articulation self-collisions enabled.",
    )
    parser.add_argument(
        "--no-self-collisions",
        dest="self_collisions",
        action="store_false",
        help="Spawn with self-collisions disabled; the other half of the A/B probe.",
    )
    parser.set_defaults(self_collisions=True)
    parser.add_argument("--no-video", action="store_true", help="Skip video capture.")
    parser.add_argument("--video-fps", type=float, default=15.0)
    parser.add_argument("--video-width", type=int, default=640)
    parser.add_argument("--video-height", type=int, default=360)
    return parser


def load_config(path: Path) -> dict:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot load T002 config {path}: {exc}") from exc
    if config.get("mode") != "simulation_only":
        raise SystemExit("T002 only permits mode='simulation_only'.")
    return config


def main() -> int:
    parser = build_parser()
    if "-h" in sys.argv or "--help" in sys.argv:
        parser.print_help()
        return 0
    from isaaclab.app import AppLauncher

    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()

    config = load_config(args.config.expanduser().resolve())
    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser().resolve()
    else:
        RUN_ROOT.mkdir(parents=True, exist_ok=True)
        suffix = "collide" if args.self_collisions else "nocollide"
        output_dir = RUN_ROOT / allocate_run_id(RUN_ROOT, f"{PROTOCOL}_{suffix}")
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite T002 evidence: {output_dir}")

    args.enable_cameras = not args.no_video
    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app

    import numpy as np  # noqa: E402
    import isaaclab.sim as sim_utils  # noqa: E402
    import torch  # noqa: E402
    from isaaclab.assets import Articulation  # noqa: E402
    from isaaclab.sensors import Camera, CameraCfg  # noqa: E402

    from teleop.r1.ik import ArmIKConfig, solve_arm_ik  # noqa: E402
    from teleop.r1.kinematics import load_arm_chain  # noqa: E402
    from teleop.r1.mapping import R1JointOwnership  # noqa: E402
    from teleop.r1.workspace import GridSpec, grid_spacing_m, max_consecutive_step_m, serpentine_targets  # noqa: E402
    from training.isaaclab.robot import UNITREE_R1_CFG  # noqa: E402

    ik_config = ArmIKConfig(
        position_tolerance_m=float(config["ik"]["position_tolerance_m"]),
        roll_tolerance_rad=float(config["ik"]["roll_tolerance_rad"]),
        max_iterations=int(config["ik"]["max_iterations"]),
        damping=float(config["ik"]["damping"]),
        posture_weight=float(config["ik"]["posture_weight"]),
        max_joint_step_rad=float(config["ik"]["max_joint_step_rad"]),
        posture_tolerance_rad=float(config["ik"]["posture_tolerance_rad"]),
    )
    ik_config.validate()

    sides = ("left", "right") if args.arm == "both" else (args.arm,)
    grids = {}
    for side in sides:
        declared = config["workspace_grid"][side]
        spec = GridSpec(
            x_range_m=tuple(declared["x_range_m"]),
            y_range_m=tuple(declared["y_range_m"]),
            z_range_m=tuple(declared["z_range_m"]),
            counts=tuple(declared["counts"]),
            wrist_roll_rad=float(declared["wrist_roll_rad"]),
        )
        spec.validate()
        grids[side] = (spec, serpentine_targets(spec))

    physics_dt = 1.0 / float(config["simulation"]["physics_hz"])
    settle_steps = int(config["simulation"]["settle_steps_per_target"])

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=physics_dt, device=args.device))
    sim.set_camera_view([1.4, 1.2, 1.2], [0.0, 0.0, 0.9])
    sim_utils.GroundPlaneCfg().func("/World/GroundPlane", sim_utils.GroundPlaneCfg())
    sim_utils.DomeLightCfg(intensity=3000.0).func("/World/Light", sim_utils.DomeLightCfg(intensity=3000.0))

    robot_cfg = UNITREE_R1_CFG.replace(prim_path="/World/Robot")
    robot_cfg.spawn.articulation_props.enabled_self_collisions = bool(args.self_collisions)
    robot = Articulation(robot_cfg)

    camera = None
    if not args.no_video:
        camera = Camera(
            CameraCfg(
                prim_path="/World/EvidenceCamera",
                update_period=0.0,
                height=args.video_height,
                width=args.video_width,
                data_types=["rgb"],
                spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.1, 1.0e5)),
                offset=CameraCfg.OffsetCfg(pos=(1.4, 1.2, 1.2), rot=(0.0, 0.0, 0.0, 1.0), convention="world"),
            )
        )

    sim.reset()
    if camera is not None:
        camera.set_world_poses_from_view(
            torch.tensor([[1.4, 1.2, 1.2]], device=sim.device),
            torch.tensor([[0.0, 0.0, 0.9]], device=sim.device),
        )

    joint_names = list(robot.data.joint_names)
    ownership = R1JointOwnership()
    arm_indices = {}
    for side in sides:
        offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
        chain = load_arm_chain(side, end_effector_offset_m=offset)
        missing = [name for name in chain.joint_names if name not in joint_names]
        if missing:
            raise SystemExit(f"Articulation is missing arm joints {missing}; the asset cannot serve T002.")
        for name in chain.joint_names:
            if name in ownership.lower_body:
                raise SystemExit(f"Ownership breach: {name} is owned by locomotion, T002 must not write it.")
        arm_indices[side] = [joint_names.index(name) for name in chain.joint_names]

    default_joint_pos = robot.data.default_joint_pos.clone()
    robot.write_joint_state_to_sim(default_joint_pos, robot.data.default_joint_vel.clone())
    robot.set_joint_position_target(default_joint_pos)
    pinned_root_state = robot.data.default_root_state.clone()

    writer = None
    video_path = Path(str(output_dir) + ".video.tmp.mp4")
    if camera is not None:
        import imageio.v2 as imageio

        writer = imageio.get_writer(str(video_path), fps=args.video_fps, macro_block_size=None)

    records: list[dict] = []
    start_utc = datetime.now(timezone.utc).isoformat()
    start_monotonic = time.monotonic()
    stop_reason = "grid_complete"

    try:
        for side in sides:
            offset = tuple(float(value) for value in config["frames"]["end_effector_offset_m"])
            chain = load_arm_chain(side, end_effector_offset_m=offset)
            spec, targets = grids[side]
            indices = arm_indices[side]
            nominal = np.zeros(chain.dof)
            seed = np.zeros(chain.dof)
            previous_solution = None

            for target in targets:
                if not simulation_app.is_running():
                    stop_reason = "simulator_closed"
                    break

                result = solve_arm_ik(
                    chain, target.position_m, target.wrist_roll_rad, seed, nominal, ik_config
                )
                # Seed the next solve with this solution so the sweep stays in one
                # elbow branch; a branch switch is recorded, never smoothed away.
                branch_jump_rad = (
                    float(np.max(np.abs(result.joint_positions - previous_solution)))
                    if previous_solution is not None
                    else 0.0
                )
                seed = result.joint_positions.copy()
                previous_solution = result.joint_positions.copy()

                commanded = result.joint_positions
                if result.converged:
                    robot.set_joint_position_target(
                        torch.tensor([commanded.tolist()], device=robot.device, dtype=torch.float32),
                        joint_ids=indices,
                    )
                for _ in range(settle_steps):
                    robot.write_root_state_to_sim(pinned_root_state)
                    robot.write_data_to_sim()
                    sim.step(render=False)
                robot.update(physics_dt)

                achieved = np.array(
                    [float(robot.data.joint_pos[0, index]) for index in indices], dtype=float
                )
                tracking_error = np.abs(achieved - commanded)
                fk_commanded = chain.endpoint_position(commanded)
                fk_achieved = chain.endpoint_position(achieved)

                records.append(
                    {
                        "side": side,
                        "target_index": target.index,
                        "grid_index": list(target.grid_index),
                        "target_position_m": target.position_m.tolist(),
                        "target_wrist_roll_rad": target.wrist_roll_rad,
                        "solver_status": result.status,
                        "converged": bool(result.converged),
                        "iterations": result.iterations,
                        "position_residual_m": result.position_residual_m,
                        "roll_residual_rad": result.roll_residual_rad,
                        "limit_margin_rad": result.limit_margin_rad,
                        "clamped_joints": list(result.clamped_joints),
                        "branch_jump_rad": branch_jump_rad,
                        "commanded_joint_positions_rad": commanded.tolist(),
                        "achieved_joint_positions_rad": achieved.tolist(),
                        "joint_tracking_error_rad": tracking_error.tolist(),
                        "max_joint_tracking_error_rad": float(np.max(tracking_error)),
                        "fk_commanded_endpoint_m": fk_commanded.tolist(),
                        "fk_achieved_endpoint_m": fk_achieved.tolist(),
                        "achieved_endpoint_error_m": float(np.linalg.norm(fk_achieved - target.position_m)),
                    }
                )

                if camera is not None:
                    sim.render()
                    camera.update(physics_dt)
                    frame = camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
                    writer.append_data(frame.astype("uint8"))
    except KeyboardInterrupt:
        stop_reason = "keyboard_interrupt"
    finally:
        if writer is not None:
            writer.close()

    # Evidence is written before closing Isaac Sim: SimulationApp.close() has been
    # observed not to return on this workstation, and anything after it is lost.
    output_dir.mkdir(parents=True)
    video_recorded = False
    if video_path.is_file() and video_path.stat().st_size > 0:
        video_path.rename(output_dir / "workspace_sweep.mp4")
        video_recorded = True
    elif video_path.exists():
        video_path.unlink()

    write_json(output_dir / "targets.json", records)

    reachable = [record for record in records if record["converged"]]
    threshold = float(config["self_collision_probe"]["joint_tracking_error_threshold_rad"])
    blocked = [
        record
        for record in reachable
        if record["max_joint_tracking_error_rad"] > threshold
    ]
    per_side = {}
    for side in sides:
        side_records = [record for record in records if record["side"] == side]
        side_reachable = [record for record in side_records if record["converged"]]
        side_blocked = [record for record in side_reachable if record["max_joint_tracking_error_rad"] > threshold]
        spec, targets = grids[side]
        per_side[side] = {
            "target_count": len(side_records),
            "converged_count": len(side_reachable),
            "unreachable_count": len(side_records) - len(side_reachable),
            "tracking_blocked_count": len(side_blocked),
            "max_joint_tracking_error_rad": max(
                (record["max_joint_tracking_error_rad"] for record in side_reachable), default=None
            ),
            "mean_joint_tracking_error_rad": (
                sum(record["max_joint_tracking_error_rad"] for record in side_reachable) / len(side_reachable)
                if side_reachable
                else None
            ),
            "max_position_residual_m": max(
                (record["position_residual_m"] for record in side_reachable), default=None
            ),
            "max_branch_jump_rad": max((record["branch_jump_rad"] for record in side_records), default=None),
            "limit_saturated_count": sum(1 for record in side_reachable if record["clamped_joints"]),
            "grid_spacing_m": list(grid_spacing_m(spec)),
            "max_consecutive_target_step_m": max_consecutive_step_m(targets),
            "solver_status_counts": {
                status: sum(1 for record in side_records if record["solver_status"] == status)
                for status in sorted({record["solver_status"] for record in side_records})
            },
        }

    metrics = {
        "schema_version": 1,
        "mode": "simulation_only",
        "self_collisions_enabled": bool(args.self_collisions),
        "arms": list(sides),
        "target_count": len(records),
        "converged_count": len(reachable),
        "unreachable_count": len(records) - len(reachable),
        "tracking_blocked_count": len(blocked),
        "joint_tracking_error_threshold_rad": threshold,
        "base_velocity_dispatch_count": 0,
        "per_side": per_side,
        "wall_time_s": time.monotonic() - start_monotonic,
        "stop_reason": stop_reason,
    }
    write_json(output_dir / "metrics.json", metrics)

    resolved = dict(config)
    resolved["t002_runtime"] = {
        "self_collisions_enabled": bool(args.self_collisions),
        "arms": list(sides),
        "device": args.device,
        "physics_dt_s": physics_dt,
        "settle_steps_per_target": settle_steps,
        "video": video_recorded,
        "self_collisions_note": (
            "This run is one half of the declared A/B probe. A workspace claim needs both halves; "
            "a single run with self-collisions disabled measures joint-limit reachability only."
        ),
    }
    write_resolved_config(output_dir, resolved)
    write_experiment_config(output_dir, config)
    write_runner_command(output_dir)
    write_metadata(
        output_dir,
        ROOT,
        {
            "record_type": "experiment_run_provenance",
            "created_at": start_utc,
            "method_record": config.get("method_record"),
            "run": {"id": output_dir.name, "path": str(output_dir)},
        },
    )
    write_evidence_completeness(
        output_dir,
        {
            "targets": True,
            "metrics": True,
            "video": video_recorded,
            "video_reason": None if video_recorded else "Video capture disabled or produced no frames.",
            "self_collision_probe_complete": False,
            "self_collision_probe_reason": (
                "This is one half of the A/B probe; the comparison needs a paired run with the "
                "opposite self-collision setting."
            ),
            "contact_forces": False,
            "contact_forces_reason": (
                "No contact sensor is attached. Blocking is detected as commanded-versus-achieved "
                "joint tracking error, which is what distinguishes a held target from a blocked one."
            ),
        },
    )
    write_status(
        output_dir,
        "completed" if stop_reason == "grid_complete" else "aborted",
        reason=None if stop_reason == "grid_complete" else stop_reason,
        extra={
            "stop_reason": stop_reason,
            "dds_or_hardware_called": False,
            "base_velocity_dispatched": False,
        },
    )

    print(json.dumps(metrics, sort_keys=True), flush=True)
    print(f"T002 evidence written to: {output_dir}", flush=True)

    sys.stdout.flush()
    sys.stderr.flush()
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
