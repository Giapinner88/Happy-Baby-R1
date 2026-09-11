#!/usr/bin/env python3
"""Launch the canonical upstream arms/head baseline as one command.

Replaces the hand-typed two-process `conda run ... | conda run ...` pipeline
from `experiments/r1_teleop/quest3_sim_v1/baseline/README.md`. It allocates one run id,
derives the stop file and evidence directory from it, and starts the
Quest bridge, vendor R1-A5 IK, and IsaacLab simulator.

    python3 scripts/teleop/run_r1_baseline.py --host-ip 10.42.0.1

This is a simulation-only path. It fixes the root and legs, prohibits base
velocity, and produces no DDS or hardware output. Both wrists are expressed
against the initial head anchor before the vendor IK, so head-only motion and
controller motion remain independent.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.launcher import PilotLaunchSpec, run_pilot  # noqa: E402


EXPERIMENT_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "baseline"
RUN_ROOT = EXPERIMENT_ROOT / "runs"
PROTOCOL = "baseline_upstream"
UPSTREAM_PROFILE = EXPERIMENT_ROOT / "config" / "upstream_stream.json"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host-ip", required=True, help="Host IP shown in the Quest connection URL.")
    parser.add_argument("--duration-s", type=float, default=180.0, help="Lifetime of both processes.")
    parser.add_argument(
        "--cert-file",
        type=Path,
        default=Path.home() / ".config/xr_teleoperate/t001_10_42/cert.pem",
        help="HTTPS certificate whose SAN matches --host-ip.",
    )
    parser.add_argument(
        "--key-file",
        type=Path,
        default=Path.home() / ".config/xr_teleoperate/t001_10_42/key.pem",
        help="Private key paired with --cert-file.",
    )
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument(
        "--device",
        default="cuda:0",
        help="Isaac Lab simulation device, for example cuda:0 or cuda:1.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac without a desktop viewport; useful for transport baselines.",
    )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Disable evidence cameras/video to minimize load during connectivity tests.",
    )
    parser.add_argument(
        "--video-fps",
        type=float,
        default=10.0,
        help="Evidence-video rate; 10 fps preserves the validated 30 Hz control loop.",
    )
    parser.add_argument(
        "--trigger-value-threshold",
        type=float,
        default=5.0,
        help="TeleVuer inverted analog threshold: 10=released, 0=fully pressed.",
    )
    parser.add_argument(
        "--self-collisions",
        action="store_true",
        help=(
            "Keep the project asset's self-collisions enabled. Off by default because with them "
            "enabled the R1 head joints are mechanically blocked and the run shows a motionless head."
        ),
    )
    parser.add_argument(
        "--single-view",
        action="store_true",
        help="Record only the left-side evidence camera instead of both side views.",
    )
    parser.add_argument(
        "--idle-stop-s",
        type=float,
        default=0.0,
        help="Stop after this many seconds with no command; 0 keeps a recoverable WebXR gap alive.",
    )
    parser.add_argument(
        "--quest-ready-timeout-s",
        type=float,
        default=180.0,
        help=(
            "Start Vuer first and wait this long for a validated Enter VR pose before "
            "starting Isaac. This preserves one WebSocket session across startup."
        ),
    )
    parser.add_argument("--stop-file-dir", type=Path, default=Path("/tmp"), help="Where the stop file is created.")
    parser.add_argument("--dry-run", action="store_true", help="Print the allocated paths and commands, run nothing.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profile = UPSTREAM_PROFILE
    if not profile.is_file():
        raise SystemExit(f"Upstream stream profile does not exist: {profile}")
    try:
        profile_payload = json.loads(profile.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Cannot read upstream stream profile {profile}: {exc}") from exc
    if "R1_A5_ArmIK" not in str(profile_payload.get("solver", {}).get("source", "")):
        raise SystemExit(f"{profile} is not an upstream R1_A5_ArmIK profile")
    extra = ["--upstream-joint-stream-config", str(profile), "--video-fps", str(args.video_fps)]
    extra += ["--device", args.device]
    if args.headless:
        extra.append("--headless")
    if args.no_video:
        extra.append("--no-video")
    if not args.no_video and not args.single_view:
        extra.append("--dual-view")

    return run_pilot(
        PilotLaunchSpec(
            protocol=PROTOCOL,
            run_root=RUN_ROOT,
            repo_root=ROOT,
            host_ip=args.host_ip,
            duration_s=args.duration_s,
            cert_file=args.cert_file,
            key_file=args.key_file,
            physics_hz=args.physics_hz,
            control_hz=args.control_hz,
            trigger_value_threshold=args.trigger_value_threshold,
            stop_file_dir=args.stop_file_dir,
            disable_self_collisions=not args.self_collisions,
            extra_sim_args=extra,
            idle_stop_s=args.idle_stop_s,
            quest_ready_timeout_s=args.quest_ready_timeout_s,
            solver_args=["scripts/teleop/run_r1_upstream_ik_stream.py", "--passthrough"],
        ),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
