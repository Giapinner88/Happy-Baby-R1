#!/usr/bin/env python3
"""Launch the T007 coupled whole-upper-body pilot as one command.

Replaces the hand-typed two-process `conda run ... | conda run ...` pipeline
from `experiments/r1_teleop/quest3_sim_v1/T007/T007.md`. It allocates one run id
under T007, derives the stop file and evidence directory from it, and starts the
Quest bridge piped into the IsaacLab simulator.

    python3 scripts/teleop/run_t007_upper_body_pilot.py --host-ip 10.42.0.1

This is a simulation-only path. It fixes the root and legs, prohibits base
velocity, and produces no DDS or hardware output.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.launcher import PilotLaunchSpec, run_pilot  # noqa: E402


EXPERIMENT_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T007"
RUN_ROOT = EXPERIMENT_ROOT / "runs"
PROTOCOL = "t007_whole_upper_body"
DEFAULT_PROFILE = EXPERIMENT_ROOT / "config" / "r1_t007_whole_upper_body_live.json"


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
    parser.add_argument(
        "--whole-upper-body-config",
        type=Path,
        default=DEFAULT_PROFILE,
        help="Editable T007 schema-3 coupled upper-body profile.",
    )
    parser.add_argument("--physics-hz", type=float, default=100.0)
    parser.add_argument("--control-hz", type=float, default=20.0)
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
        "--body-mode",
        choices=("arms_head", "waist_yaw", "full_upper_body"),
        help=(
            "Which torso joints to drive, overriding the profile: 'arms_head' freezes the "
            "torso and drives both arms + head only (12 joints); 'waist_yaw' is the "
            "hardware-common set (13); 'full_upper_body' adds waist roll, a simulation-only "
            "deviation (14). Freezing the torso stops it chasing out-of-reach hand targets."
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
    parser.add_argument("--stop-file-dir", type=Path, default=Path("/tmp"), help="Where the stop file is created.")
    parser.add_argument("--dry-run", action="store_true", help="Print the allocated paths and commands, run nothing.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profile = args.whole_upper_body_config.expanduser()
    if not profile.is_file():
        raise SystemExit(f"Whole-upper-body profile does not exist: {profile}")

    extra = ["--whole-upper-body-config", str(profile)]
    if not args.single_view:
        extra.append("--dual-view")
    if args.body_mode:
        extra += ["--body-mode", args.body_mode]

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
        ),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
