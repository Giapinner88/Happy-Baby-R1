#!/usr/bin/env python3
"""Launch the T001-B head-only pilot: allocate one run id, then bridge into simulator.

The bridge and the simulator are separate processes in separate Conda
environments, so they cannot each invent a run id — they would disagree. The
shared launcher allocates one free id, derives every path from it, and starts
both processes joined by a pipe.

    python3 scripts/teleop/run_t001_b_pilot.py --host-ip 10.42.0.1

It runs on the host Python 3 and shells out to `conda run` for each side, so it
needs neither the Quest vendor wrapper nor IsaacLab itself.

For the T007 coupled whole-upper-body pilot use
`scripts/teleop/run_t007_upper_body_pilot.py` (or `make teleop`).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.launcher import PilotLaunchSpec, run_pilot  # noqa: E402


RUN_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T001" / "runs"
PROTOCOL = "t001_b"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host-ip", required=True, help="Host IP shown in the Quest connection URL.")
    parser.add_argument("--duration-s", type=float, default=90.0, help="Lifetime of both processes.")
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
    parser.add_argument("--stop-file-dir", type=Path, default=Path("/tmp"), help="Where the stop file is created.")
    parser.add_argument("--dry-run", action="store_true", help="Print the allocated paths and commands, run nothing.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
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
        ),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
