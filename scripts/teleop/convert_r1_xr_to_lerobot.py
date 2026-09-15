#!/usr/bin/env python3
"""Convert R1 XR episodes to LeRobot while preserving both head joints."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.lerobot_conversion import convert  # noqa: E402


UPSTREAM_SCRIPT_DIR = ROOT / "third_party/lerobot/r1_lerobot_data_pipeline/scripts"
sys.path.insert(0, str(UPSTREAM_SCRIPT_DIR))
from convert_xr_to_lerobot import (  # noqa: E402
    load_episodes,
    parse_camera_map,
    validate_images,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--profile", choices=("a5", "a7"), default="a5")
    parser.add_argument("--repo-id")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--fps", type=int, help="Override FPS after the raw FPS gate")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument(
        "--camera-map",
        action="append",
        metavar="RAW=NAME",
        help="Repeatable mapping, e.g. color_0=head_camera.",
    )
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--video", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--vcodec", default="h264")
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--push-to-hub", action="store_true")
    parser.add_argument("--private", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be positive")
    if not args.validate_only and (not args.repo_id or not args.output_root):
        parser.error("conversion requires --repo-id and --output-root")
    return args


def main() -> int:
    args = parse_args()
    raw_dir = args.raw_dir.expanduser().resolve()
    camera_map = parse_camera_map(args.camera_map)
    episodes = load_episodes(raw_dir, args.profile, args.fps)

    # Head validation runs even in validate-only mode, before any output exists.
    from teleop.r1.lerobot_conversion import validate_head_contract

    validate_head_contract(episodes)
    frames = validate_images(episodes, camera_map)
    print(
        f"PASS: XR raw arms+head episodes={len(episodes)} frames={frames} "
        f"cameras={list(camera_map)}"
    )
    if not args.validate_only:
        convert(args, episodes, camera_map)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
