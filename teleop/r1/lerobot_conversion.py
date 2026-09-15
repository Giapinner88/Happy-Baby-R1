"""Project-owned R1 XR → LeRobot conversion contracts.

The upstream converter under ``third_party/lerobot`` intentionally exports only
the arm profile.  D002 moves the camera through the two head joints, so its VLA
contract appends ``head_pitch`` and ``head_yaw`` to both state and action while
reusing the upstream episode/image parsing code unchanged.
"""

from __future__ import annotations

import json
import sys
from argparse import Namespace
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
UPSTREAM_SCRIPT_DIR = ROOT / "third_party/lerobot/r1_lerobot_data_pipeline/scripts"
HEAD_RAW_NAMES = ("head_pitch_joint", "head_yaw_joint")
HEAD_FEATURE_NAMES = ("head_pitch.q", "head_yaw.q")


def _load_upstream_converter() -> Any:
    """Import the unmodified helper script from its own script directory."""

    script_dir = str(UPSTREAM_SCRIPT_DIR)
    inserted = script_dir not in sys.path
    if inserted:
        sys.path.insert(0, script_dir)
    try:
        import convert_xr_to_lerobot as upstream
    finally:
        if inserted:
            sys.path.remove(script_dir)
    return upstream


def feature_name(raw_joint_name: str) -> str:
    """Map the raw URDF joint name to the existing LeRobot ``*.q`` convention."""

    suffix = "_joint"
    if not raw_joint_name.endswith(suffix):
        raise ValueError(f"joint name must end in {suffix!r}: {raw_joint_name!r}")
    return f"{raw_joint_name.removesuffix(suffix)}.q"


def validate_head_contract(episodes: list[Any]) -> tuple[str, ...]:
    """Require the same two ordered head joints in every raw episode and frame."""

    upstream = _load_upstream_converter()
    for episode in episodes:
        data_path = episode.path / "data.json"
        try:
            payload = json.loads(data_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot read head contract from {data_path}: {exc}") from exc
        body_names = tuple((payload.get("info", {}).get("joint_names", {}) or {}).get("body") or ())
        if body_names != HEAD_RAW_NAMES:
            raise ValueError(
                f"{data_path}: info.joint_names.body must be {list(HEAD_RAW_NAMES)}, "
                f"got {list(body_names)}"
            )
        for position, frame in enumerate(episode.frames):
            for section in ("states", "actions"):
                qpos = upstream.nested_qpos(
                    frame[section], "body", episode=data_path, index=position
                )
                if qpos.shape != (len(HEAD_RAW_NAMES),):
                    raise ValueError(
                        f"{data_path} frame {position}: {section}.body.qpos has "
                        f"{len(qpos)} values, expected {len(HEAD_RAW_NAMES)}"
                    )
    return HEAD_FEATURE_NAMES


def joint_vector(content: dict[str, Any], *, episode: Path, index: int) -> np.ndarray:
    """Return left arm, right arm, head pitch, head yaw in that exact order."""

    upstream = _load_upstream_converter()
    return np.concatenate(
        [
            upstream.nested_qpos(content, group, episode=episode, index=index)
            for group in ("left_arm", "right_arm", "body")
        ]
    ).astype(np.float32)


def write_manifest(
    output_root: Path,
    args: Namespace,
    episodes: list[Any],
    camera_map: dict[str, str],
    names: list[str],
) -> None:
    manifest = {
        "schema": "happy_baby_r1.xr_to_lerobot_conversion",
        "schema_version": 2,
        "source_format": "xr_teleoperate.EpisodeWriter",
        "source_root": str(args.raw_dir.resolve()),
        "profile": args.profile,
        "joint_contract": "arms_head",
        "joint_order": names,
        "state_dim": len(names),
        "action_dim": len(names),
        "camera_map": camera_map,
        "source_episodes": [str(ep.path.resolve()) for ep in episodes],
        "episodes": len(episodes),
        "frames": sum(len(ep.frames) for ep in episodes),
        "state_source": "post_physics_joint_position",
        "action_source": "xr_ik_joint_target",
    }
    meta = output_root / "meta"
    meta.mkdir(parents=True, exist_ok=True)
    (meta / "r1_conversion.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def convert(args: Namespace, episodes: list[Any], camera_map: dict[str, str]) -> None:
    """Convert validated raw episodes to a 12-DoF arms+head LeRobot dataset."""

    upstream = _load_upstream_converter()
    if not args.repo_id or not args.output_root:
        raise ValueError("conversion requires --repo-id and --output-root")
    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite output dataset: {output_root}")

    validate_head_contract(episodes)
    width, height = upstream.infer_size(
        episodes, camera_map, args.width, args.height
    )
    names = [*upstream.arm_names(args.profile), *HEAD_FEATURE_NAMES]
    image_dtype = "video" if args.video else "image"
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(names),),
            "names": names,
        },
        "action": {"dtype": "float32", "shape": (len(names),), "names": names},
    }
    for destination in camera_map.values():
        features[f"observation.images.{destination}"] = {
            "dtype": image_dtype,
            "shape": (height, width, 3),
            "names": ["height", "width", "channels"],
        }

    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        root=output_root,
        fps=episodes[0].fps,
        robot_type=f"unitree_r1_{args.profile}_arms_head",
        features=features,
        use_videos=args.video,
        image_writer_processes=0,
        image_writer_threads=args.image_writer_threads,
        rgb_encoder=RGBEncoderConfig(vcodec=args.vcodec) if args.video else None,
    )
    try:
        for episode in episodes:
            data_path = episode.path / "data.json"
            for position, frame in enumerate(episode.frames):
                row: dict[str, Any] = {
                    "observation.state": joint_vector(
                        frame["states"], episode=data_path, index=position
                    ),
                    "action": joint_vector(
                        frame["actions"], episode=data_path, index=position
                    ),
                    "task": episode.task,
                }
                for source, destination in camera_map.items():
                    row[f"observation.images.{destination}"] = upstream.read_rgb(
                        upstream.resolve_image(episode, frame, source, position),
                        (width, height),
                    )
                dataset.add_frame(row)
            dataset.save_episode()
    finally:
        dataset.finalize()

    write_manifest(output_root, args, episodes, camera_map, names)
    print(
        f"PASS: converted arms+head episodes={len(episodes)} "
        f"frames={sum(len(ep.frames) for ep in episodes)} fps={episodes[0].fps} "
        f"state={len(names)} action={len(names)} size={width}x{height} root={output_root}"
    )
    if args.push_to_hub:
        dataset.push_to_hub(private=args.private)
        print(f"PASS: pushed https://huggingface.co/datasets/{args.repo_id}")


__all__ = [
    "HEAD_FEATURE_NAMES",
    "HEAD_RAW_NAMES",
    "convert",
    "feature_name",
    "joint_vector",
    "validate_head_contract",
]
