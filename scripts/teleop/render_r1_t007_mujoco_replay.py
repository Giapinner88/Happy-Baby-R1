#!/usr/bin/env python3
"""Render quantitative video from a completed T007 MuJoCo replay.

The controller is not rerun. Recorded joint states and Cartesian target/actual
traces are interpolated and rendered with the resolved source-run model.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


DEFAULT_RUN = Path(
    "experiments/r1_teleop/quest3_sim_v1/T007/runs/"
    "t007_mujoco_replay_20260820T122648Z"
)

MARKERS = (
    ("desired_left", (0.00, 0.90, 1.00, 0.82), 0.025),
    ("actual_left", (0.10, 1.00, 0.20, 0.92), 0.017),
    ("desired_right", (1.00, 0.10, 0.90, 0.82), 0.025),
    ("actual_right", (1.00, 0.62, 0.05, 0.92), 0.017),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="New output directory (default: RUN/analysis/video_validation).",
    )
    parser.add_argument("--case", help="Defaults to metrics primary_case.")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--view-width", type=int, default=640)
    parser.add_argument("--view-height", type=int, default=480)
    parser.add_argument("--max-duration-s", type=float, help="Smoke-render prefix.")
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_font(size: int) -> ImageFont.ImageFont:
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ):
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def make_render_model(source_xml: Path, output_xml: Path) -> None:
    tree = ET.parse(source_xml)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError(f"No <worldbody> in {source_xml}")
    for name, rgba, radius in MARKERS:
        body = ET.SubElement(
            worldbody,
            "body",
            name=f"video_{name}",
            mocap="true",
            pos="0 0 0",
        )
        ET.SubElement(
            body,
            "geom",
            name=f"video_{name}_geom",
            type="sphere",
            size=f"{radius:.4f}",
            rgba=" ".join(f"{value:.3f}" for value in rgba),
            contype="0",
            conaffinity="0",
            group="1",
        )
    tree.write(output_xml, encoding="unicode")


def interpolate(t: np.ndarray, values: np.ndarray, at: float) -> np.ndarray:
    if values.ndim == 1:
        return np.asarray(np.interp(at, t, values))
    return np.asarray([np.interp(at, t, values[:, i]) for i in range(values.shape[1])])


def make_camera(azimuth: float) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.27, 0.0, 1.00)
    camera.distance = 1.45
    camera.azimuth = azimuth
    camera.elevation = -15.0
    return camera


def overlay(
    frame: np.ndarray,
    *,
    at: float,
    duration: float,
    left_error_mm: float,
    right_error_mm: float,
    target_head_deg: np.ndarray,
    actual_head_deg: np.ndarray,
    title_font: ImageFont.ImageFont,
    text_font: ImageFont.ImageFont,
) -> np.ndarray:
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image, "RGBA")
    width, height = image.size
    draw.rectangle((0, 0, width, 94), fill=(6, 10, 17, 205))
    draw.text(
        (14, 8),
        f"T007 MuJoCo trajectory validation | t = {at:6.2f} / {duration:6.2f} s",
        font=title_font,
        fill=(255, 255, 255, 255),
    )
    draw.text(
        (14, 38),
        f"Wrist error: left {left_error_mm:5.1f} mm | right {right_error_mm:5.1f} mm",
        font=text_font,
        fill=(235, 240, 246, 255),
    )
    draw.text(
        (14, 66),
        "Head pitch/yaw [deg]: "
        f"target {target_head_deg[0]:+5.1f}/{target_head_deg[1]:+5.1f} | "
        f"actual {actual_head_deg[0]:+5.1f}/{actual_head_deg[1]:+5.1f}",
        font=text_font,
        fill=(235, 240, 246, 255),
    )
    legend = (
        ("target L", (0, 230, 255)),
        ("actual L", (26, 255, 51)),
        ("target R", (255, 26, 230)),
        ("actual R", (255, 158, 13)),
    )
    x = width - 452
    for label, color in legend:
        draw.ellipse((x, 15, x + 14, 29), fill=(*color, 255))
        draw.text((x + 20, 12), label, font=text_font, fill="white")
        x += 108
    draw.text((14, height - 28), "front-left view", font=text_font, fill="white")
    draw.text(
        (width // 2 + 14, height - 28),
        "front-right view",
        font=text_font,
        fill="white",
    )
    return np.asarray(image)


def main() -> int:
    args = parse_args()
    if args.fps <= 0 or args.view_width <= 0 or args.view_height <= 0:
        raise ValueError("fps and view dimensions must be positive")
    run_dir = args.run_dir.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir
        else run_dir / "analysis" / "video_validation"
    )
    if output_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_dir}")

    metrics_path = run_dir / "metrics.json"
    trace_path = run_dir / "trajectory_trace.npz"
    source_xml = run_dir / "resolved_mujoco_model.xml"
    for path in (metrics_path, trace_path, source_xml):
        if not path.is_file():
            raise FileNotFoundError(path)
    output_dir.mkdir(parents=True)

    metrics = json.loads(metrics_path.read_text())
    case = args.case or metrics["primary_case"]
    prefix = f"{case}__"
    with np.load(trace_path, allow_pickle=False) as trace:
        required = (
            "t",
            "truth_lp",
            "truth_rp",
            "actual_lp",
            "actual_rp",
            "truth_ha",
            "actual_ha",
            "q",
        )
        arrays = {}
        for suffix in required:
            key = prefix + suffix
            if key not in trace:
                raise KeyError(f"Missing trace array: {key}")
            arrays[suffix] = np.asarray(trace[key])
        joint_names = [str(name) for name in trace["joint_names"]]

    times = arrays["t"] - arrays["t"][0]
    duration = float(times[-1])
    if args.max_duration_s is not None:
        if args.max_duration_s <= 0:
            raise ValueError("--max-duration-s must be positive")
        duration = min(duration, args.max_duration_s)
    frame_times = np.arange(int(np.floor(duration * args.fps)) + 1) / args.fps

    render_xml = output_dir / "render_model.xml"
    make_render_model(source_xml, render_xml)
    model = mujoco.MjModel.from_xml_path(str(render_xml))
    data = mujoco.MjData(model)
    qpos_addresses = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise KeyError(f"Joint missing from resolved model: {name}")
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    if pelvis_id < 0:
        raise KeyError("Body 'pelvis' missing from resolved model")
    mocap_ids = {}
    for name, _, _ in MARKERS:
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"video_{name}"
        )
        mocap_ids[name] = int(model.body_mocapid[body_id])

    left_camera = make_camera(145.0)
    right_camera = make_camera(215.0)
    title_font = load_font(20)
    text_font = load_font(17)
    video_path = output_dir / "trajectory_tracking_validation.mp4"

    os.environ.setdefault("MUJOCO_GL", "egl")
    renderer = mujoco.Renderer(model, height=args.view_height, width=args.view_width)
    writer = imageio.get_writer(
        video_path,
        fps=args.fps,
        codec="libx264",
        quality=8,
        macro_block_size=None,
    )
    try:
        for frame_index, at in enumerate(frame_times):
            data.qpos[:] = model.qpos0
            data.qvel[:] = 0.0
            data.qpos[qpos_addresses] = interpolate(times, arrays["q"], float(at))
            mujoco.mj_forward(model, data)
            pelvis_world = data.xpos[pelvis_id].copy()
            positions = {
                "desired_left": interpolate(times, arrays["truth_lp"], float(at)),
                "actual_left": interpolate(times, arrays["actual_lp"], float(at)),
                "desired_right": interpolate(times, arrays["truth_rp"], float(at)),
                "actual_right": interpolate(times, arrays["actual_rp"], float(at)),
            }
            for name, position in positions.items():
                data.mocap_pos[mocap_ids[name]] = pelvis_world + position
                data.mocap_quat[mocap_ids[name]] = (1.0, 0.0, 0.0, 0.0)
            mujoco.mj_forward(model, data)

            renderer.update_scene(data, camera=left_camera)
            left_view = renderer.render().copy()
            renderer.update_scene(data, camera=right_camera)
            right_view = renderer.render().copy()
            combined = np.concatenate((left_view, right_view), axis=1)
            target_head = interpolate(times, arrays["truth_ha"], float(at))
            actual_head = interpolate(times, arrays["actual_ha"], float(at))
            combined = overlay(
                combined,
                at=float(at),
                duration=duration,
                left_error_mm=1000.0 * float(
                    np.linalg.norm(positions["desired_left"] - positions["actual_left"])
                ),
                right_error_mm=1000.0 * float(
                    np.linalg.norm(positions["desired_right"] - positions["actual_right"])
                ),
                target_head_deg=np.rad2deg(target_head),
                actual_head_deg=np.rad2deg(actual_head),
                title_font=title_font,
                text_font=text_font,
            )
            writer.append_data(combined)
            if frame_index % max(1, int(args.fps * 10)) == 0:
                print(
                    f"rendered {frame_index + 1}/{len(frame_times)} frames "
                    f"({at:.1f}/{duration:.1f} s)",
                    flush=True,
                )
    finally:
        writer.close()
        renderer.close()

    manifest = {
        "schema_version": 1,
        "artifact_role": "derived_visual_validation",
        "interpretation": (
            "Playback of recorded MuJoCo joint state and recorded Cartesian "
            "target/actual traces; this is not a controller rerun."
        ),
        "source_run": str(run_dir),
        "source_files": {
            name: {"path": str(path), "sha256": sha256(path)}
            for name, path in (
                ("metrics", metrics_path),
                ("trajectory_trace", trace_path),
                ("resolved_mujoco_model", source_xml),
            )
        },
        "case": case,
        "duration_s": duration,
        "fps": args.fps,
        "frame_count": len(frame_times),
        "video": {
            "path": str(video_path),
            "sha256": sha256(video_path),
            "width": 2 * args.view_width,
            "height": args.view_height,
            "codec": "libx264",
        },
        "marker_legend": {
            "cyan": "desired left wrist",
            "green": "recorded actual left wrist",
            "magenta": "desired right wrist",
            "orange": "recorded actual right wrist",
        },
        "source_acceptance_passed": metrics.get("all_declared_acceptance_checks_pass"),
    }
    (output_dir / "video_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    print(f"video: {video_path}")
    print(f"manifest: {output_dir / 'video_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
