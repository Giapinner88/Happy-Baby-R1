#!/usr/bin/env python3
"""Gate a raw Unitree-style dataset before assigning a target LeRobot FPS."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def episode_jsons(raw_dir: Path) -> list[Path]:
    paths = sorted(raw_dir.glob("episode_*/data.json"))
    if not paths:
        raise ValueError(f"không tìm thấy episode_*/data.json dưới {raw_dir}")
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--expected-fps", type=float, required=True)
    parser.add_argument("--min-measured-fps", type=float, required=True)
    parser.add_argument("--max-measured-fps", type=float, required=True)
    args = parser.parse_args()

    if not (
        math.isfinite(args.expected_fps)
        and math.isfinite(args.min_measured_fps)
        and math.isfinite(args.max_measured_fps)
        and 0 < args.min_measured_fps <= args.expected_fps <= args.max_measured_fps
    ):
        parser.error("FPS phải hữu hạn, dương và min <= expected <= max")

    raw_dir = args.raw_dir.expanduser().resolve()
    failures: list[str] = []
    frame_count = 0
    source_frame_count = 0
    measured_values: list[float] = []
    try:
        paths = episode_jsons(raw_dir)
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 2

    for path in paths:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            failures.append(f"{path}: không đọc được JSON: {exc}")
            continue
        info = payload.get("info") or {}
        image_info = info.get("image") or {}
        requested = image_info.get("fps_requested")
        measured = image_info.get("fps")
        try:
            requested_value = float(requested)
            measured_value = float(measured)
        except (TypeError, ValueError):
            failures.append(
                f"{path.parent.name}: thiếu fps_requested/fps hợp lệ "
                f"(requested={requested!r}, measured={measured!r})"
            )
            continue
        if not math.isclose(requested_value, args.expected_fps, abs_tol=1e-6):
            failures.append(
                f"{path.parent.name}: fps_requested={requested_value:g}, "
                f"cần {args.expected_fps:g}"
            )
        if not args.min_measured_fps <= measured_value <= args.max_measured_fps:
            failures.append(
                f"{path.parent.name}: FPS đo được {measured_value:g} ngoài "
                f"[{args.min_measured_fps:g}, {args.max_measured_fps:g}]"
            )
        measured_values.append(measured_value)
        if info.get("accepted") is not True:
            failures.append(f"{path.parent.name}: episode không được đánh dấu accepted=true")
        dropped = int(info.get("writer_dropped_capture_count") or 0)
        if dropped:
            failures.append(f"{path.parent.name}: writer đã drop {dropped} capture")

        frames = payload.get("data")
        if not isinstance(frames, list) or not frames:
            failures.append(f"{path.parent.name}: data rỗng hoặc không phải danh sách")
            continue
        frame_count += len(frames)
        previous_source_frame: int | None = None
        for position, frame in enumerate(frames):
            sim_state = frame.get("sim_state") or {}
            source_frame = sim_state.get("source_camera_frame")
            if isinstance(source_frame, bool) or not isinstance(source_frame, int):
                failures.append(
                    f"{path.parent.name} frame {position}: thiếu source_camera_frame nguyên"
                )
            else:
                if previous_source_frame is not None and source_frame <= previous_source_frame:
                    failures.append(
                        f"{path.parent.name} frame {position}: source_camera_frame="
                        f"{source_frame} không tăng sau {previous_source_frame}"
                    )
                previous_source_frame = source_frame
                source_frame_count += 1
            colors = frame.get("colors") or {}
            if not colors:
                failures.append(f"{path.parent.name} frame {position}: thiếu colors")
                continue
            for relative in colors.values():
                image_path = path.parent / str(relative)
                if not image_path.is_file():
                    failures.append(
                        f"{path.parent.name} frame {position}: thiếu ảnh {relative}"
                    )

    if failures:
        print(f"FAIL: source chưa đạt gate {args.expected_fps:g} FPS")
        for failure in failures:
            print(f"  - {failure}")
        return 2

    print(
        f"PASS: episodes={len(paths)} frames={frame_count} "
        f"fps_requested={args.expected_fps:g} "
        f"fps_measured={min(measured_values):g}-{max(measured_values):g} "
        f"unique_source_frames={source_frame_count} "
        "writer_drop=0 images=complete"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
