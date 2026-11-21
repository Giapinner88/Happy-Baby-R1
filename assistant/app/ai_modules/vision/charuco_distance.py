#!/usr/bin/env python3
"""Generate/detect a metric ChArUco target and estimate pose after calibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Tuple


def add_board_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--squares-x", type=int, default=5)
    parser.add_argument("--squares-y", type=int, default=7)
    parser.add_argument("--square-mm", type=float, default=35.0)
    parser.add_argument("--marker-mm", type=float, default=26.0)
    parser.add_argument("--dictionary", default="DICT_5X5_100")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    generate = commands.add_parser("generate", help="Generate an A4, 300-DPI ChArUco page")
    add_board_arguments(generate)
    generate.add_argument("--output", type=Path, required=True)

    calibrate = commands.add_parser("calibrate", help="Calibrate the R1 lens from multiple board views")
    add_board_arguments(calibrate)
    calibrate.add_argument("--images", type=Path, nargs="+", required=True)
    calibrate.add_argument("--output-dir", type=Path, required=True)
    calibrate.add_argument("--camera-model", choices=("fisheye", "pinhole"), default="fisheye")
    calibrate.add_argument("--minimum-views", type=int, default=12)

    detect = commands.add_parser("detect", help="Detect the board, optionally estimate metric pose")
    add_board_arguments(detect)
    detect.add_argument("--image", type=Path, required=True)
    detect.add_argument("--output-dir", type=Path, required=True)
    detect.add_argument("--calibration", type=Path)
    return parser


def make_board(args: argparse.Namespace) -> Tuple[Any, Any]:
    import cv2

    try:
        dictionary_id = getattr(cv2.aruco, args.dictionary)
    except AttributeError as exc:
        raise ValueError(f"unknown ArUco dictionary {args.dictionary}") from exc
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        args.square_mm / 1000.0,
        args.marker_mm / 1000.0,
        dictionary,
    )
    return board, dictionary


def board_metadata(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "squares_xy": [args.squares_x, args.squares_y],
        "square_length_m": args.square_mm / 1000.0,
        "marker_length_m": args.marker_mm / 1000.0,
        "dictionary": args.dictionary,
        "printed_board_size_mm": [args.squares_x * args.square_mm, args.squares_y * args.square_mm],
    }


def generate(args: argparse.Namespace) -> int:
    from PIL import Image

    board, _ = make_board(args)
    dpi = 300
    px_per_mm = dpi / 25.4
    a4_wh = (round(210 * px_per_mm), round(297 * px_per_mm))
    board_wh = (
        round(args.squares_x * args.square_mm * px_per_mm),
        round(args.squares_y * args.square_mm * px_per_mm),
    )
    if board_wh[0] > a4_wh[0] or board_wh[1] > a4_wh[1]:
        raise ValueError(f"board {board_wh} px does not fit A4 {a4_wh} px")
    board_image = board.generateImage(board_wh, marginSize=0, borderBits=1)
    page = Image.new("L", a4_wh, color=255)
    left = (a4_wh[0] - board_wh[0]) // 2
    top = (a4_wh[1] - board_wh[1]) // 2
    page.paste(Image.fromarray(board_image), (left, top))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    page.save(args.output, dpi=(dpi, dpi))
    report = {
        "schema": "happy_baby_r1.charuco_target",
        "schema_version": 1,
        "output": str(args.output.resolve()),
        "page": {"format": "A4", "dpi": dpi, "pixels_wh": list(a4_wh)},
        "board_pixels_wh": list(board_wh),
        "board_offset_xy": [left, top],
        "board": board_metadata(args),
        "print_instruction": (
            "Print A4 at 100% / actual size; disable fit-to-page, "
            f"then verify one square is {args.square_mm:g} mm."
        ),
    }
    metadata_path = args.output.with_suffix(".json")
    metadata_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


def detect_points(image: Any, board: Any) -> tuple[Any, Any, Any, Any]:
    import cv2

    detector = cv2.aruco.CharucoDetector(board)
    return detector.detectBoard(image)


def calibrate(args: argparse.Namespace) -> int:
    import cv2
    import numpy as np

    board, _ = make_board(args)
    object_points = []
    image_points = []
    accepted = []
    rejected = []
    image_size = None
    for path in args.images:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            rejected.append({"path": str(path), "reason": "decode_failed"})
            continue
        size = (image.shape[1], image.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            rejected.append({"path": str(path), "reason": f"size_mismatch_{size}"})
            continue
        charuco_corners, charuco_ids, _, _ = detect_points(image, board)
        count = 0 if charuco_ids is None else len(charuco_ids)
        if count < 8:
            rejected.append({"path": str(path), "reason": f"only_{count}_charuco_corners"})
            continue
        obj, img = board.matchImagePoints(charuco_corners, charuco_ids)
        object_points.append(np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3))
        image_points.append(np.asarray(img, dtype=np.float64).reshape(-1, 1, 2))
        accepted.append({"path": str(path.resolve()), "charuco_corner_count": int(count)})
    if len(accepted) < args.minimum_views:
        raise RuntimeError(
            f"only {len(accepted)} usable views; need at least {args.minimum_views}. "
            "Move the board across the image and vary tilt/distance."
        )
    assert image_size is not None

    if args.camera_model == "fisheye":
        camera_matrix = np.eye(3, dtype=np.float64)
        camera_matrix[0, 0] = camera_matrix[1, 1] = max(image_size)
        camera_matrix[0, 2] = image_size[0] / 2.0
        camera_matrix[1, 2] = image_size[1] / 2.0
        dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.fisheye.calibrate(
            object_points,
            image_points,
            image_size,
            camera_matrix,
            dist_coeffs,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8),
        )
    else:
        flags = 0
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            [points.astype(np.float32) for points in object_points],
            [points.astype(np.float32) for points in image_points],
            image_size,
            None,
            None,
            flags=flags,
        )

    per_view_rms = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        if args.camera_model == "fisheye":
            projected, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        else:
            projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        residual = projected.reshape(-1, 2) - img.reshape(-1, 2)
        per_view_rms.append(float(np.sqrt(np.mean(np.sum(residual * residual, axis=1)))))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    calibration_path = args.output_dir / "calibration.npz"
    np.savez_compressed(
        calibration_path,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        camera_model=np.asarray(args.camera_model),
        image_size_wh=np.asarray(image_size, dtype=np.int32),
    )
    report = {
        "schema": "happy_baby_r1.camera_calibration",
        "schema_version": 1,
        "camera_model": args.camera_model,
        "image_size_wh": list(image_size),
        "board": board_metadata(args),
        "accepted_views": accepted,
        "rejected_views": rejected,
        "rms_reprojection_px": float(rms),
        "per_view_rms_px": per_view_rms,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "calibration": str(calibration_path.resolve()),
        "acceptance_note": "Inspect coverage and residuals; RMS alone does not validate metric accuracy.",
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0


def detect(args: argparse.Namespace) -> int:
    import cv2
    import numpy as np

    if not args.image.is_file():
        raise FileNotFoundError(args.image)
    board, _ = make_board(args)
    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot decode {args.image}")
    charuco_corners, charuco_ids, marker_corners, marker_ids = detect_points(image, board)
    marker_count = 0 if marker_ids is None else int(len(marker_ids))
    corner_count = 0 if charuco_ids is None else int(len(charuco_ids))
    annotated = image.copy()
    if marker_ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
    if charuco_ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(annotated, charuco_corners, charuco_ids)

    pose = None
    if args.calibration is not None:
        if corner_count < 4:
            raise RuntimeError("at least four ChArUco corners are required for pose")
        calibration = np.load(args.calibration)
        camera_matrix = calibration["camera_matrix"]
        dist_coeffs = calibration["dist_coeffs"]
        camera_model = str(calibration["camera_model"]) if "camera_model" in calibration else "pinhole"
        if "image_size_wh" in calibration:
            calibrated_size = tuple(int(value) for value in calibration["image_size_wh"])
            if calibrated_size != (image.shape[1], image.shape[0]):
                raise RuntimeError(
                    f"calibration size {calibrated_size} does not match image "
                    f"{(image.shape[1], image.shape[0])}"
                )
        object_points, image_points = board.matchImagePoints(charuco_corners, charuco_ids)
        pnp_dist_coeffs = dist_coeffs
        if camera_model == "fisheye":
            image_points = cv2.fisheye.undistortPoints(
                np.asarray(image_points, dtype=np.float64),
                camera_matrix,
                dist_coeffs,
                P=camera_matrix,
            )
            pnp_dist_coeffs = np.zeros((4, 1), dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            pnp_dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed")
        rotation, _ = cv2.Rodrigues(rvec)
        center_board = np.array(
            [[args.squares_x * args.square_mm / 2000.0],
             [args.squares_y * args.square_mm / 2000.0],
             [0.0]],
            dtype=np.float64,
        )
        center_camera = rotation @ center_board + tvec
        pose = {
            "board_origin_tvec_m": tvec.reshape(-1).tolist(),
            "board_center_camera_xyz_m": center_camera.reshape(-1).tolist(),
            "board_center_range_m": float(np.linalg.norm(center_camera)),
            "board_center_optical_z_m": float(center_camera[2, 0]),
            "rvec_rad": rvec.reshape(-1).tolist(),
        }
        if camera_model == "pinhole":
            cv2.drawFrameAxes(annotated, camera_matrix, dist_coeffs, rvec, tvec, 0.08)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output_dir / "detection.png"), annotated)
    report = {
        "schema": "happy_baby_r1.charuco_detection",
        "schema_version": 1,
        "image": str(args.image.resolve()),
        "board": board_metadata(args),
        "marker_count": marker_count,
        "charuco_corner_count": corner_count,
        "calibration": str(args.calibration.resolve()) if args.calibration else None,
        "pose": pose,
        "metric_status": "measured" if pose is not None else "unavailable_without_calibration",
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))
    return 0 if corner_count >= 4 else 2


def main() -> int:
    args = build_parser().parse_args()
    if args.squares_x < 2 or args.squares_y < 2:
        raise ValueError("board needs at least 2x2 squares")
    if not 0 < args.marker_mm < args.square_mm:
        raise ValueError("marker-mm must be positive and smaller than square-mm")
    if args.command == "generate":
        return generate(args)
    if args.command == "calibrate":
        return calibrate(args)
    return detect(args)


if __name__ == "__main__":
    raise SystemExit(main())
