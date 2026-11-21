#!/usr/bin/env python3
"""Run a metric monocular-depth model on an R1 RGB frame.

The output is an estimate, not a calibrated safety measurement.  In particular,
the R1 head image is wide-angle and this script does not remove lens distortion
unless the caller supplies an already-undistorted input image.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import time
from pathlib import Path
from typing import Any


DEFAULT_MODEL = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="auto")
    return parser


def percentile(values: Any, q: float) -> float:
    import numpy as np

    return float(np.percentile(values, q))


def main() -> int:
    args = build_parser().parse_args()
    if not args.image.is_file():
        raise FileNotFoundError(args.image)

    import cv2
    import numpy as np
    import torch
    import transformers
    from PIL import Image
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("--device cuda was requested but CUDA is unavailable")
    device = "cuda" if args.device == "auto" and torch.cuda.is_available() else args.device
    if device == "auto":
        device = "cpu"

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    image_bytes = args.image.read_bytes()
    rgb = Image.open(args.image).convert("RGB")
    width, height = rgb.size

    load_started = time.perf_counter()
    processor = AutoImageProcessor.from_pretrained(args.model)
    model = AutoModelForDepthEstimation.from_pretrained(args.model).to(device)
    model.eval()
    load_s = time.perf_counter() - load_started

    inputs = processor(images=rgb, return_tensors="pt")
    inputs = {name: value.to(device) for name, value in inputs.items()}
    inference_started = time.perf_counter()
    with torch.inference_mode():
        outputs = model(**inputs)
    if device == "cuda":
        torch.cuda.synchronize()
    inference_s = time.perf_counter() - inference_started
    processed = processor.post_process_depth_estimation(
        outputs, target_sizes=[(height, width)]
    )[0]["predicted_depth"]
    depth_m = processed.detach().float().cpu().numpy().astype(np.float32)

    finite = np.isfinite(depth_m)
    if not finite.all() or not finite.any():
        raise RuntimeError("model produced non-finite depth values")
    if depth_m.shape != (height, width):
        raise RuntimeError(f"unexpected depth shape {depth_m.shape}, expected {(height, width)}")

    # A small center ROI is less sensitive than a single pixel to decoder noise.
    half = max(2, min(height, width) // 40)
    cy, cx = height // 2, width // 2
    center_roi = depth_m[cy - half : cy + half + 1, cx - half : cx + half + 1]
    statistics = {
        "min_m": float(depth_m.min()),
        "p05_m": percentile(depth_m, 5),
        "median_m": percentile(depth_m, 50),
        "p95_m": percentile(depth_m, 95),
        "max_m": float(depth_m.max()),
        "center_roi_median_m": percentile(center_roi, 50),
        "center_roi_p05_m": percentile(center_roi, 5),
        "center_roi_p95_m": percentile(center_roi, 95),
        "center_roi_bounds_xyxy": [cx - half, cy - half, cx + half, cy + half],
    }

    np.savez_compressed(output_dir / "depth_m.npz", depth_m=depth_m)
    lo, hi = statistics["p05_m"], statistics["p95_m"]
    normalized = np.clip((depth_m - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    colored = cv2.applyColorMap((normalized * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    cv2.rectangle(colored, (cx - half, cy - half), (cx + half, cy + half), (255, 255, 255), 2)
    cv2.putText(
        colored,
        f"AI estimate; center median {statistics['center_roi_median_m']:.2f} m",
        (24, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        colored,
        "NOT CALIBRATED / NOT FOR SAFETY",
        (24, height - 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(output_dir / "depth_visualization.png"), colored):
        raise RuntimeError("failed to save depth visualization")

    revision = getattr(model.config, "_commit_hash", None)
    report = {
        "schema": "happy_baby_r1.metric_monocular_depth",
        "schema_version": 1,
        "status": "exploratory_unvalidated",
        "metric_claim": (
            "Model output is expressed in metres, but accuracy on the R1 wide-angle image "
            "has not been validated against a physical reference."
        ),
        "input": {
            "path": str(args.image.resolve()),
            "sha256": hashlib.sha256(image_bytes).hexdigest(),
            "shape_hw": [height, width],
            "lens_undistorted": False,
        },
        "model": {
            "id": args.model,
            "revision": revision,
            "model_type": getattr(model.config, "model_type", None),
            "max_depth_m": getattr(model.config, "max_depth", None),
        },
        "runtime": {
            "device": device,
            "load_s": round(load_s, 6),
            "inference_s": round(inference_s, 6),
            "python": platform.python_version(),
            "torch": torch.__version__,
            "transformers": transformers.__version__,
            "opencv": cv2.__version__,
        },
        "statistics": statistics,
        "artifacts": {
            "depth_m": "depth_m.npz",
            "visualization": "depth_visualization.png",
        },
        "limitations": [
            "No physical ground-truth distance was present in the captured frame.",
            "The R1 wide-angle lens has not been calibrated or undistorted.",
            "A monocular model can be biased by scene content and domain shift.",
            "Do not use this output as a motion-safety or collision-avoidance signal.",
        ],
    }
    (output_dir / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
