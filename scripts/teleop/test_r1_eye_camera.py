#!/usr/bin/env python3
"""Read-only probe for the Unitree R1 EDU head camera via videohub RPC.

Run this script on the robot computer, where ``eth10`` reaches the R1 main
computer that serves ``videohub``. The camera is not exposed to the Orin NX as
``/dev/video*`` or an NVIDIA Argus sensor, and no fixed TCP camera port is
required.

Example::

    python3 test_r1_eye_camera.py --interface eth10 \
      --output-dir /tmp/r1_eye_camera_test

The probe only requests JPEG samples. It creates no DDS publisher and sends no
motor or configuration command.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


def jpeg_dimensions(payload: bytes) -> Optional[tuple[int, int]]:
    """Return ``(width, height)`` from a JPEG SOF marker without decoding."""

    if not payload.startswith(b"\xff\xd8"):
        return None
    sof_markers = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    offset = 2
    while offset + 4 <= len(payload):
        if payload[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(payload) and payload[offset] == 0xFF:
            offset += 1
        if offset >= len(payload):
            break
        marker = payload[offset]
        offset += 1
        if marker in (0x01, 0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(payload):
            break
        length = int.from_bytes(payload[offset : offset + 2], "big")
        if length < 2 or offset + length > len(payload):
            break
        if marker in sof_markers and length >= 7:
            height = int.from_bytes(payload[offset + 3 : offset + 5], "big")
            width = int.from_bytes(payload[offset + 5 : offset + 7], "big")
            return width, height
        offset += length
    return None


def is_complete_jpeg(payload: bytes) -> bool:
    """Accept a JPEG only when both SOI and EOI markers are present."""

    return payload.startswith(b"\xff\xd8") and payload.rstrip(b"\x00").endswith(b"\xff\xd9")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--timeout-s", type=float, default=0.1)
    parser.add_argument("--requests", type=int, default=10)
    parser.add_argument(
        "--interval-s",
        type=float,
        default=0.1,
        help="Pause between requests so successive files can contain fresh camera frames.",
    )
    return parser


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.interface.strip():
        parser.error("--interface must not be empty")
    if args.domain < 0:
        parser.error("--domain must be non-negative")
    if args.timeout_s <= 0:
        parser.error("--timeout-s must be positive")
    if args.requests <= 0:
        parser.error("--requests must be positive")
    if args.interval_s < 0:
        parser.error("--interval-s must be non-negative")
    return args


def _write_report(output_dir: Path, report: dict[str, object]) -> None:
    (output_dir / "report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        print(f"Refusing to overwrite existing output directory: {output_dir}", file=sys.stderr)
        return 2
    output_dir.mkdir(parents=True)

    report: dict[str, object] = {
        "schema": "happy_baby_r1.eye_camera_probe",
        "schema_version": 2,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source": "unitree_videohub_rpc",
        "dds": {"domain": args.domain, "interface": args.interface},
        "rpc_timeout_s": args.timeout_s,
        "requested_samples": args.requests,
        "request_interval_s": args.interval_s,
        "writes_motor_commands": False,
        "frames": [],
        "failures_by_code": {},
    }

    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.video.video_client import VideoClient

        ChannelFactoryInitialize(args.domain, args.interface)
        client = VideoClient()
        client.SetTimeout(args.timeout_s)
        client.Init()
    except Exception as exc:  # noqa: BLE001 - diagnostic must preserve the failure
        report["status"] = "initialization_failed"
        report["successful_frames"] = 0
        report["error"] = f"{type(exc).__name__}: {exc}"
        _write_report(output_dir, report)
        print(f"[FAIL] videohub initialization: {exc}", file=sys.stderr)
        return 1

    frames: list[dict[str, object]] = []
    failures: dict[str, int] = {}
    for request_index in range(args.requests):
        started = time.monotonic()
        try:
            code, data = client.GetImageSample()
            error = None
        except Exception as exc:  # noqa: BLE001 - keep probing and record diagnostics
            code, data = -1, b""
            error = f"{type(exc).__name__}: {exc}"
        latency_ms = (time.monotonic() - started) * 1000.0
        payload = bytes(data) if data is not None else b""

        if int(code) == 0 and is_complete_jpeg(payload):
            dimensions = jpeg_dimensions(payload)
            filename = f"frame_{len(frames):04d}.jpg"
            (output_dir / filename).write_bytes(payload)
            record: dict[str, object] = {
                "request_index": request_index,
                "file": filename,
                "bytes": len(payload),
                "latency_ms": round(latency_ms, 3),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "dimensions": (
                    {"width": dimensions[0], "height": dimensions[1]}
                    if dimensions is not None
                    else None
                ),
            }
            frames.append(record)
            dimensions_text = (
                f"{dimensions[0]}x{dimensions[1]}" if dimensions is not None else "unknown"
            )
            print(
                f"[OK] request={request_index + 1} code=0 latency={latency_ms:.1f}ms "
                f"bytes={len(payload)} dimensions={dimensions_text} file={filename}",
                flush=True,
            )
        else:
            failure_key = str(int(code)) if int(code) != 0 else "truncated_or_non_jpeg"
            failures[failure_key] = failures.get(failure_key, 0) + 1
            suffix = f" error={error}" if error else ""
            print(
                f"[FAIL] request={request_index + 1} code={code} "
                f"latency={latency_ms:.1f}ms bytes={len(payload)}{suffix}",
                file=sys.stderr,
                flush=True,
            )
        if request_index + 1 < args.requests and args.interval_s:
            time.sleep(args.interval_s)

    report["frames"] = frames
    report["successful_frames"] = len(frames)
    report["failures_by_code"] = failures
    report["status"] = "completed" if frames else "failed"
    _write_report(output_dir, report)
    print(
        f"[RESULT] {len(frames)}/{args.requests} valid JPEG frame(s); report={output_dir / 'report.json'}",
        flush=True,
    )
    return 0 if frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
