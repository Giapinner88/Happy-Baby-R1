#!/usr/bin/env python3
"""Record JPEG frames from a TeleImager-compatible ZMQ camera publisher."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit


def validate_endpoint(endpoint: str) -> str:
    parsed = urlsplit(endpoint)
    if parsed.scheme != "tcp" or not parsed.hostname or parsed.port is None:
        raise ValueError("camera ZMQ endpoint must have form tcp://host:port")
    if parsed.path not in {"", "/"} or parsed.query or parsed.fragment:
        raise ValueError("camera ZMQ endpoint must not contain path/query/fragment")
    return endpoint


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--stop-file", type=Path)
    args = parser.parse_args()
    try:
        endpoint = validate_endpoint(args.endpoint)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    if args.duration_s <= 0.0 or args.fps <= 0.0:
        raise SystemExit("--duration-s and --fps must be positive")
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"refusing to overwrite camera recording: {output_dir}")
    output_dir.mkdir(parents=True)

    import cv2
    import numpy as np
    import zmq

    stop = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.connect(endpoint)
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    started = time.monotonic()
    deadline = started + args.duration_s
    writer = None
    frame_records: list[dict[str, object]] = []
    decode_failure_count = 0
    stop_reason = "duration_elapsed"
    try:
        while time.monotonic() < deadline and not stop.is_set():
            if args.stop_file is not None and args.stop_file.expanduser().exists():
                stop_reason = "stop_file_requested"
                break
            events = dict(poller.poll(timeout=100))
            if socket not in events:
                continue
            received_at = time.monotonic()
            payload = socket.recv()
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                decode_failure_count += 1
                continue
            height, width = frame.shape[:2]
            if writer is None:
                writer = cv2.VideoWriter(
                    str(output_dir / "robot_camera.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    args.fps,
                    (width, height),
                )
                if not writer.isOpened():
                    raise RuntimeError("OpenCV could not open robot_camera.mp4 writer")
            writer.write(frame)
            frame_records.append(
                {
                    "frame_index": len(frame_records),
                    "received_monotonic_s": received_at,
                    "received_elapsed_s": received_at - started,
                    "jpeg_bytes": len(payload),
                    "width": width,
                    "height": height,
                }
            )
    finally:
        if writer is not None:
            writer.release()
        socket.close()
        context.term()

    (output_dir / "frames.jsonl").write_text(
        "".join(json.dumps(record, separators=(",", ":")) + "\n" for record in frame_records),
        encoding="utf-8",
    )
    status = {
        "status": "completed" if frame_records else "failed",
        "stop_reason": stop_reason,
        "endpoint": endpoint,
        "requested_fps": args.fps,
        "frame_count": len(frame_records),
        "decode_failure_count": decode_failure_count,
        "timestamp_semantics": "workstation_monotonic_receive_time",
    }
    (output_dir / "status.json").write_text(
        json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(status, sort_keys=True), file=sys.stderr, flush=True)
    return 0 if frame_records else 2


if __name__ == "__main__":
    raise SystemExit(main())
