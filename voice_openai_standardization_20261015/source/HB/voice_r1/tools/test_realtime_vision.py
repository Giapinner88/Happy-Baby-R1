#!/usr/bin/env python3
"""Capture one read-only R1 camera frame and ask OpenAI Realtime about it.

This diagnostic never advertises function tools and never sends robot commands.
It preserves one JPEG plus machine-readable timing and response evidence.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_QUESTION = (
    "Quan sát ảnh này và trả lời ngắn bằng tiếng Việt. Hãy nêu: "
    "(1) có người thật trong ảnh hay không và số lượng ước tính; "
    "(2) các vật thể nổi bật; (3) biểu hiện nhìn thấy được như cười, "
    "mặt trung tính hoặc không rõ. Không suy đoán cảm xúc nội tâm, danh tính, "
    "tuổi hay khoảng cách mét. Nếu ảnh không đủ rõ, hãy nói không chắc chắn."
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture-videohub", action="store_true")
    source.add_argument("--image", type=Path)
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--camera-timeout-s", type=float, default=0.5)
    parser.add_argument("--camera-attempts", type=int, default=3)
    parser.add_argument(
        "--unitree-sdk-root",
        type=Path,
        default=Path("/home/unitree/Foundation_Unitree/unitree_sdk2_python"),
    )
    parser.add_argument("--model", default="gpt-realtime-2.1")
    parser.add_argument("--question", default=DEFAULT_QUESTION)
    parser.add_argument("--api-timeout-s", type=float, default=30.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def is_complete_jpeg(payload: bytes) -> bool:
    return payload.startswith(b"\xff\xd8") and payload.rstrip(b"\x00").endswith(
        b"\xff\xd9"
    )


def jpeg_dimensions(payload: bytes) -> tuple[int, int] | None:
    """Return width and height from a JPEG SOF marker without decoding it."""
    if not payload.startswith(b"\xff\xd8"):
        return None
    sof = {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
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
        if marker in {0x01, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(payload):
            break
        length = int.from_bytes(payload[offset : offset + 2], "big")
        if length < 2 or offset + length > len(payload):
            break
        if marker in sof and length >= 7:
            height = int.from_bytes(payload[offset + 3 : offset + 5], "big")
            width = int.from_bytes(payload[offset + 5 : offset + 7], "big")
            return width, height
        offset += length
    return None


def build_image_item(jpeg: bytes, question: str) -> dict[str, Any]:
    encoded = base64.b64encode(jpeg).decode("ascii")
    return {
        "type": "conversation.item.create",
        "item": {
            "type": "message",
            "role": "user",
            "content": [
                {"type": "input_text", "text": question},
                {
                    "type": "input_image",
                    "image_url": f"data:image/jpeg;base64,{encoded}",
                },
            ],
        },
    }


def extract_done_text(response: dict[str, Any]) -> str:
    parts: list[str] = []
    for item in response.get("output", []):
        if not isinstance(item, dict) or item.get("type") != "message":
            continue
        for content in item.get("content", []):
            if not isinstance(content, dict):
                continue
            value = content.get("text") or content.get("transcript")
            if isinstance(value, str):
                parts.append(value)
    return "".join(parts)


def capture_videohub(args: argparse.Namespace) -> tuple[bytes, dict[str, Any]]:
    sdk_root = args.unitree_sdk_root.resolve()
    if not sdk_root.is_dir():
        raise RuntimeError(f"Unitree Python SDK not found: {sdk_root}")
    sys.path.insert(0, str(sdk_root))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient

    ChannelFactoryInitialize(args.domain, args.interface)
    client = VideoClient()
    client.SetTimeout(args.camera_timeout_s)
    client.Init()
    attempts: list[dict[str, Any]] = []
    for index in range(args.camera_attempts):
        started = time.monotonic()
        code, data = client.GetImageSample()
        elapsed_ms = (time.monotonic() - started) * 1000.0
        payload = bytes(data) if data is not None else b""
        valid = int(code) == 0 and is_complete_jpeg(payload)
        attempts.append(
            {
                "attempt": index + 1,
                "code": int(code),
                "latency_ms": round(elapsed_ms, 3),
                "bytes": len(payload),
                "complete_jpeg": valid,
            }
        )
        if valid:
            return payload, {"source": "videohub:GetImageSample", "attempts": attempts}
    raise RuntimeError(f"videohub returned no complete JPEG: {attempts}")


async def query_realtime(
    jpeg: bytes,
    *,
    model: str,
    question: str,
    timeout_s: float,
) -> tuple[str, dict[str, Any]]:
    from openai import AsyncOpenAI

    if not os.getenv("OPENAI_API_KEY", "").strip():
        raise RuntimeError("OPENAI_API_KEY is not set")
    client = AsyncOpenAI()
    timeline: list[dict[str, Any]] = []
    text_parts: list[str] = []
    started = time.monotonic()
    first_delta_ms: float | None = None
    request_sent_at: float | None = None
    response_done_ms: float | None = None
    response_record: dict[str, Any] = {}

    async with client.realtime.connect(
        model=model,
        websocket_connection_options={"open_timeout": timeout_s},
    ) as connection:
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for session.created")
            raw = await asyncio.wait_for(connection.recv_bytes(), timeout=remaining)
            event = json.loads(raw)
            event_type = event.get("type", "unknown")
            timeline.append(
                {"type": event_type, "elapsed_ms": round((time.monotonic() - started) * 1000, 3)}
            )
            if event_type == "error":
                raise RuntimeError(f"Realtime session error: {event.get('error')}")
            if event_type == "session.created":
                break

        request_sent_at = time.monotonic()
        await connection.send_raw(json.dumps(build_image_item(jpeg, question)))
        await connection.send_raw(
            json.dumps(
                {
                    "type": "response.create",
                    "response": {
                        "output_modalities": ["text"],
                        "instructions": (
                            "Chỉ mô tả điều nhìn thấy trong ảnh. Không gọi công cụ, "
                            "không suy đoán danh tính, nội tâm hay khoảng cách."
                        ),
                    },
                }
            )
        )

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for response.done")
            raw = await asyncio.wait_for(connection.recv_bytes(), timeout=remaining)
            event = json.loads(raw)
            event_type = event.get("type", "unknown")
            elapsed_ms = (time.monotonic() - started) * 1000.0
            timeline.append({"type": event_type, "elapsed_ms": round(elapsed_ms, 3)})
            if event_type == "error":
                raise RuntimeError(f"Realtime response error: {event.get('error')}")
            if event_type == "response.output_text.delta":
                delta = event.get("delta")
                if isinstance(delta, str):
                    if first_delta_ms is None:
                        first_delta_ms = elapsed_ms
                    text_parts.append(delta)
            if event_type == "response.done":
                response_done_ms = elapsed_ms
                response_record = event.get("response", {})
                if response_record.get("status") != "completed":
                    raise RuntimeError(
                        "Realtime response did not complete: "
                        f"{response_record.get('status_details') or response_record.get('status')}"
                    )
                break

    text = "".join(text_parts).strip() or extract_done_text(response_record).strip()
    if not text:
        raise RuntimeError("Realtime completed without text output")
    assert request_sent_at is not None and response_done_ms is not None
    request_sent_ms = (request_sent_at - started) * 1000.0
    return text, {
        "first_text_delta_ms": round(first_delta_ms, 3) if first_delta_ms is not None else None,
        "response_done_ms": round(response_done_ms, 3),
        "request_to_first_text_delta_ms": (
            round(first_delta_ms - request_sent_ms, 3)
            if first_delta_ms is not None
            else None
        ),
        "request_to_response_done_ms": round(response_done_ms - request_sent_ms, 3),
        "usage": response_record.get("usage"),
        "timeline": timeline,
    }


def write_evidence(
    output_dir: Path,
    *,
    jpeg: bytes,
    args: argparse.Namespace,
    capture: dict[str, Any],
    answer: str,
    api: dict[str, Any],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=False)
    image_path = output_dir / "frame.jpg"
    image_path.write_bytes(jpeg)
    dimensions = jpeg_dimensions(jpeg)
    record = {
        "schema": "happy_baby_r1.realtime_vision_probe",
        "schema_version": 1,
        "run_utc": datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"),
        "safety_scope": "read-only camera RPC; text-only Realtime response; no tools advertised",
        "model": args.model,
        "question": args.question,
        "image": {
            "path": image_path.name,
            "bytes": len(jpeg),
            "sha256": hashlib.sha256(jpeg).hexdigest(),
            "dimensions_wh": list(dimensions) if dimensions else None,
        },
        "capture": capture,
        "answer": answer,
        "api": api,
    }
    (output_dir / "result.json").write_text(
        json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "command.txt").write_text(
        " ".join(
            [
                "python tools/test_realtime_vision.py",
                "--capture-videohub" if args.capture_videohub else f"--image {args.image}",
                f"--interface {args.interface}",
                f"--model {args.model}",
                f"--output-dir {output_dir}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


async def async_main() -> int:
    args = parse_args()
    if args.domain < 0 or args.camera_timeout_s <= 0 or args.camera_attempts <= 0:
        raise ValueError("domain must be >= 0; camera timeout and attempts must be positive")
    if args.api_timeout_s <= 0:
        raise ValueError("api timeout must be positive")

    if args.capture_videohub:
        jpeg, capture = await asyncio.to_thread(capture_videohub, args)
    else:
        assert args.image is not None
        jpeg = args.image.read_bytes()
        if not is_complete_jpeg(jpeg):
            raise ValueError(f"input is not a complete JPEG: {args.image}")
        capture = {"source": "file", "path": str(args.image.resolve())}

    answer, api = await query_realtime(
        jpeg,
        model=args.model,
        question=args.question,
        timeout_s=args.api_timeout_s,
    )
    write_evidence(
        args.output_dir,
        jpeg=jpeg,
        args=args,
        capture=capture,
        answer=answer,
        api=api,
    )
    print(answer)
    print(f"evidence={args.output_dir / 'result.json'}")
    return 0


def main() -> int:
    return asyncio.run(async_main())


if __name__ == "__main__":
    raise SystemExit(main())
