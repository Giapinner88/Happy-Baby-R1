"""On-demand, read-only R1 camera tool for an OpenAI Realtime conversation."""

from __future__ import annotations

import asyncio
import base64
import hashlib
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from loguru import logger
from pipecat.services.openai.realtime import events


VISION_TOOL_INSTRUCTIONS = """
# Công cụ quan sát camera
- MỖI lượt người dùng hỏi về cảnh hiện tại, hãy gọi inspect_camera để chụp ảnh
  mới trước khi trả lời; không dùng lại ảnh của lượt trước. Quy tắc này gồm các
  câu như “đây là gì/số mấy/màu gì”, robot đang thấy gì, có người/vật gì trước
  mặt, có bao nhiêu người, hoặc người dùng đang cầm vật gì.
- Không gọi inspect_camera cho câu hỏi kiến thức chung hoặc khi người dùng không
  hỏi về cảnh hiện tại trước camera.
- Phản hồi đầu tiên của lượt quan sát phải là đúng một tool call inspect_camera,
  không nói câu đệm như “để mình nhìn” trước tool. Sau khi tool trả ok=true, hãy
  quan sát ảnh vừa được thêm vào cuộc hội thoại và trả lời trực tiếp, tự nhiên,
  tối đa 2 câu. Nếu ảnh mờ, khuất hoặc không đủ rõ, hãy nói rõ mức độ không chắc chắn.
- Chỉ mô tả biểu hiện nhìn thấy được như đang cười, mặt trung tính hoặc không rõ;
  không khẳng định cảm xúc nội tâm, danh tính, tuổi, tình trạng sức khỏe hay ý định.
- Không ước lượng khoảng cách theo mét và không dùng ảnh này để quyết định an toàn
  chuyển động. Không nhắc tên tool, API, base64 hoặc cơ chế nội bộ với người dùng.
""".strip()


def _is_complete_jpeg(payload: bytes) -> bool:
    return payload.startswith(b"\xff\xd8") and payload.rstrip(b"\x00").endswith(
        b"\xff\xd9"
    )


@dataclass(frozen=True)
class CameraCapture:
    jpeg: bytes
    elapsed_ms: float
    sha256: str
    subprocess_ms: float = 0.0


class VisionToolController:
    """Capture one frame in a system-Python subprocess and append it to Realtime."""

    def __init__(
        self,
        *,
        network_interface: str,
        camera_probe: Path = Path(
            "/home/unitree/Foundation_Unitree/tools/r1_video_rpc_probe.py"
        ),
        python_executable: Path = Path("/usr/bin/python3"),
        camera_timeout_s: float = 0.5,
        camera_attempts: int = 3,
        process_timeout_s: float = 5.0,
        max_jpeg_bytes: int = 5_000_000,
        latency_tracker=None,
    ) -> None:
        if not network_interface.strip():
            raise ValueError("network_interface must not be empty")
        if camera_timeout_s <= 0 or camera_attempts < 1 or process_timeout_s <= 0:
            raise ValueError("camera timeouts and attempts must be positive")
        if max_jpeg_bytes < 4:
            raise ValueError("max_jpeg_bytes is too small")
        self.network_interface = network_interface
        self.camera_probe = camera_probe
        self.python_executable = python_executable
        self.camera_timeout_s = camera_timeout_s
        self.camera_attempts = camera_attempts
        self.process_timeout_s = process_timeout_s
        self.max_jpeg_bytes = max_jpeg_bytes
        self._capture_lock = asyncio.Lock()
        self._latency_tracker = latency_tracker

    @property
    def tools(self) -> list:
        from pipecat.adapters.schemas.function_schema import FunctionSchema

        return [
            FunctionSchema(
                name="inspect_camera",
                description=(
                    "Chụp một ảnh mới từ camera đầu của robot để trả lời câu hỏi "
                    "về người, vật thể hoặc biểu hiện đang nhìn thấy ở thời điểm hiện tại. "
                    "Bao gồm câu chỉ định như 'đây là gì', 'đây là số mấy' hoặc "
                    "'màu gì'. Chỉ đọc camera; không điều khiển chuyển động. "
                    "Mỗi lượt hỏi về cảnh hiện tại phải gọi lại tool ngay và không nói câu đệm."
                ),
                properties={},
                required=[],
                handler=self.inspect_camera,
            )
        ]

    async def inspect_camera(self, params) -> None:
        """Append a fresh JPEG before returning the result that resumes the model."""
        try:
            if self._latency_tracker:
                self._latency_tracker.mark_tool("camera")
            async with self._capture_lock:
                capture = await asyncio.to_thread(self._capture_frame)
            if self._latency_tracker:
                self._latency_tracker.camera_capture(
                    subprocess_ms=capture.subprocess_ms,
                    total_ms=capture.elapsed_ms,
                )
            encoded = base64.b64encode(capture.jpeg).decode("ascii")
            item = events.ConversationItem(
                type="message",
                role="user",
                content=[
                    events.ItemContent(
                        type="input_image",
                        image_url=f"data:image/jpeg;base64,{encoded}",
                        detail="auto",
                    )
                ],
            )
            await params.llm.send_client_event(
                events.ConversationItemCreateEvent(item=item)
            )
            if self._latency_tracker:
                self._latency_tracker.mark("image_attached")
            result: dict[str, Any] = {
                "ok": True,
                "status": "image_attached",
                "captured_now": True,
                "capture_ms": round(capture.elapsed_ms, 3),
                "jpeg_bytes": len(capture.jpeg),
                "sha256": capture.sha256,
                "guidance": (
                    "Quan sát ảnh mới trong cuộc hội thoại rồi trả lời câu hỏi của người dùng."
                ),
            }
            logger.info(
                "Vision tool attached camera frame bytes={} capture_ms={:.1f} sha256={}",
                len(capture.jpeg),
                capture.elapsed_ms,
                capture.sha256[:16],
            )
        except Exception as error:
            logger.error("Vision tool could not capture/attach a frame: {}", error)
            result = {
                "ok": False,
                "status": "camera_unavailable",
                "guidance": (
                    "Nói ngắn gọn rằng hiện tại chưa nhìn được camera và mời người dùng thử lại."
                ),
            }
        await params.result_callback(result)

    def _capture_frame(self) -> CameraCapture:
        if not self.camera_probe.is_file():
            raise RuntimeError(f"camera probe not found: {self.camera_probe}")
        if not self.python_executable.is_file():
            raise RuntimeError(f"system Python not found: {self.python_executable}")

        started = time.monotonic()
        with tempfile.TemporaryDirectory(prefix="hb-voice-camera-") as directory:
            output_dir = Path(directory)
            command = [
                str(self.python_executable),
                str(self.camera_probe),
                "--interface",
                self.network_interface,
                "--timeout",
                str(self.camera_timeout_s),
                "--requests",
                str(self.camera_attempts),
                "--save-dir",
                str(output_dir),
            ]
            subprocess_started = time.monotonic()
            completed = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=self.process_timeout_s,
                check=False,
            )
            subprocess_ms = (time.monotonic() - subprocess_started) * 1000.0
            if completed.returncode != 0:
                tail = completed.stdout[-800:].replace("\n", " | ")
                raise RuntimeError(
                    f"camera probe exited {completed.returncode}: {tail}"
                )
            images = sorted(output_dir.glob("*.jpg"))
            if len(images) != 1:
                raise RuntimeError(
                    f"camera probe produced {len(images)} JPEG files; expected one"
                )
            jpeg = images[0].read_bytes()

        if len(jpeg) > self.max_jpeg_bytes:
            raise RuntimeError(
                f"camera JPEG is {len(jpeg)} bytes; limit is {self.max_jpeg_bytes}"
            )
        if not _is_complete_jpeg(jpeg):
            raise RuntimeError("camera probe produced an incomplete JPEG")
        return CameraCapture(
            jpeg=jpeg,
            elapsed_ms=(time.monotonic() - started) * 1000.0,
            sha256=hashlib.sha256(jpeg).hexdigest(),
            subprocess_ms=subprocess_ms,
        )
