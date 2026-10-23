import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock

from hb_voice.vision_tools import (
    CameraCapture,
    VISION_TOOL_INSTRUCTIONS,
    VisionToolController,
    _is_complete_jpeg,
)


class FakeParams:
    def __init__(self):
        self.llm = type("FakeLLM", (), {})()
        self.llm.send_client_event = AsyncMock()
        self.result_callback = AsyncMock()


class VisionToolControllerTest(unittest.IsolatedAsyncioTestCase):
    def controller(self) -> VisionToolController:
        return VisionToolController(network_interface="eth10")

    async def test_schema_is_read_only_and_requires_no_arguments(self):
        tool = self.controller().tools[0]
        self.assertEqual(tool.name, "inspect_camera")
        self.assertEqual(tool.properties, {})
        self.assertNotIn("gesture", tool.description.lower())
        self.assertIn("đây là số mấy", tool.description)
        self.assertIn("không dùng lại ảnh", VISION_TOOL_INSTRUCTIONS)
        self.assertIn("không nói câu đệm", VISION_TOOL_INSTRUCTIONS)
        self.assertIn("để mình kiểm tra ảnh", VISION_TOOL_INSTRUCTIONS)
        self.assertIn("không nói rằng họ chưa hề giơ", VISION_TOOL_INSTRUCTIONS)
        self.assertIn("phải bằng tiếng Việt", VISION_TOOL_INSTRUCTIONS)

    async def test_image_is_attached_before_success_result(self):
        controller = self.controller()
        controller._capture_frame = lambda: CameraCapture(
            jpeg=b"\xff\xd8frame\xff\xd9", elapsed_ms=12.5, sha256="abc123"
        )
        params = FakeParams()
        order = []

        async def send(event):
            order.append("image")
            payload = event.model_dump()
            content = payload["item"]["content"][0]
            self.assertEqual(content["type"], "input_image")
            self.assertTrue(content["image_url"].startswith("data:image/jpeg;base64,"))

        async def result(value):
            order.append("result")
            self.assertTrue(value["ok"])
            self.assertEqual(value["response_language"], "vi")

        params.llm.send_client_event.side_effect = send
        params.result_callback.side_effect = result
        await controller.inspect_camera(params)
        self.assertEqual(order, ["image", "result"])

    async def test_prefetch_is_reused_by_next_camera_tool_call(self):
        controller = self.controller()
        captures = 0

        def capture():
            nonlocal captures
            captures += 1
            return CameraCapture(
                jpeg=b"\xff\xd8prefetched\xff\xd9",
                elapsed_ms=8.0,
                sha256="prefetched",
            )

        controller._capture_frame = capture
        controller.prefetch()
        params = FakeParams()
        await controller.inspect_camera(params)

        self.assertEqual(captures, 1)
        self.assertTrue(params.result_callback.await_args.args[0]["ok"])

    async def test_discarded_prefetch_is_not_reused(self):
        controller = self.controller()
        captures = 0

        def capture():
            nonlocal captures
            captures += 1
            return CameraCapture(
                jpeg=b"\xff\xd8frame\xff\xd9",
                elapsed_ms=8.0,
                sha256=str(captures),
            )

        controller._capture_frame = capture
        controller.prefetch()
        await controller._prefetch_task
        controller.discard_prefetch()
        await controller.inspect_camera(FakeParams())

        self.assertEqual(captures, 2)

    async def test_late_transcript_does_not_prefetch_again_after_tool_call(self):
        controller = self.controller()
        controller._capture_frame = lambda: CameraCapture(
            jpeg=b"\xff\xd8frame\xff\xd9", elapsed_ms=8.0, sha256="one"
        )
        await controller.inspect_camera(FakeParams())

        self.assertFalse(controller.prefetch())
        self.assertIsNone(controller._prefetch_task)
        controller.discard_prefetch()
        self.assertTrue(controller.prefetch())
        await controller._prefetch_task

    async def test_capture_failure_returns_safe_result_without_image(self):
        controller = self.controller()

        def fail():
            raise RuntimeError("camera offline")

        controller._capture_frame = fail
        params = FakeParams()
        await controller.inspect_camera(params)
        params.llm.send_client_event.assert_not_awaited()
        self.assertFalse(params.result_callback.await_args.args[0]["ok"])

    def test_rejects_incomplete_jpeg(self):
        self.assertFalse(_is_complete_jpeg(b"not-a-jpeg"))
        self.assertTrue(_is_complete_jpeg(b"\xff\xd8ok\xff\xd9"))
        self.assertTrue(_is_complete_jpeg(b"\xff\xd8ok\xff\xd9\x00\x00"))

    def test_invalid_capture_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            VisionToolController(network_interface="")
        with self.assertRaises(ValueError):
            VisionToolController(network_interface="eth10", camera_attempts=0)
        with self.assertRaises(ValueError):
            VisionToolController(network_interface="eth10", prefetch_ttl_s=0)
        with tempfile.TemporaryDirectory() as directory:
            missing = Path(directory) / "missing.py"
            controller = VisionToolController(
                network_interface="eth10", camera_probe=missing
            )
            with self.assertRaisesRegex(RuntimeError, "camera probe not found"):
                controller._capture_frame()


if __name__ == "__main__":
    unittest.main()
