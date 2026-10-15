import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock

from hb_voice.vision_tools import (
    CameraCapture,
    VISION_TOOL_INSTRUCTIONS,
    VisionToolController,
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

        params.llm.send_client_event.side_effect = send
        params.result_callback.side_effect = result
        await controller.inspect_camera(params)
        self.assertEqual(order, ["image", "result"])

    async def test_capture_failure_returns_safe_result_without_image(self):
        controller = self.controller()

        def fail():
            raise RuntimeError("camera offline")

        controller._capture_frame = fail
        params = FakeParams()
        await controller.inspect_camera(params)
        params.llm.send_client_event.assert_not_awaited()
        self.assertFalse(params.result_callback.await_args.args[0]["ok"])

    async def test_rejects_incomplete_jpeg(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            probe = root / "probe.py"
            python = root / "python3"
            probe.write_text("placeholder")
            python.write_text("placeholder")
            controller = VisionToolController(
                network_interface="eth10", camera_probe=probe, python_executable=python
            )
            # This check is exercised after the subprocess in production; the
            # lightweight unit tests cover the validation helper directly.
            from hb_voice.vision_tools import _is_complete_jpeg

            self.assertFalse(_is_complete_jpeg(b"not-a-jpeg"))
            self.assertTrue(_is_complete_jpeg(b"\xff\xd8ok\xff\xd9"))


if __name__ == "__main__":
    unittest.main()
