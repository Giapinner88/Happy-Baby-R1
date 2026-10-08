import asyncio
import json
import tempfile
import unittest
from pathlib import Path

from hb_voice.gesture_tools import GestureToolController


class FakeParams:
    def __init__(self, call_id: str):
        self.tool_call_id = call_id
        self.results = []

    async def result_callback(self, result):
        self.results.append(result)


class GestureToolControllerTest(unittest.IsolatedAsyncioTestCase):
    async def test_off_mode_advertises_no_tools(self):
        controller = GestureToolController(
            mode="off", socket_path=Path("/run/hb/gesture_owner.sock"), timeout_s=0.1
        )
        self.assertEqual(controller.tools, [])

    async def test_shadow_never_connects_to_owner(self):
        controller = GestureToolController(
            mode="shadow",
            socket_path=Path("/tmp/missing-gesture-owner.sock"),
            timeout_s=0.05,
        )
        params = FakeParams("call-shadow")

        await controller.perform_gesture(params, "wave")

        self.assertEqual(params.results[0]["status"], "shadow")
        self.assertFalse(params.results[0]["executed"])

    async def test_unknown_action_is_rejected_before_ipc(self):
        controller = GestureToolController(
            mode="execute",
            socket_path=Path("/tmp/missing-gesture-owner.sock"),
            timeout_s=0.05,
        )
        params = FakeParams("call-unknown")

        await controller.perform_gesture(params, "dance")

        self.assertEqual(params.results[0]["reason"], "unknown_gesture")

    async def test_duplicate_tool_call_returns_cached_result(self):
        controller = GestureToolController(
            mode="shadow",
            socket_path=Path("/run/hb/gesture_owner.sock"),
            timeout_s=0.1,
        )
        first = FakeParams("call-duplicate")
        duplicate = FakeParams("call-duplicate")

        await controller.perform_gesture(first, "heart")
        await controller.perform_gesture(duplicate, "wave")

        self.assertEqual(first.results, duplicate.results)
        self.assertEqual(duplicate.results[0]["gesture_id"], "heart")

    async def test_execute_fails_closed_without_owner(self):
        controller = GestureToolController(
            mode="execute",
            socket_path=Path("/tmp/missing-gesture-owner.sock"),
            timeout_s=0.05,
        )
        params = FakeParams("call-unavailable")

        await controller.cancel_gesture(params)

        self.assertEqual(params.results[0]["status"], "unavailable")

    async def test_execute_uses_owner_reply(self):
        with tempfile.TemporaryDirectory(prefix="hb-gesture-test-") as directory:
            socket_path = Path(directory) / "owner.sock"
            received = []

            async def owner(reader, writer):
                request = json.loads(await reader.readline())
                received.append(request)
                writer.write(
                    json.dumps(
                        {
                            "ok": True,
                            "status": "accepted",
                            "gesture_id": request["gesture_id"],
                            "request_id": request["request_id"],
                        }
                    ).encode()
                    + b"\n"
                )
                await writer.drain()
                writer.close()
                await writer.wait_closed()

            server = await asyncio.start_unix_server(owner, path=str(socket_path))
            try:
                controller = GestureToolController(
                    mode="execute", socket_path=socket_path, timeout_s=0.5
                )
                params = FakeParams("call-heart")
                await controller.perform_gesture(params, "heart")
            finally:
                server.close()
                await server.wait_closed()

            self.assertEqual(params.results[0]["status"], "accepted")
            self.assertEqual(received[0]["gesture_id"], "heart")
            self.assertEqual(received[0]["command"], "play")

    async def test_dev_exit_hides_tools_and_cancels_active_voice_gesture(self):
        with tempfile.TemporaryDirectory(prefix="hb-gesture-test-") as directory:
            socket_path = Path(directory) / "owner.sock"
            received = []

            async def owner(reader, writer):
                request = json.loads(await reader.readline())
                received.append(request)
                writer.write(
                    json.dumps(
                        {
                            "ok": True,
                            "status": "accepted",
                            "request_id": request["request_id"],
                        }
                    ).encode()
                    + b"\n"
                )
                await writer.drain()
                writer.close()
                await writer.wait_closed()

            server = await asyncio.start_unix_server(owner, path=str(socket_path))
            try:
                controller = GestureToolController(
                    mode="execute", socket_path=socket_path, timeout_s=0.5
                )
                await controller.perform_gesture(FakeParams("call-wave"), "wave")
                result = await controller.disable_for_dev_exit()
                rejected = await controller._dispatch(
                    "call-after-dev-exit", command="play", gesture_id="heart"
                )
            finally:
                server.close()
                await server.wait_closed()

            self.assertEqual(result["status"], "accepted")
            self.assertEqual([request["command"] for request in received], ["play", "cancel"])
            self.assertEqual(controller.tools, [])
            self.assertEqual(rejected["status"], "disabled")


if __name__ == "__main__":
    unittest.main()
