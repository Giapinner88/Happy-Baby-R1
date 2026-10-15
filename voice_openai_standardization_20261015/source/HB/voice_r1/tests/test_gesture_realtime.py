"""Exercise the installed Pipecat adapter, handlers and response frame ordering.

Offline only: fake credentials, captured transport frames, local Unix socket.
These checks do not claim to validate the model's Vietnamese/audio recognition.
"""

import asyncio
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock

from pipecat.frames.frames import LLMFullResponseEndFrame, LLMTextFrame
from pipecat.processors.aggregators.llm_context import LLMContext
from pipecat.services.llm_service import FunctionCallParams
from pipecat.services.openai.realtime import events
from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService

from hb_voice.app import ServerTurnRealtimeLLMService
from hb_voice.gesture_tools import GestureToolController, SUPPORTED_GESTURES


class GestureRealtimeTest(unittest.IsolatedAsyncioTestCase):
    def service(self, controller=None, *, guard=True):
        tools = controller.tools if controller else []
        service = ServerTurnRealtimeLLMService(
            api_key="offline-test-only",
            guard_gesture_text=guard,
            settings=OpenAIRealtimeLLMService.Settings(
                model="gpt-realtime-2.1",
                system_instruction="offline test",
                session_properties=events.SessionProperties(
                    output_modalities=["text"], tools=tools,
                ),
            ),
        )
        service.push_frame = AsyncMock()
        service.send_client_event = AsyncMock()
        return service

    def delta(self, response_id, text):
        return events.ResponseTextDelta(
            type="response.output_text.delta", event_id="event-delta",
            response_id=response_id, item_id="item-text", output_index=0,
            content_index=0, delta=text,
        )

    def done(self, response_id, *, tool=False, status="completed"):
        return events.ResponseDone(
            type="response.done", event_id="event-done",
            response=events.Response(
                id=response_id, object="realtime.response", status=status,
                status_details=None,
                output=[events.ConversationItem(
                    id="item-tool", type="function_call", name="perform_gesture",
                    call_id="call-wave", arguments='{"gesture_id":"wave"}',
                )] if tool else [],
                usage=events.Usage(
                    total_tokens=2, input_tokens=1, output_tokens=1,
                    input_token_details=events.TokenDetails(text_tokens=1),
                    output_token_details=events.TokenDetails(text_tokens=1),
                ),
            ),
        )

    def spoken_text(self, service):
        return "".join(
            call.args[0].text for call in service.push_frame.await_args_list
            if isinstance(call.args[0], LLMTextFrame)
        )

    async def test_schema_reaches_session_and_handlers_survive_context_sync(self):
        controller = GestureToolController(
            mode="shadow", socket_path=Path("/unused"), timeout_s=0.1,
        )
        service = self.service(controller)
        for context in (None, LLMContext(), LLMContext(tools=controller.tools)):
            service._context = context
            service._sync_registered_tool_handlers(context.tools if context else None)
            await service._send_session_update()
            payload = service.send_client_event.await_args.args[0].model_dump()
            advertised = {tool["name"]: tool for tool in payload["session"]["tools"]}
            self.assertEqual(set(advertised), {
                "perform_gesture", "cancel_gesture", "get_gesture_status",
            })
            enum = advertised["perform_gesture"]["parameters"]["properties"]["gesture_id"]["enum"]
            self.assertEqual(set(enum), SUPPORTED_GESTURES)
            self.assertTrue(all(name in service._functions for name in advertised))

    async def test_dev_exit_removes_remote_tools_and_local_handlers(self):
        controller = GestureToolController(
            mode="off", socket_path=Path("/unused"), timeout_s=0.1,
        )
        service = self.service()
        context = LLMContext(tools=[])
        service._context = context

        controller.set_mode("execute")
        tools = controller.tools
        context.set_tools(tools)
        service._register_advertised_tool_handlers(tools)
        service._settings.session_properties = events.SessionProperties(
            output_modalities=["text"], tools=tools
        )
        await service._send_session_update()
        self.assertEqual(
            [tool["name"] for tool in service.send_client_event.await_args.args[0].model_dump()["session"]["tools"]],
            ["perform_gesture", "cancel_gesture", "get_gesture_status"],
        )

        controller.set_mode("off")
        context.set_tools([])
        service._settings.session_properties = events.SessionProperties(
            output_modalities=["text"], tools=[]
        )
        service._sync_registered_tool_handlers([])
        await service._send_session_update()
        session = service.send_client_event.await_args.args[0].model_dump()["session"]
        self.assertIsNone(session["tools"])
        self.assertIsNone(session["tool_choice"])
        self.assertFalse(service._functions)

    async def test_registered_handlers_dispatch_all_six_actions_to_owner(self):
        with tempfile.TemporaryDirectory(prefix="hb-voice-runtime-") as directory:
            path = Path(directory) / "owner.sock"
            received = []

            async def owner(reader, writer):
                request = json.loads(await reader.readline())
                received.append(request)
                writer.write(json.dumps({
                    "ok": True, "status": "accepted",
                    "request_id": request["request_id"],
                    "gesture_id": request["gesture_id"],
                }).encode() + b"\n")
                await writer.drain()
                writer.close()
                await writer.wait_closed()

            async with await asyncio.start_unix_server(owner, path=str(path)):
                controller = GestureToolController(mode="execute", socket_path=path, timeout_s=1)
                service = self.service(controller)
                context = LLMContext(tools=controller.tools)
                service._sync_registered_tool_handlers(context.tools)
                for action in sorted(SUPPORTED_GESTURES):
                    callback = AsyncMock()
                    params = FunctionCallParams(
                        function_name="perform_gesture", tool_call_id=f"call-{action}",
                        arguments={"gesture_id": action}, llm=service,
                        pipeline_worker=None, context=context, result_callback=callback,
                    )
                    await service._functions["perform_gesture"].handler(params)
                    self.assertEqual(callback.await_args.args[0]["status"], "accepted")
            self.assertEqual({r["gesture_id"] for r in received}, SUPPORTED_GESTURES)
            self.assertTrue(all(r["command"] == "play" for r in received))

    async def test_preamble_is_silent_but_post_tool_reply_is_spoken(self):
        service = self.service()
        await service._handle_evt_text_delta(self.delta("r1", "Đang thực hiện lệnh vẫy tay."))
        self.assertEqual(self.spoken_text(service), "")
        await service._handle_evt_response_done(self.done("r1", tool=True))
        self.assertEqual(self.spoken_text(service), "")
        await service._handle_evt_text_delta(self.delta("r2", "Chào cả nhà nha!"))
        await service._handle_evt_response_done(self.done("r2"))
        self.assertEqual(self.spoken_text(service), "Chào cả nhà nha!")
        self.assertIsInstance(service.push_frame.await_args.args[0], LLMFullResponseEndFrame)

    async def test_regular_chat_preserves_text_order(self):
        service = self.service()
        for chunk in ("Chào ", "bạn", " nha!"):
            await service._handle_evt_text_delta(self.delta("chat", chunk))
        await service._handle_evt_response_done(self.done("chat"))
        self.assertEqual(self.spoken_text(service), "Chào bạn nha!")

    async def test_cancelled_response_does_not_speak_buffered_text(self):
        service = self.service()
        await service._handle_evt_text_delta(self.delta("cancelled", "Không được đọc."))
        await service._handle_evt_response_done(self.done("cancelled", status="cancelled"))
        self.assertEqual(self.spoken_text(service), "")
        self.assertFalse(service._gesture_response_text)

    async def test_disabled_guard_keeps_streaming(self):
        service = self.service(guard=False)
        await service._handle_evt_text_delta(self.delta("chat", "Chào bạn!"))
        self.assertEqual(self.spoken_text(service), "Chào bạn!")

    async def test_turn_buffered_before_dev_exit_is_not_spoken_after_exit(self):
        service = self.service(guard=True)
        await service._handle_evt_text_delta(self.delta("old", "Không được đọc."))
        service.set_gesture_text_guard(False)
        await service._handle_evt_response_done(self.done("old"))
        self.assertEqual(self.spoken_text(service), "")
        await service._handle_evt_text_delta(self.delta("new", "Vẫn trò chuyện bình thường."))
        self.assertEqual(self.spoken_text(service), "Vẫn trò chuyện bình thường.")

    async def test_interruption_discards_late_chunks_of_the_old_response(self):
        service = self.service()
        await service._handle_evt_text_delta(self.delta("old", "Câu cũ"))
        await service._handle_interruption()
        await service._handle_evt_text_delta(self.delta("old", " vẫn đang về."))
        await service._handle_evt_response_done(self.done("old"))
        self.assertEqual(self.spoken_text(service), "")
        await service._handle_evt_text_delta(self.delta("new", "Câu mới."))
        await service._handle_evt_response_done(self.done("new"))
        self.assertEqual(self.spoken_text(service), "Câu mới.")

    async def test_server_session_ack_accepts_the_serialized_tools(self):
        controller = GestureToolController(
            mode="shadow", socket_path=Path("/unused"), timeout_s=0.1,
        )
        service = self.service(controller)
        await service._send_session_update()
        payload = service.send_client_event.await_args.args[0].model_dump()
        await service._handle_evt_session_updated(events.SessionUpdatedEvent(
            type="session.updated", event_id="session-ack",
            session=events.SessionProperties(**payload["session"]),
        ))
        self.assertTrue(service._api_session_ready)


if __name__ == "__main__":
    unittest.main()
