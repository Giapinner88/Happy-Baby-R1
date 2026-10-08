"""Bounded voice-tool boundary for high-level-owned R1 gestures.

The Realtime model selects one stable action ID. This module validates and
deduplicates that request, then asks the local high-level owner to execute it.
It never publishes motor commands or accepts asset paths/joint values.
"""

from __future__ import annotations

import asyncio
import json
from collections import OrderedDict
from pathlib import Path
from typing import Any

from loguru import logger


GESTURE_TOOL_INSTRUCTIONS = """
# Tương tác bằng cử chỉ
- Chỉ gọi perform_gesture khi người dùng yêu cầu robot thực hiện một cử chỉ rõ ràng.
- Các lựa chọn: wave (vẫy tay), heart (tạo trái tim), handshake (bắt tay),
  cool_pose (tạo dáng ngầu), determined (thể hiện quyết tâm), give_gift (tặng quà).
- Nhận lời mời tự nhiên, không đòi người dùng nói tên ID hay từ “lệnh”:
  “bắt tay với mình được không?” / “cho mình bắt tay” -> handshake;
  “thả tim đi” / “làm hình trái tim nhé” -> heart;
  “vẫy tay chào mọi người đi” -> wave;
  “khoanh tay tạo dáng ngầu nào” -> cool_pose;
  “giơ nắm tay quyết tâm nào” -> determined; “đưa quà cho bé đi” -> give_gift.
- Câu hỏi lịch sự “... được không?” kèm một động tác cụ thể là một lời mời thực hiện.
  Chỉ hỏi chung “bạn biết làm những động tác gì?” thì giới thiệu các cử chỉ, không chạy.
  Lời nói được trích dẫn, yêu cầu phủ định và trò chuyện bình thường không được chạy cử chỉ.
- “Chào bạn” chỉ là chào hỏi. “Đọ tay”, “giơ tay” mà chưa rõ kiểu động tác thì hỏi
  lại ngắn gọn, không tự gán sang một cử chỉ khác hoặc giả vờ đang làm.
- Nếu chưa rõ người dùng muốn cử chỉ nào, hãy hỏi lại; không tự đoán.
- Mỗi lượt chỉ chạy tối đa một cử chỉ; nếu người dùng yêu cầu nhiều cử chỉ, hãy hỏi chọn một.
- Dùng cancel_gesture khi người dùng nói rõ muốn dừng hoặc hạ tay. Chỉ hủy cử chỉ do lệnh voice khởi chạy.
- Với yêu cầu cử chỉ hợp lệ, phản hồi ĐẦU TIÊN phải là đúng một tool call
  perform_gesture; không kèm chữ, lời chào hay lời xác nhận nào trước/sau tool call đó.
  Chờ tool trả kết quả rồi mới được nói.
- Tuyệt đối không tường thuật cơ chế điều khiển, kể cả trước và sau kết quả tool.
  Không nói “đang thực hiện lệnh”, “đã thực hiện lệnh”, “nhận được lệnh”, “đang xử lý yêu cầu”,
  “robot sẽ gửi lệnh”,
  “em sẽ gửi lệnh”, “đang gửi lệnh”, “robot sẽ thực hiện” hoặc “mình sẽ thực hiện”.
- Kết quả accepted chỉ xác nhận high-level đã bắt đầu lệnh, không xác nhận cử chỉ đã hoàn thành.
  Sau khi được chấp nhận, trả lời ngắn ở thì hiện tại, ví dụ “Dạ, mình vẫy tay chào bạn nè!”;
  không nói accepted, tên hàm, ID, socket, lý do nội bộ hoặc mô tả việc gửi lệnh. Không nói đã
  làm xong cho đến khi trạng thái báo idle.
- Dùng câu giao tiếp gắn với cử chỉ: bắt tay “Rất vui được gặp bạn nha!”, trái tim
  “Tặng bạn một trái tim nè!”, vẫy tay “Chào cả nhà nha!”, tạo dáng “Chụp tấm hình nha!”,
  quyết tâm “Cố lên nào!”, tặng quà “Một món quà dành cho bạn nè!”.
  Chỉ nói những câu này sau kết quả ok=true, status=accepted cho đúng động tác.
  Chưa gọi được tool hoặc tool báo lỗi thì không được nói đang làm hay đã làm động tác.
- Nếu tool báo shadow, chưa thực hiện chuyển động; hãy nói rõ là robot chưa làm cử chỉ.
- Nếu tool từ chối, giải thích tự nhiên bằng tiếng Việt, không đọc mã lỗi nội bộ.
""".strip()

SUPPORTED_GESTURES = frozenset(
    {"wave", "heart", "handshake", "cool_pose", "determined", "give_gift"}
)
SUPPORTED_MODES = frozenset({"off", "shadow", "execute"})


class GestureToolController:
    """Expose validated Pipecat direct functions for voice gesture requests."""

    def __init__(
        self,
        *,
        mode: str,
        socket_path: Path,
        timeout_s: float,
        cache_size: int = 128,
    ) -> None:
        if mode not in SUPPORTED_MODES:
            raise ValueError(f"Unsupported gesture mode: {mode}")
        if timeout_s <= 0.0:
            raise ValueError("Gesture timeout must be positive")
        if cache_size < 1:
            raise ValueError("Gesture result cache must contain at least one entry")
        self.mode = mode
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self.cache_size = cache_size
        self._results: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._voice_owned_active = False
        self._dev_exit_sequence = 0

    def set_mode(self, mode: str) -> None:
        if mode not in SUPPORTED_MODES:
            raise ValueError(f"Unsupported gesture mode: {mode}")
        self.mode = mode

    @property
    def tools(self) -> list:
        """Advertise concrete action choices and their handlers to Realtime."""
        if self.mode == "off":
            return []
        from pipecat.adapters.schemas.function_schema import FunctionSchema

        return [
            FunctionSchema(
                name="perform_gesture",
                description=(
                    "Thực hiện một cử chỉ khi người dùng yêu cầu hoặc mời robot làm, "
                    "kể cả cách nói lịch sự như 'bắt tay với mình được không?'. "
                    "Gọi ngay, không nói lời xác nhận trước khi có kết quả. "
                    "Không dùng cho lời chào thông thường, câu phủ định, trích dẫn "
                    "hoặc động tác chưa rõ."
                ),
                properties={
                    "gesture_id": {
                        "type": "string",
                        "enum": sorted(SUPPORTED_GESTURES),
                        "description": (
                            "wave: vẫy tay chào; heart: thả tim, tạo hình trái tim; "
                            "handshake: bắt tay; cool_pose: khoanh tay tạo dáng ngầu; "
                            "determined: giơ nắm tay quyết tâm; give_gift: đưa/tặng quà."
                        ),
                    }
                },
                required=["gesture_id"],
                handler=self._perform_gesture_handler,
            ),
            FunctionSchema(
                name="cancel_gesture",
                description="Dừng cử chỉ đang làm khi người dùng yêu cầu dừng hoặc hạ tay.",
                properties={}, required=[], handler=self.cancel_gesture,
            ),
            FunctionSchema(
                name="get_gesture_status",
                description="Kiểm tra cử chỉ đang chạy hay đã kết thúc khi người dùng hỏi trạng thái.",
                properties={}, required=[], handler=self.get_gesture_status,
            ),
        ]

    async def _perform_gesture_handler(self, params):
        # FunctionSchema handlers receive arguments through FunctionCallParams.
        await self.perform_gesture(params, params.arguments.get("gesture_id"))

    async def perform_gesture(self, params, gesture_id: str):
        """Ask the R1 owner to perform one approved gesture.

        Args:
            gesture_id: One approved gesture ID from SUPPORTED_GESTURES.
        """
        result = await self._dispatch(
            params.tool_call_id, command="play", gesture_id=gesture_id
        )
        await params.result_callback(result)

    async def cancel_gesture(self, params):
        """Stop the voice-owned gesture and return the arms through the owner."""
        result = await self._dispatch(params.tool_call_id, command="cancel")
        await params.result_callback(result)

    async def get_gesture_status(self, params):
        """Get gesture availability and the current high-level owner status."""
        result = await self._dispatch(params.tool_call_id, command="status")
        await params.result_callback(result)

    async def _dispatch(
        self,
        call_id: str,
        *,
        command: str,
        gesture_id: str | None = None,
    ) -> dict[str, Any]:
        if not self._valid_call_id(call_id):
            return {"ok": False, "status": "rejected", "reason": "invalid_request_id"}

        cached = self._results.get(call_id)
        if cached is not None:
            logger.info("Gesture tool duplicate call_id={} returned from cache", call_id)
            return cached

        if command == "play" and (
            not isinstance(gesture_id, str) or gesture_id not in SUPPORTED_GESTURES
        ):
            result = {
                "ok": False,
                "status": "rejected",
                "reason": "unknown_gesture",
                "available_gestures": sorted(SUPPORTED_GESTURES),
            }
        elif self.mode == "off":
            result = {"ok": False, "status": "disabled", "reason": "gesture_tools_off"}
        elif self.mode == "shadow":
            result = {
                "ok": True,
                "status": "shadow",
                "command": command,
                "gesture_id": gesture_id,
                "executed": False,
                "reason": "shadow_mode_motor_dispatch_disabled",
            }
        else:
            result = await self._send_owner_request(
                call_id=call_id, command=command, gesture_id=gesture_id
            )

        if command == "play" and result.get("ok") and result.get("status") == "accepted":
            self._voice_owned_active = True
        elif command == "cancel" and result.get("ok"):
            self._voice_owned_active = False

        self._remember(call_id, result)
        logger.info(
            "Gesture tool call_id={} command={} gesture={} result_status={} reason={}",
            call_id,
            command,
            gesture_id,
            result.get("status"),
            result.get("reason", ""),
        )
        return result

    async def disable_for_dev_exit(self) -> dict[str, Any] | None:
        """Immediately hide tools and retract a voice-owned gesture if needed."""
        active = self._voice_owned_active
        self.set_mode("off")
        if not active:
            return None

        self._dev_exit_sequence += 1
        result = await self._send_owner_request(
            call_id=f"dev-exit-{self._dev_exit_sequence}",
            command="cancel",
            gesture_id=None,
        )
        self._voice_owned_active = False
        logger.info(
            "Gesture Dev exit cancel result_status={} reason={}",
            result.get("status"),
            result.get("reason", ""),
        )
        return result

    @staticmethod
    def _valid_call_id(call_id: str) -> bool:
        return bool(call_id) and len(call_id) <= 128 and all(
            ch.isascii() and (ch.isalnum() or ch in "_-") for ch in call_id
        )

    async def _send_owner_request(
        self,
        *,
        call_id: str,
        command: str,
        gesture_id: str | None,
    ) -> dict[str, Any]:
        request = {
            "v": 1,
            "request_id": call_id,
            "command": command,
            "gesture_id": gesture_id,
            "source": "voice",
        }
        writer = None
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_unix_connection(str(self.socket_path)),
                timeout=self.timeout_s,
            )
            writer.write(json.dumps(request, separators=(",", ":")).encode() + b"\n")
            await asyncio.wait_for(writer.drain(), timeout=self.timeout_s)
            raw = await asyncio.wait_for(reader.readline(), timeout=self.timeout_s)
        except (OSError, asyncio.TimeoutError) as error:
            return {
                "ok": False,
                "status": "unavailable",
                "reason": "gesture_owner_unavailable",
                "detail": type(error).__name__,
            }
        finally:
            if writer is not None:
                writer.close()
                try:
                    await writer.wait_closed()
                except OSError:
                    pass

        try:
            result = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError):
            return {"ok": False, "status": "failed", "reason": "invalid_owner_response"}
        if not isinstance(result, dict) or not isinstance(result.get("ok"), bool) or not isinstance(
            result.get("status"), str
        ):
            return {"ok": False, "status": "failed", "reason": "invalid_owner_response"}
        if result.get("request_id", call_id) != call_id:
            return {"ok": False, "status": "failed", "reason": "invalid_owner_response"}
        return result

    def _remember(self, call_id: str, result: dict[str, Any]) -> None:
        self._results[call_id] = result
        self._results.move_to_end(call_id)
        while len(self._results) > self.cache_size:
            self._results.popitem(last=False)
