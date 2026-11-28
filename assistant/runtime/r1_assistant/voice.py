"""External HB voice extension; loaded before hb_voice.app, without source edits."""
import asyncio
import fcntl
import json
import os
from pathlib import Path
import time
from urllib import error, request

INSTRUCTIONS = """
# Tác vụ thống nhất
- Trò chuyện bình thường bằng tiếng Việt. Chỉ gọi công cụ khi đúng yêu cầu.
- Tìm/chỉ vật: find_and_point. Đi tới/tiến lại gần vật: approach_object (tự tìm,
  quay về phía vật rồi đi tới; không gọi find_and_point trước).
- query giữ nguyên tên vật và đặc điểm người dùng nói, không dịch, không đổi sang vật khác.
  Chỉ chọn một vật mỗi tác vụ; không gọi inspect_camera trước hai công cụ này.
- Khi công cụ hỏi lại, hỏi đúng câu đó. Với tìm/chỉ, gửi câu trả lời trái/phải,
  to/nhỏ lại find_and_point; không đổi vật nếu người dùng chưa yêu cầu.
- Camera/ảnh chỉ dùng khi người dùng vừa nói rõ yêu cầu (nhìn, mô tả, tìm, chỉ, đi tới).
  Không tự đề nghị hay tự chụp ảnh khi chỉ nghe tiếng động, câu lửng hoặc không có lời
  yêu cầu; không gọi inspect_camera để "xem thử". Công cụ trả not_requested thì chỉ nói
  "Mình chưa nghe rõ, bạn nói lại giúp mình nhé", không thử lại, không giải thích lý do.
- Dừng/hủy: cancel_task ngay. cancel_requested chưa xác nhận robot đã dừng.
- busy: báo đang bận; không xếp hàng, tự gọi lại hoặc chạy tác vụ thứ hai.
- Nói như người thật, ngắn, thân thiện: trước khi làm chỉ một câu rất ngắn (ví dụ "Để mình
  đi tới đó nhé"); khi xong chỉ nói đúng câu trong trường answer của kết quả, tự nhiên, không
  thêm giải thích. Không nói "ước lượng", "camera", "xác nhận", "tác vụ", "kết quả" hay lý do
  kỹ thuật, trừ khi người dùng hỏi.
- accepted chỉ là đã nhận việc, chưa xong; shadow là robot chưa đi. Không tự thử lại khi lỗi
  hoặc không thấy vật; nói ngắn điều cần người dùng làm.
- Không tự arm, đổi policy hay mode. Không đọc tên hàm, mã trạng thái hoặc cơ chế điều khiển.
""".strip()


# Consent gate: a camera/vision tool (which sends an image to OpenAI) runs only after a fresh, unused
# user transcript. One spoken request unlocks one capture; noise, filler and model-initiated calls are refused.
CONSENT = {"text": "", "t": 0.0, "used": True}
CONSENT_TTL_S = 60.0
# OpenAI Realtime often calls the tool a few ms BEFORE the user's transcript is delivered (seen 12:18:10: call .133,
# transcript .145). With no unused transcript yet, wait this long for it before refusing.
CONSENT_WAIT_S = 3.0
AFFIRM = {"có", "ừ", "ừm", "ok", "okay", "vâng", "dạ", "đồng", "rồi"}
GATED = ("inspect_camera", "find_and_point", "approach_object")


def note_transcript(text):
    CONSENT.update(text=str(text or ""), t=time.monotonic(), used=False)


def consented():
    words = CONSENT["text"].split()
    if CONSENT["used"] or time.monotonic() - CONSENT["t"] > CONSENT_TTL_S:
        return False
    if len(words) < 2 and not (words and words[0].strip(".,!?…").lower() in AFFIRM):
        return False
    CONSENT["used"] = True
    return True


def gated(handler):
    async def run(params):
        end = time.monotonic() + CONSENT_WAIT_S
        while time.monotonic() < end and (CONSENT["used"] or time.monotonic() - CONSENT["t"] > CONSENT_TTL_S):
            await asyncio.sleep(.05)   # no fresh transcript yet: the spoken request may still be arriving
        if not consented():
            await params.result_callback({
                "ok": False, "status": "not_requested",
                "answer": "Chưa nghe rõ yêu cầu nên chưa làm gì. Nói với người dùng đúng một câu ngắn: "
                          "'Mình chưa nghe rõ, bạn nói lại giúp mình nhé.' Không nhắc tới ảnh, camera, "
                          "trạng thái hay việc yêu cầu bị từ chối."})
            return
        await handler(params)
    return run


def install_tts_fallback():
    """tts.provider=elevenlabs needs ELEVENLABS_API_KEY and ELEVENLABS_VOICE_ID (/etc/hb/stack.env). Without them HB
    refuses to start; instead keep the robot speaking with the OpenAI voice and log why, until the keys are added."""
    if os.environ.get("HB_TTS_STRICT", "").strip() == "1":
        return
    try:
        from hb_voice import config as voice_config
    except ImportError:
        return
    cls = voice_config.VoiceConfig
    if getattr(cls, "_r1_tts_fallback", False):
        return
    original = cls.load.__func__

    def load(klass, tuning_path=None):
        try:
            return original(klass, tuning_path)
        except voice_config.VoiceConfigError as error:
            if "ELEVENLABS_" not in str(error):
                raise
            import re
            import sys
            import tempfile
            path = Path(tuning_path or os.environ.get("VOICE_TUNING_PATH", voice_config.DEFAULT_TUNING_PATH))
            if not path.is_absolute():
                path = voice_config.VOICE_ROOT / path
            text, count = re.subn(r'(?m)^(\s*provider:\s*)["\']?elevenlabs["\']?', r'\1"openai"', path.read_text(encoding="utf-8"))
            if count != 1:
                raise
            handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", prefix="hb_tuning_openai_fallback_", delete=False)
            with handle:
                handle.write(text)
            print("r1_assistant: %s; speaking with the OpenAI voice until the ElevenLabs keys are set" % error,
                  file=sys.stderr, flush=True)
            return original(klass, handle.name)

    cls.load = classmethod(load)
    cls._r1_tts_fallback = True


def call(path, payload):
    port = int(os.environ.get("R1_ASSISTANT_PORT", "5595"))
    req = request.Request("http://127.0.0.1:%d%s" % (port, path),
                          data=json.dumps(payload, ensure_ascii=False).encode(),
                          headers={"Content-Type": "application/json"}, method="POST")
    try:
        response = request.urlopen(req, timeout=3)
    except error.HTTPError as exc:
        response = exc
    with response:
        data = response.read(1048577)
    if len(data) > 1048576:
        raise ValueError("assistant reply too large")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("assistant reply must be an object")
    return value


SPOKEN_KEYS = ("status", "answer", "task_id")


def spoken(result):
    """What the voice model sees: status + one sentence to say (details stay in the service logs)."""
    result = result if isinstance(result, dict) else {}
    out = {k: result[k] for k in SPOKEN_KEYS if k in result}
    out.setdefault("status", "unknown")
    return out


async def invoke(params, action):
    task_id = None
    try:
        admitted = await asyncio.to_thread(call, "/tasks", {
            "action": action, "query": params.arguments.get("query", ""),
            "request_id": params.tool_call_id})
        if admitted.get("status") != "accepted":
            await params.result_callback(spoken(admitted))
            return
        task_id = admitted["task_id"]
        began = time.monotonic()
        while time.monotonic() - began < 320:
            state = await asyncio.to_thread(call, "/status", {"task_id": task_id})
            if not state.get("active") or state.get("status") == "fault":
                await params.result_callback(spoken(dict(state.get("result", state), task_id=task_id)))
                return
            await asyncio.sleep(.2)
        await asyncio.to_thread(call, "/cancel", {"task_id": task_id})
        await params.result_callback({"status": "cancel_requested", "task_id": task_id,
                                      "answer": "Tác vụ quá thời gian; đã yêu cầu dừng, chưa xác nhận kết quả."})
    except asyncio.CancelledError:
        try:
            await asyncio.to_thread(call, "/cancel", {"request_id": params.tool_call_id})
        except Exception:
            pass
        raise
    except Exception:
        try:
            await asyncio.to_thread(call, "/cancel", {"request_id": params.tool_call_id})
        except Exception:
            pass
        await params.result_callback({"status": "unavailable",
                                      "answer": "Chưa xác nhận được tác vụ. Hãy kiểm tra robot hoặc dùng tay cầm để dừng."})


async def find(params):
    await invoke(params, "find_and_point")


async def approach(params):
    await invoke(params, "approach")


async def cancel(params):
    try:
        result = await asyncio.to_thread(call, "/cancel", {})
    except Exception:
        result = {"status": "stop_unconfirmed", "answer": "Chưa xác nhận dừng; hãy dùng tay cầm."}
    await params.result_callback(result)


async def status(params):
    try:
        result = await asyncio.to_thread(call, "/status", {})
    except Exception:
        result = {"status": "unavailable", "answer": "Chưa đọc được trạng thái tác vụ."}
    await params.result_callback(result)


def install():
    from hb_voice import vision_tools, gesture_tools
    from pipecat.adapters.schemas.function_schema import FunctionSchema
    controller = vision_tools.VisionToolController
    if getattr(controller, "_r1_assistant_installed", False):
        return
    original_tools = controller.tools.fget
    replaced = {"find_and_point", "cancel_find", "approach_object", "cancel_approach",
                "approach_status", "cancel_task", "task_status"}

    def tools(self):
        available = [tool for tool in original_tools(self) if tool.name not in replaced]
        query = {"query": {"type": "string", "description": "Một vật đích và đặc điểm phân biệt."}}
        # FunctionSchema.handler is a read-only property: rebuild the schema with the gated handler instead.
        available = [FunctionSchema(name=tool.name, description=tool.description, properties=tool.properties,
                                    required=tool.required, handler=gated(tool.handler))
                     if tool.name in GATED and tool.handler is not None else tool for tool in available]
        for name, description, handler, properties, required in (
            ("find_and_point", "Tìm và chỉ một vật; có thể hỏi lại nếu nhiều vật phù hợp.", find, query, ["query"]),
            ("approach_object", "Tìm vật, quay về phía vật rồi đi tới khi mép thân còn cách vật khoảng 30 cm; nếu vật bị dời chỗ thì tự tìm lại và đổi hướng. Đích ước lượng chưa xác nhận khoảng cách thực.", approach, query, ["query"]),
            ("cancel_task", "Yêu cầu dừng ngay tác vụ robot đang thực hiện.", cancel, {}, []),
            ("task_status", "Đọc kết quả hoặc tiến độ tác vụ khi người dùng hỏi.", status, {}, []),
        ):
            available.append(FunctionSchema(name=name, description=description, properties=properties,
                                            required=required, handler=gated(handler) if name in GATED else handler))
        return available

    async def capture(self):
        async with self._capture_lock:
            admitted = await asyncio.to_thread(call, "/camera/acquire", {})
            if admitted.get("status") != "reserved":
                raise RuntimeError("camera busy or assistant unavailable")
            lease = None
            try:
                lease = open(os.environ.get("R1_ASSISTANT_CAMERA_LOCK", "/run/hb-approach/camera.lock"), "a")
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
                worker = asyncio.create_task(asyncio.to_thread(self._capture_frame))
                try:
                    return await asyncio.shield(worker)
                except asyncio.CancelledError:
                    await worker
                    raise
            finally:
                if lease:
                    lease.close()
                try:
                    await asyncio.to_thread(call, "/camera/release", {"task_id": admitted["task_id"]})
                except Exception:
                    pass

    async def owner(self, *, call_id, command, gesture_id):
        if command == "play":
            payload = {"action": "gesture", "gesture_id": gesture_id, "request_id": call_id}
            path = "/tasks"
        elif command == "cancel":
            payload, path = {}, "/gesture/cancel"
        else:
            payload, path = {}, "/gesture/status"
        try:
            return await asyncio.to_thread(call, path, payload)
        except Exception:
            return {"ok": False, "status": "unavailable", "reason": "assistant_unavailable"}

    install_tts_fallback()
    try:
        from hb_voice import latency
        original_transcript = latency.TurnLatencyTracker.set_transcript

        def set_transcript(self, text):
            note_transcript(text)
            return original_transcript(self, text)
        latency.TurnLatencyTracker.set_transcript = set_transcript
    except ImportError:
        pass  # hb_voice.latency absent: consented() stays False, so camera tools refuse (fail closed).
    # A missing TurnLatencyTracker raises AttributeError on purpose: the installer's doctor then aborts instead of
    # installing a voice stack whose camera gate has no transcript source.
    controller.tools = property(tools)
    controller._capture_serialized = capture
    controller._r1_assistant_installed = True
    gesture_tools.GestureToolController._send_owner_request = owner
    vision_tools.VISION_TOOL_INSTRUCTIONS += "\n\n" + INSTRUCTIONS
