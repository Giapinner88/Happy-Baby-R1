"""Extend the installed HB voice session without changing vendor source."""
import asyncio
import json
import os
import time
from urllib import request

_search_pending = 0
_suppressed_responses = set()
_speaker = None
_front_notice = 'Mình chưa thấy vật ở phía trước. Mình sẽ xoay người để tìm tiếp.'
_front_notice_pcm = None


def _post(path, payload, timeout):
    url = 'http://127.0.0.1:' + os.environ.get('R1_P5_PORT', '5590') + path
    body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    req = request.Request(url, data=body, headers={'Content-Type': 'application/json'}, method='POST')
    with request.urlopen(req, timeout=timeout) as response:
        return json.load(response)


async def find_and_point(self, params):
    global _search_pending
    query = params.arguments.get('query', '')
    _search_pending += 1
    try:
        task = asyncio.create_task(asyncio.to_thread(_post, '/find', {'query': query}, 250))
        front_announced = False
        while not task.done():
            await asyncio.wait({task}, timeout=0.5)
            if task.done() or front_announced:
                continue
            try:
                progress = await asyncio.to_thread(_post, '/progress', {}, 2)
            except Exception:
                continue
            if progress.get('front_scan_complete'):
                front_announced = True
                await _announce_front_not_found(task)
        result = await task
    except Exception:
        result = {'status': 'unavailable', 'answer': 'Kết nối tìm vật đang không sẵn sàng.'}
    finally:
        _search_pending -= 1
    await params.result_callback(result)


def _speech_pcm(text):
    global _front_notice_pcm
    if text == _front_notice and _front_notice_pcm is not None:
        return _front_notice_pcm
    key = os.environ.get('OPENAI_API_KEY', '').strip()
    if not key:
        raise RuntimeError('OpenAI key unavailable for progress speech')
    payload = json.dumps({'model': 'gpt-4o-mini-tts', 'voice': 'sage',
                          'input': text, 'response_format': 'pcm'}, ensure_ascii=False).encode()
    req = request.Request('https://api.openai.com/v1/audio/speech', data=payload,
                          headers={'Authorization': 'Bearer ' + key,
                                   'Content-Type': 'application/json'}, method='POST')
    with request.urlopen(req, timeout=12) as response:
        pcm = response.read(500_001)
    if not pcm or len(pcm) > 500_000:
        raise ValueError('invalid progress speech PCM size')
    if text == _front_notice:
        _front_notice_pcm = pcm
    return pcm


async def _announce_front_not_found(task):
    if _speaker is None or task.done():
        return
    try:
        pcm = await asyncio.to_thread(_speech_pcm, _front_notice)
        if task.done() or not pcm:
            return
        from pipecat.frames.frames import OutputAudioRawFrame
        await _speaker._write_audio(OutputAudioRawFrame(audio=pcm, sample_rate=24000,
                                                       num_channels=1), defer_if_blocked=False)
        # The speaker bridge can queue PCM faster than it physically plays it.
        # Let this progress sentence finish before delivering the final tool result.
        await asyncio.sleep(min(5.0, max(0.0, _speaker._playback_deadline-time.monotonic())))
    except Exception as exc:
        from loguru import logger
        logger.warning('Front scan progress speech unavailable: {}', type(exc).__name__)


async def cancel_find(self, params):
    try:
        result = await asyncio.to_thread(_post, '/cancel', {}, 5)
    except Exception:
        result = {'status': 'unavailable', 'answer': 'Không kết nối được tác vụ tìm vật.'}
    await params.result_callback(result)


def install():
    from hb_voice import vision_tools
    from hb_voice import output
    from hb_voice.gesture_response import GestureResponseGuard
    from pipecat.adapters.schemas.function_schema import FunctionSchema
    from pipecat.services.openai.realtime.llm import OpenAIRealtimeLLMService

    # Realtime can begin another audio response while a long tool is still
    # running. Its mid-search guess must never reach the speaker or transcript.
    if not getattr(OpenAIRealtimeLLMService._handle_evt_audio_delta, '_r1_search', False):
        original_audio = OpenAIRealtimeLLMService._handle_evt_audio_delta
        original_transcript = OpenAIRealtimeLLMService._handle_evt_audio_transcript_delta
        original_text = GestureResponseGuard._handle_evt_text_delta
        original_done = OpenAIRealtimeLLMService._handle_evt_response_done
        original_speaker_init = output.UnitreeSpeakerBridge.__init__

        async def audio_delta(llm, evt):
            if _search_pending or evt.response_id in _suppressed_responses:
                _suppressed_responses.add(evt.response_id)
                return
            await original_audio(llm, evt)

        async def audio_transcript_delta(llm, evt):
            if _search_pending or evt.response_id in _suppressed_responses:
                _suppressed_responses.add(evt.response_id)
                return
            await original_transcript(llm, evt)

        async def text_delta(llm, evt):
            if _search_pending or evt.response_id in _suppressed_responses:
                _suppressed_responses.add(evt.response_id)
                return
            await original_text(llm, evt)

        async def response_done(llm, evt):
            try:
                await original_done(llm, evt)
            finally:
                _suppressed_responses.discard(evt.response.id)

        def speaker_init(speaker, *args, **kwargs):
            global _speaker
            original_speaker_init(speaker, *args, **kwargs)
            _speaker = speaker

        audio_delta._r1_search = True
        OpenAIRealtimeLLMService._handle_evt_audio_delta = audio_delta
        OpenAIRealtimeLLMService._handle_evt_audio_transcript_delta = audio_transcript_delta
        GestureResponseGuard._handle_evt_text_delta = text_delta
        OpenAIRealtimeLLMService._handle_evt_response_done = response_done
        output.UnitreeSpeakerBridge.__init__ = speaker_init

    original = vision_tools.VisionToolController.tools.fget
    if getattr(original, '_r1_p5', False):
        return

    def tools(self):
        async def handle_find(params):
            await find_and_point(self, params)

        async def handle_cancel(params):
            await cancel_find(self, params)

        return [*original(self),
                FunctionSchema(name='find_and_point',
                    description='Khi người dùng yêu cầu robot tìm một vật và quay đầu/chỉ tay vào nó, gọi công cụ này với mô tả một mục tiêu rõ ràng. Công cụ quét các hướng được cấu hình; có thể không thấy vật ở ngoài vùng quét. Chờ kết quả rồi trả lời đúng trạng thái. Không dùng inspect_camera cho cùng lượt tìm vật.',
                    properties={'query': {'type': 'string', 'description': 'Một vật cần tìm, kèm đặc điểm phân biệt nếu có.'}},
                    required=['query'], handler=handle_find),
                FunctionSchema(name='cancel_find',
                    description='Dừng tác vụ tìm vật hiện tại khi người dùng yêu cầu dừng tìm.',
                    properties={}, required=[], handler=handle_cancel)]

    tools._r1_p5 = True
    vision_tools.VisionToolController.tools = property(tools)
    vision_tools.VISION_TOOL_INSTRUCTIONS += (
        '\n- Khi người dùng yêu cầu tìm và chỉ một vật, gọi find_and_point với một mục tiêu rõ ràng. '
        'Không nói lời chờ hoặc suy đoán trong khi công cụ đang tìm. '
        'Sau khi đã quét ba ảnh trước mặt mà chưa thấy vật, hệ thống tự báo trạng thái và tiếp tục xoay tìm; '
        'không hỏi người dùng mô tả thêm trừ khi tool cuối cùng trả needs_clarification. '
        'Nếu có hai vật phù hợp, hỏi lại bên trái/phải hoặc to/nhỏ; sau câu trả lời, '
        'gọi lại find_and_point với chính câu trả lời để dùng tọa độ đã lưu, không quét lại. '
        'Nếu người dùng chuyển sang yêu cầu tìm vật khác, dùng đúng tên vật mới trong query; '
        'không tái sử dụng lựa chọn trái/phải của vật trước. '
        'Chỉ nói đã chỉ được vật khi tool trả status=found. '
        'Nếu tool báo chưa thấy hoặc thất bại, nói đúng trạng thái, không tự nhận đã hoàn thành. '
        'Khi người dùng nói dừng tìm, gọi cancel_find.'
    )


if os.environ.get('R1_P5_VOICE_ENABLED') == '1':
    install()
