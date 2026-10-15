"""Offline checks for the one-frame Realtime vision probe."""

from __future__ import annotations

import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "test_realtime_vision.py"
SPEC = importlib.util.spec_from_file_location("hb_realtime_vision_probe", MODULE_PATH)
assert SPEC and SPEC.loader
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


def test_image_item_contains_text_and_jpeg_without_tools() -> None:
    jpeg = b"\xff\xd8payload\xff\xd9"
    event = PROBE.build_image_item(jpeg, "Ảnh có người không?")
    assert event["type"] == "conversation.item.create"
    content = event["item"]["content"]
    assert content[0] == {"type": "input_text", "text": "Ảnh có người không?"}
    assert content[1]["type"] == "input_image"
    assert content[1]["image_url"].startswith("data:image/jpeg;base64,")
    assert "tools" not in event and "tools" not in event["item"]


def test_complete_jpeg_allows_trailing_zero_padding() -> None:
    assert PROBE.is_complete_jpeg(b"\xff\xd8data\xff\xd9\x00\x00")
    assert not PROBE.is_complete_jpeg(b"\xff\xd8truncated")
    assert not PROBE.is_complete_jpeg(b"not-jpeg")


def test_extract_done_text_reads_text_and_transcript() -> None:
    response = {
        "output": [
            {
                "type": "message",
                "content": [
                    {"type": "output_text", "text": "Có một người. "},
                    {"type": "output_audio", "transcript": "Ảnh hơi tối."},
                ],
            },
            {"type": "function_call", "arguments": "{}"},
        ]
    }
    assert PROBE.extract_done_text(response) == "Có một người. Ảnh hơi tối."
