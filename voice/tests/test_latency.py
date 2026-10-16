import unittest

from hb_voice.latency import looks_like_gesture_request


class GestureLatencyHintTest(unittest.TestCase):
    def test_supported_gesture_phrases_are_buffered(self):
        for text in (
            "Vẫy tay chào mọi người đi",
            "Bắt tay với mình được không?",
            "Tạo một trái tim nhé",
            "Giơ nắm tay quyết tâm nào",
            "Đưa quà cho bé đi",
            "Dừng lại và hạ tay xuống",
        ):
            with self.subTest(text=text):
                self.assertTrue(looks_like_gesture_request(text))

    def test_regular_and_camera_questions_keep_streaming(self):
        for text in (
            "Hôm nay là thứ mấy?",
            "Bạn nhìn thấy gì trước camera?",
            "Kể cho mình một câu chuyện ngắn",
        ):
            with self.subTest(text=text):
                self.assertFalse(looks_like_gesture_request(text))
