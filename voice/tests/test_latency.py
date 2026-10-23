import unittest

from hb_voice.latency import (
    GestureTurnGuardProcessor,
    looks_like_camera_request,
    looks_like_gesture_request,
)


class GestureLatencyHintTest(unittest.TestCase):
    def test_supported_gesture_phrases_are_buffered(self):
        for text in (
            "Vẫy tay chào mọi người đi",
            "Bắt tay với mình được không?",
            "Tạo một trái tim nhé",
            "Giơ nắm tay quyết tâm nào",
            "Đưa quà cho bé đi",
            "Dừng lại và hạ tay xuống",
            "Chào bạn nha",
            "Tạm biệt, hẹn gặp lại",
            "Robot dễ thương quá",
            "Bạn thật thông minh và giỏi",
            "Rất vui được gặp bạn",
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

    def test_visual_questions_start_camera_prefetch(self):
        for text in (
            "Đây là số mấy?",
            "Cái gì đây?",
            "Bạn nhìn thấy gì trước mặt?",
            "Mình đang giơ mấy ngón tay?",
            "Xem giúp mình vật này nhé",
        ):
            with self.subTest(text=text):
                self.assertTrue(looks_like_camera_request(text))

    def test_non_visual_questions_do_not_start_camera_prefetch(self):
        for text in (
            "Hôm nay là thứ mấy?",
            "Kể mình một câu chuyện",
            "Bạn đang làm động tác gì?",
        ):
            with self.subTest(text=text):
                self.assertFalse(looks_like_camera_request(text))


class RealtimeTranscriptHintTest(unittest.TestCase):
    def test_split_interim_transcript_starts_camera_prefetch(self):
        camera_prefetches = []
        processor = GestureTurnGuardProcessor(
            lambda enabled: None,
            camera_prefetch=lambda: camera_prefetches.append(True),
        )
        processor.observe_transcript("Đây là số ", incremental=True)
        self.assertEqual(camera_prefetches, [])
        processor.observe_transcript("mấy", incremental=True)
        self.assertEqual(camera_prefetches, [True])
