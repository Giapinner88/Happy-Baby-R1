"""Kiểm tra kênh chở ảnh camera đầu từ simulator sang bridge.

Không cần Isaac Sim và không cần kính: hai đầu của kênh được dựng trong cùng
một tiến trình, trên một cổng thật, và ảnh là mảng numpy dựng sẵn.
"""

import time
import unittest

import numpy as np

from teleop.r1.head_view_stream import (
    HEADER,
    HeadViewPublisher,
    HeadViewSubscriber,
    stereo_side_by_side,
)
from teleop.r1.operator_cue import OperatorCue


PORT = 5599


def _frame(height: int = 8, width: int = 12) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, : width // 2] = (255, 0, 0)  # nửa trái đỏ trong RGB
    return frame


def _pump(subscriber: HeadViewSubscriber, timeout_s: float = 2.0):
    """SUB của ZMQ mất một chút để bắt tay, nên phải chờ chứ không đọc một lần."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        frame = subscriber.latest_bgr()
        if frame is not None:
            return frame
        time.sleep(0.01)
    return None


class HeadViewStreamTests(unittest.TestCase):
    def setUp(self) -> None:
        self.publisher = HeadViewPublisher(port=PORT)
        self.subscriber = HeadViewSubscriber(port=PORT)
        self.addCleanup(self.subscriber.close)
        self.addCleanup(self.publisher.close)

    def test_a_published_frame_arrives_with_the_same_shape(self) -> None:
        deadline = time.monotonic() + 2.0
        received = None
        while received is None and time.monotonic() < deadline:
            self.publisher.publish(_frame())
            received = self.subscriber.latest_bgr()
            time.sleep(0.01)
        self.assertIsNotNone(received, "không nhận được khung nào")
        self.assertEqual(received.shape, (8, 12, 3))

    def test_rgb_becomes_bgr_because_render_to_xr_expects_bgr(self) -> None:
        # `render_to_xr` tự gọi cvtColor(BGR2RGB), nên gửi RGB thẳng sẽ ra ảnh
        # đảo màu trong kính. Nửa trái đỏ trong RGB phải thành kênh 2 ở BGR.
        deadline = time.monotonic() + 2.0
        received = None
        while received is None and time.monotonic() < deadline:
            self.publisher.publish(_frame())
            received = self.subscriber.latest_bgr()
            time.sleep(0.01)
        self.assertIsNotNone(received)
        left = received[:, :4]
        self.assertGreater(int(left[..., 2].mean()), 200, "kênh đỏ phải nằm ở chỉ số 2 (BGR)")
        self.assertLess(int(left[..., 0].mean()), 60)

    def test_latency_is_measured_for_every_received_frame(self) -> None:
        received = 0
        deadline = time.monotonic() + 2.0
        while received < 3 and time.monotonic() < deadline:
            self.publisher.publish(_frame())
            if self.subscriber.latest_bgr() is not None:
                received += 1
            time.sleep(0.01)
        stats = self.subscriber.stats()
        self.assertEqual(stats["head_view_received_count"], received)
        self.assertEqual(stats["head_view_latency_s"]["count"], received)
        # Cùng máy, cùng CLOCK_MONOTONIC: độ trễ phải dương và rất nhỏ.
        self.assertGreater(stats["head_view_latency_s"]["min"], 0.0)
        self.assertLess(stats["head_view_latency_s"]["max"], 1.0)

    def test_no_frame_yields_none_instead_of_blocking(self) -> None:
        started = time.monotonic()
        self.assertIsNone(self.subscriber.latest_bgr())
        self.assertLess(time.monotonic() - started, 0.5, "recv phải không chặn")

    def test_stats_before_any_frame_report_nothing_rather_than_zero_latency(self) -> None:
        stats = self.subscriber.stats()
        self.assertEqual(stats["head_view_received_count"], 0)
        self.assertIsNone(stats["head_view_latency_s"])


class StereoLayoutTests(unittest.TestCase):
    def test_a_single_view_is_duplicated_into_the_binocular_layout(self) -> None:
        # TeleVuer chạy với img_shape=(480, 1280): nửa trái mắt trái, nửa phải
        # mắt phải. Một camera thì hai nửa giống nhau.
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        stereo = stereo_side_by_side(frame)
        self.assertEqual(stereo.shape, (480, 1280, 3))
        np.testing.assert_array_equal(stereo[:, :640], stereo[:, 640:])


class OperatorCueTests(unittest.TestCase):
    def test_headset_cue_decorates_a_copy_not_the_dataset_source(self) -> None:
        class Layout:
            target_digit = 3
            digit_to_slot = {8: 0, 3: 1, 5: 2, 1: 3}

        source = np.zeros((120, 320, 3), dtype=np.uint8)
        before = source.copy()
        cue = OperatorCue("TARGET NUMBER {n}", desktop_hud=False)
        cue.show(Layout())
        decorated = cue.decorate_head_view(source)
        np.testing.assert_array_equal(source, before)
        self.assertFalse(np.array_equal(decorated, source))


class HeaderTests(unittest.TestCase):
    def test_header_is_fixed_width_so_the_payload_split_is_unambiguous(self) -> None:
        self.assertEqual(HEADER.size, 16)
        packed = HEADER.pack(1234.5, 7)
        self.assertEqual(HEADER.unpack(packed), (1234.5, 7))


if __name__ == "__main__":
    unittest.main()
