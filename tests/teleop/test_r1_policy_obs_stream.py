"""Kiểm tra kênh quan sát simulator → policy.

Không cần Isaac Sim và không cần model: hai đầu của kênh được dựng trong cùng
một tiến trình, trên một cổng thật, còn ảnh/state/prompt là dữ liệu dựng sẵn.
Điều đáng kiểm nhất không phải là "có nhận được gì không" mà là ba thứ đi cùng
một khung có về đúng nguyên vẹn hay không: lệch state một chiều, hay đảo kênh
màu, đều là lỗi câm mà policy không có cách nào báo.
"""

import time
import unittest

import numpy as np

from teleop.r1.policy_obs_stream import (
    HEADER,
    PolicyObsPublisher,
    PolicyObsSubscriber,
)


PORT = 5598


def _frame(height: int = 8, width: int = 12) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, : width // 2] = (255, 0, 0)  # nửa trái đỏ trong RGB
    return frame


def _pump(subscriber: PolicyObsSubscriber, timeout_s: float = 2.0):
    """SUB của ZMQ mất một chút để bắt tay, nên phải chờ chứ không đọc một lần."""

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        observation = subscriber.latest()
        if observation is not None:
            return observation
        time.sleep(0.01)
    return None


class PolicyObsStreamTest(unittest.TestCase):
    def setUp(self) -> None:
        self.publisher = PolicyObsPublisher(port=PORT)
        self.subscriber = PolicyObsSubscriber(port=PORT)
        self.addCleanup(self.publisher.close)
        self.addCleanup(self.subscriber.close)

    def _publish_until_received(self, rgb, state, task, attempts: int = 50):
        for _ in range(attempts):
            self.publisher.publish(rgb, state, task)
            observation = _pump(self.subscriber, timeout_s=0.1)
            if observation is not None:
                return observation
        return None

    def test_state_and_task_survive_the_round_trip(self) -> None:
        state = np.arange(12, dtype=np.float32) * 0.1 - 0.5
        task = "Move the right hand onto marker 5."
        observation = self._publish_until_received(_frame(), state, task)
        self.assertIsNotNone(observation, "không nhận được quan sát nào")
        _rgb, received_state, received_task = observation
        self.assertEqual(received_task, task)
        self.assertEqual(received_state.shape, (12,))
        np.testing.assert_allclose(received_state, state, rtol=0, atol=0)

    def test_channel_order_is_rgb_not_bgr(self) -> None:
        """Nửa trái phải về lại là ĐỎ. Nếu kênh bị đảo nó sẽ là xanh dương."""

        state = np.zeros(12, dtype=np.float32)
        observation = self._publish_until_received(_frame(), state, "task")
        self.assertIsNotNone(observation)
        rgb, _state, _task = observation
        left = rgb[:, :4].reshape(-1, 3).mean(axis=0)
        self.assertGreater(left[0], 200.0, f"kênh đỏ quá thấp: {left}")
        self.assertLess(left[2], 55.0, f"kênh xanh dương quá cao, có thể bị đảo BGR: {left}")

    def test_header_carries_dimensions_so_the_frame_parses_in_one_pass(self) -> None:
        state = np.zeros(7, dtype=np.float32)
        task = "xin chào"  # utf-8 nhiều byte: task_len phải tính theo byte
        observation = self._publish_until_received(_frame(), state, task)
        self.assertIsNotNone(observation)
        _rgb, received_state, received_task = observation
        self.assertEqual(received_state.shape, (7,))
        self.assertEqual(received_task, task)
        self.assertEqual(HEADER.size, 20)

    def test_oversized_task_is_dropped_rather_than_truncated(self) -> None:
        """Cắt ngắn câu lệnh sẽ đổi mục tiêu mà không ai biết; thà bỏ khung."""

        before = self.publisher.dropped_count
        self.publisher.publish(_frame(), np.zeros(12, dtype=np.float32), "x" * 70000)
        self.assertEqual(self.publisher.dropped_count, before + 1)
        self.assertEqual(self.publisher.published_count, 0)


if __name__ == "__main__":
    unittest.main()
