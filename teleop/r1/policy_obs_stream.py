"""Kênh quan sát simulator → policy cho rollout tự lái.

`head_view_stream` đã chở ảnh camera đầu sang bridge, nhưng nó chỉ chở ảnh.
Một policy học từ dataset D002 cần đúng ba thứ mà dataset đã dạy nó:
`observation.images.head_camera`, `observation.state` 12 chiều, và câu lệnh
ngôn ngữ của episode. Thiếu state thì vòng lặp không còn là closed-loop, và
thiếu prompt thì model không biết phải chạm vào chữ số nào.

Module này là kênh thứ hai, tách hẳn khỏi `head_view_stream` để đường teleop
của người vận hành không bị đụng tới. Cùng kiến trúc: ZMQ PUB/SUB, JPEG, hàng
đợi giữ đúng một khung mới nhất — khung cũ vô dụng với một policy đang điều
khiển, y như với người đang đeo kính.

Khung đi trên dây có bố cục cố định, đọc một lần từ trái sang phải:

    HEADER(<dQHH) | state[state_dim] float32 | task utf-8[task_len] | JPEG

`CONFLATE` của ZMQ không hỗ trợ multipart, nên tất cả nằm trong một message
duy nhất thay vì nhiều phần.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

# `<` = little-endian, không đệm. d = dấu thời gian publish, Q = số thứ tự
# khung, H = số chiều state, H = độ dài câu lệnh tính theo byte utf-8.
HEADER = struct.Struct("<dQHH")
DEFAULT_PORT = 5557
DEFAULT_JPEG_QUALITY = 95
"""Cao hơn mức 80 của head-view: ảnh này là đầu vào của model, không phải để
người nhìn, nên nhiễu nén là nhiễu đưa thẳng vào quan sát."""


@dataclass
class PolicyObsPublisher:
    """Phía simulator. Không bao giờ chặn vòng điều khiển."""

    port: int = DEFAULT_PORT
    host: str = "127.0.0.1"
    jpeg_quality: int = DEFAULT_JPEG_QUALITY
    published_count: int = field(default=0, init=False)
    dropped_count: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        import zmq

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        self._socket.setsockopt(zmq.SNDHWM, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(f"tcp://{self.host}:{self.port}")

    def publish(self, rgb, state_rad, task: str) -> None:
        """Phát một quan sát. Lỗi mã hoá hoặc hàng đầy chỉ được đếm, không ném."""

        import cv2
        import numpy as np
        import zmq

        state = np.asarray(state_rad, dtype=np.float32).reshape(-1)
        task_bytes = task.encode("utf-8")
        if state.size > 0xFFFF or len(task_bytes) > 0xFFFF:
            self.dropped_count += 1
            return
        bgr = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
        ok, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)])
        if not ok:
            self.dropped_count += 1
            return
        payload = (
            HEADER.pack(time.monotonic(), self.published_count, state.size, len(task_bytes))
            + state.tobytes()
            + task_bytes
            + buffer.tobytes()
        )
        try:
            self._socket.send(payload, zmq.NOBLOCK)
        except zmq.Again:
            self.dropped_count += 1
            return
        self.published_count += 1

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()

    def stats(self) -> dict[str, int]:
        return {
            "policy_obs_published_count": self.published_count,
            "policy_obs_dropped_count": self.dropped_count,
        }


@dataclass
class PolicyObsSubscriber:
    """Phía policy. Trả về quan sát mới nhất và đo độ trễ tới lúc nhận."""

    port: int = DEFAULT_PORT
    host: str = "127.0.0.1"
    received_count: int = field(default=0, init=False)
    latency_samples: list[float] = field(default_factory=list, init=False)

    def __post_init__(self) -> None:
        import zmq

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self._socket.connect(f"tcp://{self.host}:{self.port}")

    def latest(self):
        """`(rgb, state, task)` mới nhất, hoặc None nếu chưa có khung nào mới.

        `rgb` trả về đúng thứ tự kênh RGB mà dataset đã dùng, không phải BGR
        của OpenCV: sai thứ tự kênh ở đây là một domain shift lặng lẽ mà model
        không có cách nào báo.
        """

        import cv2
        import numpy as np
        import zmq

        try:
            payload = self._socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            return None
        if len(payload) <= HEADER.size:
            return None
        published_at, _sequence, state_dim, task_len = HEADER.unpack(payload[: HEADER.size])
        offset = HEADER.size
        state_bytes = state_dim * 4
        if len(payload) < offset + state_bytes + task_len:
            return None
        state = np.frombuffer(payload, dtype=np.float32, count=state_dim, offset=offset).copy()
        offset += state_bytes
        task = payload[offset : offset + task_len].decode("utf-8", errors="replace")
        offset += task_len
        bgr = cv2.imdecode(np.frombuffer(payload[offset:], dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        self.latency_samples.append(time.monotonic() - published_at)
        self.received_count += 1
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), state, task

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()

    def stats(self) -> dict[str, object]:
        if not self.latency_samples:
            return {"policy_obs_received_count": 0, "policy_obs_latency_s": None}
        ordered = sorted(self.latency_samples)
        return {
            "policy_obs_received_count": self.received_count,
            "policy_obs_latency_s": {
                "count": len(ordered),
                "min": round(ordered[0], 6),
                "median": round(ordered[len(ordered) // 2], 6),
                "max": round(ordered[-1], 6),
                "mean": round(sum(ordered) / len(ordered), 6),
            },
        }
