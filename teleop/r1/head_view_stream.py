"""Chuyển ảnh camera đầu robot từ simulator sang bridge để hiện trong kính.

Ảnh được render trong tiến trình simulator (môi trường `unitree_sim_env`), còn
`render_to_xr()` — hàm duy nhất đẩy ảnh vào kính — nằm trong tiến trình bridge
(môi trường `tv`). Ống hiện có chỉ chảy một chiều bridge → simulator và chỉ chở
lệnh, nên cần một kênh ngược. Module này là kênh đó.

Kiến trúc lấy theo `teleimager` của Unitree: ZMQ PUB/SUB, khung nén JPEG, hàng
đợi giữ đúng một khung mới nhất. Ảnh cũ vô dụng với người vận hành, nên thà bỏ
còn hơn xếp hàng.

Mỗi khung mang theo dấu thời gian `time.monotonic()` của lúc publish. Trên Linux
đồng hồ này là CLOCK_MONOTONIC toàn hệ thống nên so được giữa hai tiến trình
cùng máy — đó là cách đo độ trễ mà không cần đồng bộ đồng hồ. Nếu hai đầu chạy
khác máy thì con số này vô nghĩa, và cả pipeline vốn đã giả định cùng máy.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

# `<` = little-endian, không đệm. d = dấu thời gian publish, Q = số thứ tự khung.
HEADER = struct.Struct("<dQ")
DEFAULT_PORT = 5556
DEFAULT_JPEG_QUALITY = 80


@dataclass
class HeadViewPublisher:
    """Phía simulator: nén khung và phát đi. Không bao giờ chặn vòng điều khiển."""

    port: int = DEFAULT_PORT
    host: str = "127.0.0.1"
    jpeg_quality: int = DEFAULT_JPEG_QUALITY
    published_count: int = field(default=0, init=False)
    dropped_count: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        import zmq

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.PUB)
        # Chỉ giữ khung mới nhất ở phía gửi; khung cũ bị bỏ chứ không dồn ứ.
        self._socket.setsockopt(zmq.SNDHWM, 1)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.bind(f"tcp://{self.host}:{self.port}")

    def publish(self, rgb) -> None:
        """Phát một khung RGB. Lỗi mã hoá hoặc hàng đầy chỉ được đếm, không ném."""

        import cv2
        import numpy as np
        import zmq

        # `render_to_xr` nhận BGR và tự đổi sang RGB, nên đổi ở đây một lần rồi
        # gửi đi ở dạng bên nhận cần, thay vì đổi hai lần.
        bgr = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2BGR)
        ok, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)])
        if not ok:
            self.dropped_count += 1
            return
        payload = HEADER.pack(time.monotonic(), self.published_count) + buffer.tobytes()
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
            "head_view_published_count": self.published_count,
            "head_view_dropped_count": self.dropped_count,
        }


@dataclass
class HeadViewSubscriber:
    """Phía bridge: lấy khung mới nhất và đo độ trễ tới lúc nhận."""

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

    def latest_bgr(self):
        """Khung BGR mới nhất, hoặc None nếu chưa có khung nào kể từ lần gọi trước."""

        import cv2
        import numpy as np
        import zmq

        try:
            payload = self._socket.recv(zmq.NOBLOCK)
        except zmq.Again:
            return None
        if len(payload) <= HEADER.size:
            return None
        published_at, _sequence = HEADER.unpack(payload[: HEADER.size])
        frame = cv2.imdecode(np.frombuffer(payload[HEADER.size :], dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None
        self.latency_samples.append(time.monotonic() - published_at)
        self.received_count += 1
        return frame

    def close(self) -> None:
        self._socket.close(linger=0)
        self._context.term()

    def stats(self) -> dict[str, object]:
        """Thống kê độ trễ. Đây là con số mà cổng ra của Giai đoạn 3 đòi hỏi."""

        if not self.latency_samples:
            return {"head_view_received_count": 0, "head_view_latency_s": None}
        ordered = sorted(self.latency_samples)
        return {
            "head_view_received_count": self.received_count,
            "head_view_latency_s": {
                "count": len(ordered),
                "min": round(ordered[0], 6),
                "median": round(ordered[len(ordered) // 2], 6),
                "max": round(ordered[-1], 6),
                "mean": round(sum(ordered) / len(ordered), 6),
            },
        }


def stereo_side_by_side(bgr):
    """Nhân đôi ảnh đơn thành khung đôi mà TeleVuer chờ ở chế độ binocular.

    `img_shape=(480, 1280)` nghĩa là nửa trái cho mắt trái, nửa phải cho mắt
    phải. Một camera thì hai nửa giống nhau: không có chiều sâu lập thể, nhưng
    đúng định dạng và hiện được.
    """

    import numpy as np

    return np.concatenate([bgr, bgr], axis=1)


__all__ = [
    "DEFAULT_JPEG_QUALITY",
    "DEFAULT_PORT",
    "HEADER",
    "HeadViewPublisher",
    "HeadViewSubscriber",
    "stereo_side_by_side",
]
