"""Project-owned runtime switching between Quest passthrough and robot video.

The vendored TeleVuer revision selects its display mode only at construction
time.  This adapter leaves that source tree untouched and replaces only the
pass-through scene coroutine for this process.  Pose/controller acquisition is
still performed by the pinned vendor wrapper.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from multiprocessing import Value, shared_memory
from urllib.parse import urlsplit, urlunsplit

import numpy as np


CAMERA_TOGGLE_BUTTONS = {
    "right_a": "right_ctrl_aButton",
    "right_b": "right_ctrl_bButton",
    "left_x": "left_ctrl_aButton",
    "left_y": "left_ctrl_bButton",
    "right_thumbstick": "right_ctrl_thumbstick",
    "left_thumbstick": "left_ctrl_thumbstick",
}


@dataclass(frozen=True)
class RobotCameraConfig:
    """Display-only contract for a robot camera WebRTC offer endpoint."""

    webrtc_url: str
    toggle_button: str = "right_a"
    layout: str = "mono"
    aspect_ratio: float = 16.0 / 9.0

    def __post_init__(self) -> None:
        parsed = urlsplit(self.webrtc_url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("robot camera WebRTC URL must be an absolute http(s) URL")
        if parsed.username is not None or parsed.password is not None:
            raise ValueError("robot camera WebRTC URL must not contain embedded credentials")
        if self.toggle_button not in CAMERA_TOGGLE_BUTTONS:
            raise ValueError(f"unsupported camera toggle button: {self.toggle_button}")
        if self.layout not in {"mono", "stereo-sbs"}:
            raise ValueError("robot camera layout must be 'mono' or 'stereo-sbs'")
        if not 0.1 <= self.aspect_ratio <= 10.0:
            raise ValueError("robot camera aspect ratio must be in [0.1, 10.0]")

    @property
    def log_url(self) -> str:
        """Return a URL safe for status logs (query/fragment may hold tokens)."""

        parsed = urlsplit(self.webrtc_url)
        return urlunsplit((parsed.scheme, parsed.netloc, parsed.path, "", ""))


@dataclass(frozen=True)
class RobotCameraFrameConfig:
    """Shape and controller contract for locally decoded robot frames."""

    toggle_button: str = "right_a"
    width: int = 640
    height: int = 360
    aspect_ratio: float = 16.0 / 9.0

    def __post_init__(self) -> None:
        if self.toggle_button not in CAMERA_TOGGLE_BUTTONS:
            raise ValueError(f"unsupported camera toggle button: {self.toggle_button}")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("robot camera frame dimensions must be positive")
        if not 0.1 <= self.aspect_ratio <= 10.0:
            raise ValueError("robot camera aspect ratio must be in [0.1, 10.0]")


class CameraToggleLatch:
    """Edge-triggered view latch; the first observation only arms the latch."""

    def __init__(self, button: str = "right_a") -> None:
        if button not in CAMERA_TOGGLE_BUTTONS:
            raise ValueError(f"unsupported camera toggle button: {button}")
        self.button = button
        self.robot_camera_enabled = False
        self._previous_pressed: bool | None = None

    def update(self, telemetry: object) -> bool | None:
        pressed = bool(getattr(telemetry, CAMERA_TOGGLE_BUTTONS[self.button], False))
        if self._previous_pressed is None:
            self._previous_pressed = pressed
            return None
        changed = pressed and not self._previous_pressed
        self._previous_pressed = pressed
        if not changed:
            return None
        self.robot_camera_enabled = not self.robot_camera_enabled
        return self.robot_camera_enabled

    def set_enabled(self, enabled: bool) -> None:
        """Synchronize the latch after a rejected enable or safety fallback."""

        self.robot_camera_enabled = bool(enabled)


def build_switchable_televuer_wrapper(tv_wrapper_module: object, config: RobotCameraConfig, **kwargs: object):
    """Instantiate the vendor wrapper with a project-owned switchable scene.

    ``tv_wrapper_module`` is injected after the pinned TeleVuer tree is placed
    on ``sys.path``.  Temporarily replacing its constructor keeps all vendor
    transforms and telemetry semantics intact while avoiding edits under
    ``third_party/``.
    """

    vendor_televuer = tv_wrapper_module.TeleVuer
    vendor_module = __import__(vendor_televuer.__module__, fromlist=["TeleVuer"])

    class SwitchableTeleVuer(vendor_televuer):
        def __init__(self, *args: object, **vendor_kwargs: object) -> None:
            self.robot_camera_enabled_shared = Value("b", False, lock=True)
            super().__init__(*args, **vendor_kwargs)

        def set_robot_camera_enabled(self, enabled: bool) -> None:
            with self.robot_camera_enabled_shared.get_lock():
                self.robot_camera_enabled_shared.value = bool(enabled)

        def robot_camera_enabled(self) -> bool:
            with self.robot_camera_enabled_shared.get_lock():
                return bool(self.robot_camera_enabled_shared.value)

        async def main_pass_through(self, session: object) -> None:
            session.upsert(
                vendor_module.MotionControllers(
                    stream=True,
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )
            video_key = "hb-robot-camera"
            previous_enabled: bool | None = None
            while True:
                enabled = self.robot_camera_enabled()
                if enabled:
                    video_type = (
                        vendor_module.WebRTCStereoVideoPlane
                        if config.layout == "stereo-sbs"
                        else vendor_module.WebRTCVideoPlane
                    )
                    video_kwargs: dict[str, object] = {
                        "src": config.webrtc_url,
                        "iceServer": None,
                        "iceServers": [],
                        "key": video_key,
                        "aspect": config.aspect_ratio,
                        "height": 7,
                    }
                    if config.layout == "stereo-sbs":
                        video_kwargs["layout"] = "stereo-left-right"
                    session.upsert(video_type(**video_kwargs), to="bgChildren")
                elif previous_enabled:
                    session.remove @ video_key
                previous_enabled = enabled
                await asyncio.sleep(1.0 / self.display_fps)

    original = tv_wrapper_module.TeleVuer
    tv_wrapper_module.TeleVuer = SwitchableTeleVuer
    try:
        wrapper = tv_wrapper_module.TeleVuerWrapper(
            display_mode="pass-through",
            zmq=False,
            webrtc=False,
            webrtc_url=config.webrtc_url,
            **kwargs,
        )
    finally:
        tv_wrapper_module.TeleVuer = original
    return wrapper


def build_frame_switchable_televuer_wrapper(
    tv_wrapper_module: object,
    config: RobotCameraFrameConfig,
    **kwargs: object,
):
    """Instantiate a pass-through wrapper with switchable local RGB frames."""

    vendor_televuer = tv_wrapper_module.TeleVuer
    vendor_module = __import__(vendor_televuer.__module__, fromlist=["TeleVuer"])

    class FrameSwitchableTeleVuer(vendor_televuer):
        _BACKGROUND_KEY = "hb-robot-camera-background"

        def __init__(self, *args: object, **vendor_kwargs: object) -> None:
            self.robot_camera_enabled_shared = Value("b", False, lock=True)
            self.robot_camera_sequence_shared = Value("q", -1, lock=True)
            self.robot_camera_rgb_shm = shared_memory.SharedMemory(
                create=True,
                size=config.height * config.width * 3 * np.uint8().itemsize,
            )
            self.robot_camera_rgb = np.ndarray(
                (config.height, config.width, 3),
                dtype=np.uint8,
                buffer=self.robot_camera_rgb_shm.buf,
            )
            self.robot_camera_rgb.fill(0)
            self._robot_camera_closed = False
            try:
                super().__init__(*args, **vendor_kwargs)
            except BaseException:
                self.robot_camera_rgb_shm.close()
                self.robot_camera_rgb_shm.unlink()
                raise

        def set_robot_camera_enabled(self, enabled: bool) -> None:
            with self.robot_camera_enabled_shared.get_lock():
                self.robot_camera_enabled_shared.value = bool(enabled)

        def robot_camera_enabled(self) -> bool:
            with self.robot_camera_enabled_shared.get_lock():
                return bool(self.robot_camera_enabled_shared.value)

        def set_robot_camera_frame(self, bgr: np.ndarray, sequence: int) -> bool:
            frame = np.asarray(bgr)
            expected_shape = (config.height, config.width, 3)
            if frame.dtype != np.uint8 or frame.shape != expected_shape:
                raise ValueError(
                    f"robot camera frame must be uint8 BGR with shape {expected_shape}"
                )
            sequence = int(sequence)
            if sequence < 0:
                raise ValueError("robot camera sequence must be non-negative")
            with self.robot_camera_sequence_shared.get_lock():
                if sequence <= self.robot_camera_sequence_shared.value:
                    return False
                self.robot_camera_rgb[:] = frame[:, :, ::-1]
                self.robot_camera_sequence_shared.value = sequence
            return True

        def _scene_snapshot(self) -> tuple[bool, int, np.ndarray | None]:
            enabled = self.robot_camera_enabled()
            if not enabled:
                return False, -1, None
            with self.robot_camera_sequence_shared.get_lock():
                sequence = int(self.robot_camera_sequence_shared.value)
                image = self.robot_camera_rgb.copy() if sequence >= 0 else None
            return True, sequence, image

        async def main_pass_through(self, session: object) -> None:
            if self.use_hand_tracking:
                session.upsert(
                    vendor_module.Hands(
                        stream=True,
                        key="hands",
                        hideLeft=True,
                        hideRight=True,
                    ),
                    to="bgChildren",
                )
            else:
                session.upsert(
                    vendor_module.MotionControllers(
                        stream=True,
                        key="motionControllers",
                        left=True,
                        right=True,
                    ),
                    to="bgChildren",
                )
            background_present = False
            displayed_sequence = -1
            while True:
                enabled, sequence, image = self._scene_snapshot()
                if enabled and image is not None and (
                    not background_present or sequence != displayed_sequence
                ):
                    session.upsert(
                        vendor_module.ImageBackground(
                            image,
                            aspect=config.aspect_ratio,
                            height=1,
                            distanceToCamera=1,
                            format="jpeg",
                            quality=80,
                            key=self._BACKGROUND_KEY,
                            interpolate=True,
                        ),
                        to="bgChildren",
                    )
                    background_present = True
                    displayed_sequence = sequence
                elif not enabled and background_present:
                    session.remove @ self._BACKGROUND_KEY
                    background_present = False
                    displayed_sequence = -1
                await asyncio.sleep(1.0 / self.display_fps)

        def close(self) -> None:
            if self._robot_camera_closed:
                return
            self._robot_camera_closed = True
            try:
                super().close()
            finally:
                self.robot_camera_rgb_shm.close()
                try:
                    self.robot_camera_rgb_shm.unlink()
                except FileNotFoundError:
                    pass

    original = tv_wrapper_module.TeleVuer
    tv_wrapper_module.TeleVuer = FrameSwitchableTeleVuer
    wrapper_kwargs = dict(kwargs)
    wrapper_kwargs.update(
        {
            "binocular": False,
            "img_shape": (config.height, config.width),
            "display_mode": "pass-through",
            "zmq": False,
            "webrtc": False,
            "webrtc_url": None,
        }
    )
    try:
        wrapper = tv_wrapper_module.TeleVuerWrapper(**wrapper_kwargs)
    finally:
        tv_wrapper_module.TeleVuer = original
    return wrapper
