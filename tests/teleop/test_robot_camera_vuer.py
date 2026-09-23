from __future__ import annotations

import asyncio
from multiprocessing import shared_memory
import sys
import types
from types import SimpleNamespace

import numpy as np
import pytest

from scripts.teleop.quest_bridge import _redacted_command
from scripts.teleop.robot_camera_vuer import (
    CameraToggleLatch,
    RobotCameraConfig,
    RobotCameraFrameConfig,
    build_frame_switchable_televuer_wrapper,
)


def telemetry(**overrides: bool) -> SimpleNamespace:
    values = {
        "right_ctrl_aButton": False,
        "right_ctrl_bButton": False,
        "left_ctrl_aButton": False,
        "left_ctrl_bButton": False,
        "right_ctrl_thumbstick": False,
        "left_ctrl_thumbstick": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_camera_starts_in_quest_view_and_toggles_on_press_edges() -> None:
    latch = CameraToggleLatch("right_a")
    assert latch.robot_camera_enabled is False
    assert latch.update(telemetry()) is None
    assert latch.update(telemetry(right_ctrl_aButton=True)) is True
    assert latch.update(telemetry(right_ctrl_aButton=True)) is None
    assert latch.update(telemetry()) is None
    assert latch.update(telemetry(right_ctrl_aButton=True)) is False


def test_held_button_on_first_sample_does_not_change_view() -> None:
    latch = CameraToggleLatch("right_a")
    assert latch.update(telemetry(right_ctrl_aButton=True)) is None
    assert latch.robot_camera_enabled is False


def test_selected_button_is_independent_of_trigger_and_other_buttons() -> None:
    latch = CameraToggleLatch("left_y")
    latch.update(telemetry())
    sample = telemetry(right_ctrl_aButton=True)
    sample.right_ctrl_trigger = True
    assert latch.update(sample) is None
    assert latch.update(telemetry(left_ctrl_bButton=True)) is True


def test_right_a_edges_are_independent_of_both_triggers() -> None:
    latch = CameraToggleLatch("right_a")
    initial = telemetry()
    initial.right_ctrl_trigger = False
    initial.left_ctrl_trigger = False
    assert latch.update(initial) is None

    right_deadman_and_a = telemetry(right_ctrl_aButton=True)
    right_deadman_and_a.right_ctrl_trigger = True
    right_deadman_and_a.left_ctrl_trigger = False
    assert latch.update(right_deadman_and_a) is True

    held_a_with_left_recalibration = telemetry(right_ctrl_aButton=True)
    held_a_with_left_recalibration.right_ctrl_trigger = True
    held_a_with_left_recalibration.left_ctrl_trigger = True
    assert latch.update(held_a_with_left_recalibration) is None

    released_a = telemetry()
    released_a.right_ctrl_trigger = False
    released_a.left_ctrl_trigger = True
    assert latch.update(released_a) is None

    second_a_edge = telemetry(right_ctrl_aButton=True)
    second_a_edge.right_ctrl_trigger = False
    second_a_edge.left_ctrl_trigger = True
    assert latch.update(second_a_edge) is False


def test_camera_config_validates_and_redacts_url_for_logs() -> None:
    config = RobotCameraConfig("https://robot.local/webrtc/offer?token=secret")
    assert config.log_url == "https://robot.local/webrtc/offer"
    assert _redacted_command(
        ["quest_bridge.py", "--robot-camera-webrtc-url", config.webrtc_url]
    )[-1] == config.log_url


@pytest.mark.parametrize(
    "kwargs",
    [
        {"webrtc_url": "rtsp://robot/camera"},
        {"webrtc_url": "https://user:secret@robot/camera"},
        {"webrtc_url": "https://robot/camera", "layout": "sideways"},
        {"webrtc_url": "https://robot/camera", "toggle_button": "trigger"},
    ],
)
def test_camera_config_rejects_unsupported_contracts(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        RobotCameraConfig(**kwargs)


class _Node:
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.args = args
        self.kwargs = kwargs


class _RemoveRecorder:
    def __init__(self, removed: list[str]) -> None:
        self._removed = removed

    def __matmul__(self, key: str) -> None:
        self._removed.append(key)


class _Session:
    def __init__(self) -> None:
        self.upserts: list[tuple[object, str | None]] = []
        self.removed: list[str] = []
        self.remove = _RemoveRecorder(self.removed)

    def upsert(self, node: object, to: str | None = None) -> None:
        self.upserts.append((node, to))


def _fake_tv_wrapper_module(monkeypatch: pytest.MonkeyPatch) -> object:
    module_name = "tests.fake_robot_camera_vendor"
    vendor = types.ModuleType(module_name)
    vendor.ImageBackground = type("ImageBackground", (_Node,), {})
    vendor.MotionControllers = type("MotionControllers", (_Node,), {})

    class FakeTeleVuer:
        def __init__(self, **kwargs: object) -> None:
            self.display_fps = float(kwargs.get("display_fps", 120.0))
            self.use_hand_tracking = bool(kwargs.get("use_hand_tracking", False))
            self.close_count = 0

        def close(self) -> None:
            self.close_count += 1

    FakeTeleVuer.__module__ = module_name
    vendor.TeleVuer = FakeTeleVuer
    monkeypatch.setitem(sys.modules, module_name, vendor)

    wrapper_module = SimpleNamespace()
    wrapper_module.TeleVuer = FakeTeleVuer

    class FakeWrapper:
        def __init__(self, **kwargs: object) -> None:
            self.kwargs = kwargs
            self.tvuer = wrapper_module.TeleVuer(**kwargs)

        def close(self) -> None:
            self.tvuer.close()

    wrapper_module.TeleVuerWrapper = FakeWrapper
    return wrapper_module


def test_frame_wrapper_copies_only_new_bgr_sequence_into_independent_rgb_memory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    wrapper_module = _fake_tv_wrapper_module(monkeypatch)
    config = RobotCameraFrameConfig(width=4, height=3)
    wrapper = build_frame_switchable_televuer_wrapper(
        wrapper_module,
        config,
        use_hand_tracking=False,
        binocular=False,
        display_fps=120.0,
    )
    tvuer = wrapper.tvuer
    shm_name = tvuer.robot_camera_rgb_shm.name
    try:
        bgr = np.zeros((3, 4, 3), dtype=np.uint8)
        bgr[0, 0] = [1, 2, 3]
        assert tvuer.robot_camera_rgb.shape == (3, 4, 3)
        assert tvuer.set_robot_camera_frame(bgr, sequence=2) is True
        assert tvuer.robot_camera_rgb[0, 0].tolist() == [3, 2, 1]
        tvuer.robot_camera_rgb[0, 0] = [9, 8, 7]
        assert tvuer.set_robot_camera_frame(bgr, sequence=2) is False
        assert tvuer.set_robot_camera_frame(bgr, sequence=1) is False
        assert tvuer.robot_camera_rgb[0, 0].tolist() == [9, 8, 7]
    finally:
        wrapper.close()
        wrapper.close()

    with pytest.raises(FileNotFoundError):
        shared_memory.SharedMemory(name=shm_name)


def test_frame_wrapper_adds_and_removes_one_full_field_background(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    wrapper_module = _fake_tv_wrapper_module(monkeypatch)
    config = RobotCameraFrameConfig(width=4, height=3, aspect_ratio=16.0 / 9.0)
    wrapper = build_frame_switchable_televuer_wrapper(
        wrapper_module,
        config,
        use_hand_tracking=False,
        binocular=False,
        display_fps=120.0,
    )
    tvuer = wrapper.tvuer
    session = _Session()
    frame = np.zeros((3, 4, 3), dtype=np.uint8)
    tvuer.set_robot_camera_frame(frame, sequence=1)
    tvuer.set_robot_camera_enabled(True)

    async def exercise_scene() -> None:
        task = asyncio.create_task(tvuer.main_pass_through(session))
        await asyncio.sleep(0.03)
        tvuer.set_robot_camera_enabled(False)
        await asyncio.sleep(0.03)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    try:
        asyncio.run(exercise_scene())
        controller_nodes = [
            node
            for node, target in session.upserts
            if type(node).__name__ == "MotionControllers" and target == "bgChildren"
        ]
        background_nodes = [
            node
            for node, target in session.upserts
            if type(node).__name__ == "ImageBackground" and target == "bgChildren"
        ]
        assert len(controller_nodes) == 1
        assert len(background_nodes) == 1
        background = background_nodes[0]
        assert background.kwargs == {
            "aspect": 16.0 / 9.0,
            "height": 1,
            "distanceToCamera": 1,
            "format": "jpeg",
            "quality": 80,
            "key": "hb-robot-camera-background",
            "interpolate": True,
        }
        assert session.removed == ["hb-robot-camera-background"]
    finally:
        wrapper.close()


@pytest.mark.parametrize(
    "kwargs",
    [
        {"width": 0},
        {"height": 0},
        {"aspect_ratio": 0.0},
        {"toggle_button": "trigger"},
    ],
)
def test_frame_config_rejects_invalid_contract(kwargs: dict[str, object]) -> None:
    with pytest.raises(ValueError):
        RobotCameraFrameConfig(**kwargs)
