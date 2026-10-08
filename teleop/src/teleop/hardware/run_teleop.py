"""Read-only R1-A5 hardware preflight.

This module deliberately has no command transport and never creates a DDS
publisher. Its only purpose is to prove that the robot-side Python runtime can
read ``rt/lowstate`` and that the expected R1-A5 state shape is present. Motor
targets belong exclusively to ``hb_high_level``; Quest targets reach that sole
owner through :mod:`teleop.hardware.high_level_sidecar` and loopback UTL1.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any


STATE_TOPIC = "rt/lowstate"
R1_A5_MOTOR_COUNT = 35
R1_A5_ARM_INDICES = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)
R1_A5_HEAD_INDICES = (29, 30)  # pitch, yaw


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth10")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    parser.add_argument("--expected-mode-machine", type=int, default=1)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.timeout_s) or not 0.1 <= args.timeout_s <= 30.0:
        raise SystemExit("--timeout-s must be finite and in [0.1, 30.0]")
    if args.expected_mode_machine < 0:
        raise SystemExit("--expected-mode-machine must be non-negative")


def _wait_for_state(subscriber: Any, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = subscriber.Read()
        if message is not None:
            return message
        time.sleep(0.01)
    raise TimeoutError(f"no {STATE_TOPIC} sample within {timeout_s:.1f}s")


def _state_summary(state: Any, expected_mode_machine: int) -> dict[str, object]:
    motor_state = tuple(state.motor_state)
    if len(motor_state) != R1_A5_MOTOR_COUNT:
        raise RuntimeError(
            f"expected {R1_A5_MOTOR_COUNT} R1-A5 motor states, got {len(motor_state)}"
        )
    selected = R1_A5_ARM_INDICES + R1_A5_HEAD_INDICES
    positions = [float(motor_state[index].q) for index in selected]
    if not all(math.isfinite(value) for value in positions):
        raise RuntimeError("selected R1-A5 encoder positions are not finite")
    mode_machine = int(state.mode_machine)
    if mode_machine != expected_mode_machine:
        raise RuntimeError(
            f"mode_machine={mode_machine}, expected {expected_mode_machine}"
        )
    return {
        "state_topic": STATE_TOPIC,
        "mode_machine": mode_machine,
        "motor_count": len(motor_state),
        "selected_motor_indices": list(selected),
        "selected_position_rad": positions,
        "command_publisher_created": False,
    }


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _validate_args(args)

    # SDK imports remain inside main so --help and static tests cannot
    # initialize DDS as an import side effect.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber(STATE_TOPIC, LowState_)
    subscriber.Init()
    summary = _state_summary(
        _wait_for_state(subscriber, args.timeout_s), args.expected_mode_machine
    )
    print(json.dumps(summary, sort_keys=True))
    print("[SAFE] rt/lowstate verified; no publisher was created")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
