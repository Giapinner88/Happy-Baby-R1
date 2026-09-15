#!/usr/bin/env python3
"""Controller-side 4-DOF state subscriber + command publisher.

Run this in the Conda terminal to simulate the high-level controller.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from dataclasses import dataclass
from typing import Dict, List


JOINTS = ["left_hip", "left_knee", "right_hip", "right_knee"]


@dataclass
class JointState:
    position: float
    velocity: float


@dataclass
class ImuState:
    timestamp: float
    rpy: List[float]
    gyro: List[float]
    accel: List[float]


@dataclass
class ControlCmd:
    timestamp: float
    targets: Dict[str, float]
    torques: Dict[str, float]


def encode_message(payload: dict) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_message(raw: bytes) -> dict:
    return json.loads(raw.decode("utf-8"))


def make_socket(bind_addr: str | None, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if bind_addr is not None:
        sock.bind((bind_addr, port))
    return sock


def main() -> int:
    parser = argparse.ArgumentParser(description="Unitree R1 controller-side sim")
    parser.add_argument("--peer", default="127.0.0.1", help="Robot IP address")
    parser.add_argument("--bind", default="0.0.0.0", help="Bind address")
    parser.add_argument("--state-port", type=int, default=15110)
    parser.add_argument("--imu-port", type=int, default=15111)
    parser.add_argument("--cmd-port", type=int, default=15112)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--target", type=float, default=0.6)
    parser.add_argument("--kp", type=float, default=6.0)
    parser.add_argument("--kd", type=float, default=1.2)
    parser.add_argument("--print-hz", type=float, default=5.0)
    args = parser.parse_args()

    state_sock = make_socket(args.bind, args.state_port)
    imu_sock = make_socket(args.bind, args.imu_port)
    cmd_sock = make_socket(None, 0)
    state_sock.settimeout(0.01)
    imu_sock.settimeout(0.01)

    target = {name: args.target for name in JOINTS}
    last_state = {name: JointState(0.0, 0.0) for name in JOINTS}
    last_imu = ImuState(time.time(), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    start = time.time()
    next_print = start
    print_interval = 1.0 / max(1, args.print_hz)

    while True:
        now = time.time()
        if now - start > args.duration:
            break

        try:
            raw, _ = state_sock.recvfrom(65535)
            data = decode_message(raw)
            if data.get("type") == "state":
                for name in JOINTS:
                    joint = data["joints"][name]
                    last_state[name] = JointState(joint["position"], joint["velocity"])
        except socket.timeout:
            pass

        try:
            raw, _ = imu_sock.recvfrom(65535)
            data = decode_message(raw)
            if data.get("type") == "imu":
                last_imu = ImuState(
                    data["timestamp"], data["rpy"], data["gyro"], data["accel"]
                )
        except socket.timeout:
            pass

        torques = {}
        for name in JOINTS:
            pos = last_state[name].position
            vel = last_state[name].velocity
            torques[name] = args.kp * (target[name] - pos) - args.kd * vel

        cmd = ControlCmd(timestamp=now, targets=target, torques=torques)
        cmd_sock.sendto(
            encode_message(
                {
                    "type": "cmd",
                    "timestamp": cmd.timestamp,
                    "targets": cmd.targets,
                    "torques": cmd.torques,
                }
            ),
            (args.peer, args.cmd_port),
        )

        if now >= next_print:
            next_print = now + print_interval
            print(
                "[ctrl] "
                + " ".join(
                    f"{name}={last_state[name].position:+.3f}"
                    for name in JOINTS
                )
                + f" rpy={last_imu.rpy[0]:+.2f}/{last_imu.rpy[2]:+.2f}"
                + f" tau={torques[JOINTS[0]]:+.3f}"
            )

        time.sleep(args.dt)

    print("=== Controller sim complete ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
