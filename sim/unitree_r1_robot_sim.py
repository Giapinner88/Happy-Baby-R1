#!/usr/bin/env python3
"""Robot-side 4-DOF state publisher + command listener.

Run this in the ROS terminal to simulate the robot sending state/IMU and
receiving control commands from the controller.
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
class RobotState:
    timestamp: float
    joints: Dict[str, JointState]


@dataclass
class ImuState:
    timestamp: float
    rpy: List[float]
    gyro: List[float]
    accel: List[float]


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
    parser = argparse.ArgumentParser(description="Unitree R1 robot-side sim")
    parser.add_argument("--peer", default="127.0.0.1", help="Controller IP address")
    parser.add_argument("--bind", default="0.0.0.0", help="Bind address")
    parser.add_argument("--state-port", type=int, default=15110)
    parser.add_argument("--imu-port", type=int, default=15111)
    parser.add_argument("--cmd-port", type=int, default=15112)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--damping", type=float, default=0.2)
    parser.add_argument("--print-hz", type=float, default=5.0)
    args = parser.parse_args()

    state_sock = make_socket(None, 0)
    imu_sock = make_socket(None, 0)
    cmd_sock = make_socket(args.bind, args.cmd_port)
    cmd_sock.settimeout(0.001)

    last_cmd = {name: 0.0 for name in JOINTS}
    position = {name: 0.0 for name in JOINTS}
    velocity = {name: 0.0 for name in JOINTS}

    start = time.time()
    next_print = start
    print_interval = 1.0 / max(1, args.print_hz)

    while True:
        now = time.time()
        if now - start > args.duration:
            break

        try:
            raw, _ = cmd_sock.recvfrom(65535)
            data = decode_message(raw)
            if data.get("type") == "cmd":
                for name in JOINTS:
                    last_cmd[name] = float(data["torques"].get(name, 0.0))
        except socket.timeout:
            pass

        for name in JOINTS:
            accel = last_cmd[name] - args.damping * velocity[name]
            velocity[name] += accel * args.dt
            position[name] += velocity[name] * args.dt

        state = RobotState(
            timestamp=now,
            joints={
                name: JointState(position=position[name], velocity=velocity[name])
                for name in JOINTS
            },
        )
        imu = ImuState(
            timestamp=now,
            rpy=[position["left_hip"] * 0.2, 0.0, position["right_hip"] * -0.2],
            gyro=[velocity["left_hip"], 0.0, velocity["right_hip"]],
            accel=[last_cmd["left_hip"], 0.0, last_cmd["right_hip"]],
        )

        state_sock.sendto(
            encode_message(
                {
                    "type": "state",
                    "timestamp": state.timestamp,
                    "joints": {
                        name: {
                            "position": state.joints[name].position,
                            "velocity": state.joints[name].velocity,
                        }
                        for name in JOINTS
                    },
                }
            ),
            (args.peer, args.state_port),
        )
        imu_sock.sendto(
            encode_message(
                {
                    "type": "imu",
                    "timestamp": imu.timestamp,
                    "rpy": imu.rpy,
                    "gyro": imu.gyro,
                    "accel": imu.accel,
                }
            ),
            (args.peer, args.imu_port),
        )

        if now >= next_print:
            next_print = now + print_interval
            print(
                "[robot] "
                + " ".join(
                    f"{name}={position[name]:+.3f}/{velocity[name]:+.3f}"
                    for name in JOINTS
                )
                + f" cmd={last_cmd[JOINTS[0]]:+.3f}"
            )

        time.sleep(args.dt)

    print("=== Robot sim complete ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
