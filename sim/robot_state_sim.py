#!/usr/bin/env python3
"""Local robot-state simulator for joint command/state flow.

This script models a small set of controllable joints, accepts either
interactive keyboard commands or a fixed trajectory, and prints the
resulting simulated state so you can exercise a control loop without
robot hardware.
"""

from __future__ import annotations

import argparse
import math
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple


DEFAULT_JOINTS = [
    "left_hip",
    "left_knee",
    "right_hip",
    "right_knee",
    "left_shoulder",
    "right_shoulder",
]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def smoothstep(ratio: float) -> float:
    ratio = clamp(ratio, 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


@dataclass
class JointState:
    position: float = 0.0
    velocity: float = 0.0
    target: float = 0.0
    stiffness: float = 28.0
    damping: float = 3.5


@dataclass
class RobotState:
    time_sec: float = 0.0
    base_x: float = 0.0
    base_y: float = 0.0
    base_z: float = 0.48
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    joints: Dict[str, JointState] = field(default_factory=dict)


class RobotSimulator:
    def __init__(self, joint_names: Iterable[str]) -> None:
        self.state = RobotState(
            joints={name: JointState() for name in joint_names}
        )
        self.current_pose_name = "stand"

    def set_pose(self, pose: Dict[str, float], pose_name: str = "custom") -> None:
        for joint_name, target in pose.items():
            if joint_name in self.state.joints:
                self.state.joints[joint_name].target = target
        self.current_pose_name = pose_name

    def set_joint_target(self, joint_name: str, target: float) -> None:
        if joint_name not in self.state.joints:
            raise KeyError(joint_name)
        self.state.joints[joint_name].target = target
        self.current_pose_name = "custom"

    def delta_joint_target(self, joint_name: str, delta: float) -> None:
        if joint_name not in self.state.joints:
            raise KeyError(joint_name)
        joint = self.state.joints[joint_name]
        joint.target += delta
        self.current_pose_name = "custom"

    def step(self, dt: float) -> RobotState:
        for joint in self.state.joints.values():
            acceleration = joint.stiffness * (joint.target - joint.position) - joint.damping * joint.velocity
            joint.velocity += acceleration * dt
            joint.position += joint.velocity * dt

        hip_pitch = 0.5 * (
            self.state.joints["left_hip"].position + self.state.joints["right_hip"].position
        )
        knee = 0.5 * (
            self.state.joints["left_knee"].position + self.state.joints["right_knee"].position
        )
        shoulder_diff = self.state.joints["left_shoulder"].position - self.state.joints["right_shoulder"].position
        shoulder_sum = self.state.joints["left_shoulder"].position + self.state.joints["right_shoulder"].position

        self.state.base_z = 0.50 - 0.05 * clamp(abs(knee), 0.0, 1.5) - 0.02 * clamp(abs(hip_pitch), 0.0, 1.5)
        self.state.base_x = 0.04 * math.tanh(hip_pitch)
        self.state.base_y = 0.03 * math.tanh(shoulder_diff)
        self.state.roll = 0.08 * math.tanh(shoulder_diff)
        self.state.pitch = 0.12 * math.tanh(hip_pitch)
        self.state.yaw = 0.04 * math.tanh(shoulder_sum)
        self.state.time_sec += dt
        return self.state

    def snapshot(self) -> RobotState:
        return self.state


def format_state(state: RobotState, joint_order: Iterable[str]) -> str:
    joint_bits = []
    for joint_name in joint_order:
        joint = state.joints[joint_name]
        joint_bits.append(f"{joint_name}={joint.position:+.3f}/{joint.target:+.3f}")
    return (
        f"t={state.time_sec:6.2f}s base=({state.base_x:+.2f},{state.base_y:+.2f},{state.base_z:+.2f}) "
        f"rpy=({state.roll:+.2f},{state.pitch:+.2f},{state.yaw:+.2f}) "
        + " ".join(joint_bits)
    )


def print_help(joint_names: Iterable[str]) -> None:
    print("Commands:")
    print("  help                         Show this help")
    print("  list                         Show joints and current targets")
    print("  stand | crouch | reach       Apply a named pose")
    print("  set <joint> <value>          Set one joint target in radians")
    print("  delta <joint> <value>        Offset one joint target in radians")
    print("  exit                         Stop the simulator")
    print("Joints:")
    print("  " + ", ".join(joint_names))


def pose_library() -> Dict[str, Dict[str, float]]:
    return {
        "stand": {
            "left_hip": 0.0,
            "left_knee": 0.0,
            "right_hip": 0.0,
            "right_knee": 0.0,
            "left_shoulder": 0.0,
            "right_shoulder": 0.0,
        },
        "crouch": {
            "left_hip": -0.28,
            "left_knee": 0.62,
            "right_hip": -0.28,
            "right_knee": 0.62,
            "left_shoulder": 0.12,
            "right_shoulder": -0.12,
        },
        "reach": {
            "left_hip": -0.10,
            "left_knee": 0.20,
            "right_hip": -0.10,
            "right_knee": 0.20,
            "left_shoulder": 0.75,
            "right_shoulder": 0.35,
        },
    }


def fixed_path(duration: float) -> List[Tuple[str, float, Dict[str, float]]]:
    library = pose_library()
    return [
        ("stand", max(0.8, duration * 0.20), library["stand"]),
        ("crouch", max(1.2, duration * 0.30), library["crouch"]),
        ("reach", max(1.0, duration * 0.25), library["reach"]),
        ("stand", max(1.0, duration * 0.25), library["stand"]),
    ]


def interpolate_path(segments: List[Tuple[str, float, Dict[str, float]]], elapsed: float) -> Tuple[str, Dict[str, float]]:
    total = sum(segment[1] for segment in segments)
    if total <= 0.0:
        name, _, pose = segments[0]
        return name, pose

    elapsed = elapsed % total
    cursor = 0.0
    for index, (name, segment_duration, pose) in enumerate(segments):
        next_cursor = cursor + segment_duration
        if elapsed <= next_cursor or index == len(segments) - 1:
            next_name, _, next_pose = segments[(index + 1) % len(segments)]
            ratio = 0.0 if segment_duration <= 0.0 else (elapsed - cursor) / segment_duration
            blend = smoothstep(ratio)
            blended = {}
            for joint_name in pose:
                blended[joint_name] = pose[joint_name] + (next_pose[joint_name] - pose[joint_name]) * blend
            return f"{name}->{next_name}", blended
        cursor = next_cursor

    name, _, pose = segments[-1]
    return name, pose


def command_reader(command_queue: "queue.Queue[str]") -> None:
    while True:
        try:
            line = input("cmd> ").strip()
        except EOFError:
            command_queue.put("exit")
            return
        command_queue.put(line)
        if line in {"exit", "quit"}:
            return


def parse_command(line: str, simulator: RobotSimulator, joint_names: Iterable[str]) -> bool:
    if not line:
        return True

    tokens = line.split()
    command = tokens[0].lower()

    if command in {"help", "h", "?"}:
        print_help(joint_names)
        return True

    if command == "list":
        print("Current joint targets:")
        for joint_name in joint_names:
            joint = simulator.snapshot().joints[joint_name]
            print(f"  {joint_name:>14s}: pos={joint.position:+.3f} target={joint.target:+.3f}")
        return True

    if command in pose_library():
        simulator.set_pose(pose_library()[command], pose_name=command)
        print(f"Applied pose: {command}")
        return True

    if command in {"set", "delta"}:
        if len(tokens) != 3:
            print(f"Usage: {command} <joint> <value>")
            return True
        joint_name = tokens[1]
        if joint_name not in simulator.snapshot().joints:
            print(f"Unknown joint: {joint_name}")
            return True
        try:
            numeric_value = float(tokens[2])
        except ValueError:
            print(f"Invalid value: {tokens[2]}")
            return True
        if command == "set":
            simulator.set_joint_target(joint_name, numeric_value)
        else:
            simulator.delta_joint_target(joint_name, numeric_value)
        print(f"Updated {joint_name} target")
        return True

    if command in {"exit", "quit"}:
        return False

    print(f"Unknown command: {line}")
    print("Type 'help' to see available commands.")
    return True


def run_keyboard_mode(simulator: RobotSimulator, hz: float) -> int:
    command_queue: "queue.Queue[str]" = queue.Queue()
    reader = threading.Thread(target=command_reader, args=(command_queue,), daemon=True)
    reader.start()

    dt = 1.0 / hz
    print_help(simulator.snapshot().joints.keys())
    print("Interactive mode started. Type commands and press Enter.")

    running = True
    while running:
        while True:
            try:
                line = command_queue.get_nowait()
            except queue.Empty:
                break
            running = parse_command(line, simulator, simulator.snapshot().joints.keys())
            if not running:
                break

        state = simulator.step(dt)
        print(format_state(state, simulator.snapshot().joints.keys()))
        time.sleep(dt)

    return 0


def run_path_mode(simulator: RobotSimulator, hz: float, duration: float, max_steps: int) -> int:
    dt = 1.0 / hz
    segments = fixed_path(duration)
    print("Fixed-path mode started.")
    print("Trajectory order: stand -> crouch -> reach -> stand")

    for step_index in range(max_steps):
        phase_name, pose = interpolate_path(segments, step_index * dt)
        simulator.set_pose(pose, pose_name=phase_name)
        state = simulator.step(dt)
        print(f"[{phase_name}] {format_state(state, simulator.snapshot().joints.keys())}")
        time.sleep(dt)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Simulate robot joint state transmission and reception")
    parser.add_argument("--mode", choices=("keyboard", "path"), default="path", help="Control source for the simulated robot")
    parser.add_argument("--hz", type=float, default=10.0, help="Simulation rate in Hz")
    parser.add_argument("--duration", type=float, default=8.0, help="Total duration used to build the fixed path")
    parser.add_argument("--steps", type=int, default=80, help="Number of simulation steps in path mode")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.hz <= 0.0:
        print("--hz must be greater than zero")
        return 2
    if args.steps <= 0:
        print("--steps must be greater than zero")
        return 2

    simulator = RobotSimulator(DEFAULT_JOINTS)
    simulator.set_pose(pose_library()["stand"], pose_name="stand")

    if args.mode == "keyboard":
        return run_keyboard_mode(simulator, args.hz)
    return run_path_mode(simulator, args.hz, args.duration, args.steps)


if __name__ == "__main__":
    raise SystemExit(main())