#!/usr/bin/env python3
"""Emit a synthetic R1TeleopCommand stream for plumbing preflight only.

This exists to exercise `run_r1_quest3_live.py` end to end without a headset. Its
output is NOT T001 evidence: no Quest is connected, so it establishes nothing
about live acquisition or transport. A T001 evidence run must be fed by
`quest_bridge.py` with a real Quest session.

The stream is built from the same `QuestCommandBridge` the live bridge uses, so
the preflight exercises the real normalization path rather than hand-written
JSON. It writes a head-yaw sweep and can optionally sweep both arms between a
compact pose and a reachable wide lateral pose. It then releases the deadman
and stops emitting so the consumer's disconnect/timeout hold can be observed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1 import QuestCommandBridge, QuestTransportSample  # noqa: E402


def yaw_matrix(yaw_rad: float) -> list[list[float]]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 1.5], [0.0, 0.0, 0.0, 1.0]]


def pose_matrix(x_m: float, y_m: float, z_m: float) -> list[list[float]]:
    return [[1.0, 0.0, 0.0, x_m], [0.0, 1.0, 0.0, y_m], [0.0, 0.0, 1.0, z_m], [0.0, 0.0, 0.0, 1.0]]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frequency-hz", type=float, default=30.0)
    parser.add_argument(
        "--start-delay-s", type=float, default=0.0,
        help="Wait before the first command so a slow simulator consumer can finish starting.",
    )
    parser.add_argument("--sweep-s", type=float, default=6.0, help="Deadman-held head yaw sweep duration.")
    parser.add_argument("--released-s", type=float, default=2.0, help="Deadman-released duration after the sweep.")
    parser.add_argument("--silence-s", type=float, default=3.0, help="Emit nothing for this long, then exit.")
    parser.add_argument("--yaw-amplitude-rad", type=float, default=0.5)
    parser.add_argument(
        "--arm-sweep", action="store_true",
        help="Sweep both virtual EEs from (0.40, +/-0.20, 0.10) m to the reachable wide pose (0.25, +/-0.55, 0.10) m.",
    )
    parser.add_argument("--realtime", action="store_true", help="Sleep between samples to mimic a live stream.")
    args = parser.parse_args()

    if args.frequency_hz <= 0.0 or min(args.start_delay_s, args.sweep_s, args.released_s, args.silence_s) < 0.0:
        raise SystemExit("Frequency must be positive and durations must be non-negative.")

    bridge = QuestCommandBridge()
    period_s = 1.0 / args.frequency_hz
    sweep_count = int(args.sweep_s * args.frequency_hz)
    released_count = int(args.released_s * args.frequency_hz)

    if args.start_delay_s > 0.0:
        print(json.dumps({"event": "preflight_start_delay", "delay_s": args.start_delay_s}), file=sys.stderr, flush=True)
        time.sleep(args.start_delay_s)

    for index in range(sweep_count + released_count):
        phase = index / max(1, sweep_count)
        yaw = args.yaw_amplitude_rad * math.sin(2.0 * math.pi * phase)
        extension = 0.5 - 0.5 * math.cos(2.0 * math.pi * min(phase, 1.0)) if args.arm_sweep else 0.0
        wrist_x = 0.40 - 0.15 * extension
        wrist_y = 0.20 + 0.35 * extension
        sample = QuestTransportSample(
            motion_data_ready=True,
            head_pose_matrix=yaw_matrix(yaw),
            left_wrist_pose_matrix=pose_matrix(wrist_x, wrist_y, 0.10),
            right_wrist_pose_matrix=pose_matrix(wrist_x, -wrist_y, 0.10),
            deadman_pressed=index < sweep_count,
        )
        command = bridge.build(sample, time.monotonic())
        if command is not None:
            print(json.dumps(command.as_dict(), sort_keys=True), flush=True)
        if args.realtime:
            time.sleep(period_s)

    print(
        json.dumps({"event": "preflight_silence_start", "silence_s": args.silence_s}),
        file=sys.stderr,
        flush=True,
    )
    if args.realtime:
        time.sleep(args.silence_s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
