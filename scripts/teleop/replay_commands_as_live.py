#!/usr/bin/env python3
"""Replay a recorded command trace as if a headset were producing it now.

The live pipeline judges a command by its age, so a recorded
`raw_commands.jsonl` cannot simply be piped into it: those lines carry the
monotonic timestamps of the session that produced them, every one reads as
thousands of seconds stale, and the simulator correctly holds on all of them.
That makes the whole pipeline untestable without a Quest on someone's head.

This stands in for the bridge. It re-stamps each line with the current
monotonic clock and releases it on the trace's own intervals, so downstream
processes see a stream indistinguishable in timing from a live one. Nothing
else about the command is touched: poses, sequence ids and the deadman state
are passed through exactly as recorded.

It is a test instrument. It produces no evidence of its own and any run fed by
it is a replay, not a live session -- the resolved config records the command
source either way, so the two cannot be confused after the fact.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback rate; 1.0 reproduces the recorded intervals.",
    )
    parser.add_argument(
        "--max-gap-s", type=float, default=1.0,
        help=(
            "Longest pause honoured between two samples. A recorded trace can "
            "contain a minute-long gap where the operator stopped, and waiting it "
            "out teaches nothing."
        ),
    )
    args = parser.parse_args()
    if args.speed <= 0.0:
        raise SystemExit("--speed must be positive.")

    lines = [
        json.loads(line)
        for line in args.input.expanduser().resolve().read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if not lines:
        raise SystemExit("trace is empty")

    origin = float(lines[0]["timestamp_monotonic_s"])
    started = time.monotonic()
    drift = 0.0
    for index, command in enumerate(lines):
        offset = (float(command["timestamp_monotonic_s"]) - origin) / args.speed
        if index > 0:
            previous = (float(lines[index - 1]["timestamp_monotonic_s"]) - origin) / args.speed
            gap = offset - previous
            if gap > args.max_gap_s:
                # Collapse the pause rather than reproduce it, and keep the
                # collapsed amount so later samples stay on a consistent clock.
                drift += gap - args.max_gap_s
        due = started + offset - drift
        delay = due - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        command["timestamp_monotonic_s"] = time.monotonic()
        sys.stdout.write(json.dumps(command) + "\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
