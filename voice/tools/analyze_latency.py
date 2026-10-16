#!/usr/bin/env python3
"""Summarize VOICE_LATENCY journal lines for a matched chat/gesture trial."""

from __future__ import annotations

import argparse
import json
import re
import statistics
from pathlib import Path


LINE = re.compile(
    r"VOICE_LATENCY turn=(?P<turn>\d+) kind=(?P<kind>\w+) "
    r"metric=(?P<metric>\w+) ms=(?P<ms>[0-9.]+)"
)


def summarize(values: list[float]) -> dict[str, float | int]:
    ordered = sorted(values)
    p95_index = max(0, min(len(ordered) - 1, int(0.95 * len(ordered)) - 1))
    return {
        "count": len(ordered),
        "median_ms": round(statistics.median(ordered), 1),
        "p95_ms": round(ordered[p95_index], 1),
        "min_ms": round(ordered[0], 1),
        "max_ms": round(ordered[-1], 1),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="journalctl output containing VOICE_LATENCY")
    parser.add_argument("--minimum", type=int, default=20)
    args = parser.parse_args()

    groups: dict[str, list[float]] = {"chat": [], "gesture": [], "camera": []}
    for line in args.log.read_text(encoding="utf-8", errors="replace").splitlines():
        match = LINE.search(line)
        if not match or match.group("metric") != "turn_to_first_audio_ms":
            continue
        kind = match.group("kind")
        if kind in groups:
            groups[kind].append(float(match.group("ms")))

    result = {kind: summarize(values) for kind, values in groups.items() if values}
    result["protocol"] = {
        "required_chat": args.minimum,
        "required_gesture": args.minimum,
        "complete": (
            len(groups["chat"]) >= args.minimum
            and len(groups["gesture"]) >= args.minimum
        ),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["protocol"]["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
