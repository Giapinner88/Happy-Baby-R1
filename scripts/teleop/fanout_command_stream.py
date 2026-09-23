#!/usr/bin/env python3
"""Forward JSONL to hardware while mirroring it to an auxiliary consumer.

The primary stdout path is synchronous and authoritative.  The mirror is fed
through a bounded queue on a daemon thread: a slow or failed simulator can lose
mirror samples, but it cannot delay or close the hardware command path.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mirror-path", type=Path, required=True)
    parser.add_argument("--stats-path", type=Path, required=True)
    parser.add_argument("--mirror-queue-size", type=int, default=512)
    args = parser.parse_args()
    if args.mirror_queue_size < 1:
        raise SystemExit("--mirror-queue-size must be positive")

    mirror_queue: "queue.Queue[str | None]" = queue.Queue(maxsize=args.mirror_queue_size)
    mirror_state: dict[str, object] = {"written": 0, "failure": None}

    def mirror_writer() -> None:
        try:
            with args.mirror_path.open("w", encoding="utf-8") as mirror:
                while True:
                    line = mirror_queue.get()
                    if line is None:
                        break
                    try:
                        mirror.write(line)
                        mirror.flush()
                    except BrokenPipeError:
                        mirror_state["failure"] = "mirror_closed"
                        break
                    mirror_state["written"] = int(mirror_state["written"]) + 1
        except OSError as exc:
            mirror_state["failure"] = f"{type(exc).__name__}: {exc}"

    writer = threading.Thread(target=mirror_writer, daemon=True)
    writer.start()
    input_count = 0
    primary_count = 0
    mirror_drop_count = 0
    primary_closed = False
    try:
        for line in sys.stdin:
            if not line.strip():
                continue
            input_count += 1
            try:
                sys.stdout.write(line)
                sys.stdout.flush()
            except BrokenPipeError:
                primary_closed = True
                sys.stdout = open("/dev/null", "w", encoding="utf-8")
                break
            primary_count += 1
            if mirror_state["failure"] is not None:
                continue
            try:
                mirror_queue.put_nowait(line)
            except queue.Full:
                try:
                    mirror_queue.get_nowait()
                except queue.Empty:
                    pass
                mirror_drop_count += 1
                try:
                    mirror_queue.put_nowait(line)
                except queue.Full:
                    mirror_drop_count += 1
    finally:
        try:
            mirror_queue.put_nowait(None)
        except queue.Full:
            try:
                mirror_queue.get_nowait()
            except queue.Empty:
                pass
            mirror_queue.put_nowait(None)
        writer.join(timeout=2.0)
        if writer.is_alive() and mirror_state["failure"] is None:
            mirror_state["failure"] = "mirror_writer_did_not_stop"
        args.stats_path.write_text(
            json.dumps(
                {
                    "input_line_count": input_count,
                    "primary_line_count": primary_count,
                    "primary_closed": primary_closed,
                    "mirror_line_count": mirror_state["written"],
                    "mirror_drop_count": mirror_drop_count,
                    "mirror_failure": mirror_state["failure"],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
