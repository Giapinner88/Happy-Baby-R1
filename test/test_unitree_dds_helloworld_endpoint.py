#!/usr/bin/env python3
"""Run one side of the Unitree SDK2 Python DDS HelloWorld test.

Use this when publisher and subscriber run on different machines, for example
Ubuntu 20.04 and Ubuntu 22.04 hosts connected through Ethernet.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run publisher or subscriber endpoint for Unitree DDS HelloWorld"
    )
    parser.add_argument("--role", choices=("publisher", "subscriber"), required=True)
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument(
        "--interface",
        required=True,
        help="DDS network interface name, for example eth0, enp3s0, or lo",
    )
    parser.add_argument("--topic", default="happy_baby/dds_compat/helloworld")
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--read-timeout", type=float, default=2.0)
    return parser.parse_args()


def import_unitree_types() -> tuple[object, object, object, object]:
    repo_root = Path(__file__).resolve().parents[1]
    hello_dir = repo_root / "third_party" / "unitree_sdk2_python" / "example" / "helloworld"
    sys.path.insert(0, str(hello_dir))

    from unitree_sdk2py.core.channel import (  # type: ignore[import-not-found]
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from user_data import UserData  # type: ignore[import-not-found]

    return ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber, UserData


def run_publisher(
    channel_publisher: object,
    user_data: object,
    topic: str,
    count: int,
    period: float,
) -> int:
    publisher = channel_publisher(topic, user_data)
    publisher.Init()
    sent = 0

    try:
        for index in range(count):
            msg = user_data(" ", 0)
            msg.string_data = f"Happy Baby DDS compat {index}"
            msg.float_data = time.time()
            if publisher.Write(msg, 0.5):
                sent += 1
                print(f"[publisher] success {sent}/{count}: {msg}", flush=True)
            else:
                print("[publisher] waiting for subscriber", flush=True)
            time.sleep(period)
    finally:
        publisher.Close()

    return 0 if sent > 0 else 1


def run_subscriber(
    channel_subscriber: object,
    user_data: object,
    topic: str,
    count: int,
    read_timeout: float,
) -> int:
    subscriber = channel_subscriber(topic, user_data)
    subscriber.Init()
    received = 0

    try:
        deadline = time.monotonic() + read_timeout
        while received < count:
            msg = subscriber.Read()
            if msg is not None:
                received += 1
                deadline = time.monotonic() + read_timeout
                print(f"[subscriber] success {received}/{count}: {msg}", flush=True)
                continue

            if time.monotonic() >= deadline:
                print("[subscriber] timeout waiting for data", flush=True)
                break
            time.sleep(0.05)
    finally:
        subscriber.Close()

    return 0 if received > 0 else 1


def main() -> int:
    args = parse_args()
    channel_factory_initialize, channel_publisher, channel_subscriber, user_data = (
        import_unitree_types()
    )

    print(
        "DDS endpoint:",
        f"role={args.role}",
        f"domain={args.domain_id}",
        f"interface={args.interface}",
        f"topic={args.topic}",
        flush=True,
    )
    channel_factory_initialize(args.domain_id, args.interface)

    if args.role == "publisher":
        return run_publisher(
            channel_publisher,
            user_data,
            args.topic,
            args.count,
            args.period,
        )

    return run_subscriber(
        channel_subscriber,
        user_data,
        args.topic,
        args.count,
        args.read_timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())
