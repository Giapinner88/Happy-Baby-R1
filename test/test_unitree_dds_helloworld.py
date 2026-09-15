#!/usr/bin/env python3
"""Run Unitree SDK2 Python DDS HelloWorld publisher/subscriber end-to-end.

This script starts subscriber and publisher in separate subprocesses,
streams both outputs in real time, then prints a communication summary.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

LOOPBACK_CYCLONEDDS_URI = (
    "<CycloneDDS><Domain><General><Interfaces>"
    "<NetworkInterface name=\"lo\"/>"
    "</Interfaces><AllowMulticast>true</AllowMulticast>"
    "</General></Domain></CycloneDDS>"
)


def stream_output(process: subprocess.Popen, prefix: str, counter: dict, key: str) -> None:
    """Stream process output line by line and count success lines."""
    assert process.stdout is not None
    for raw_line in iter(process.stdout.readline, ""):
        line = raw_line.rstrip("\n")
        if line:
            print(f"[{prefix}] {line}")
            if "success" in line.lower():
                counter[key] += 1


def terminate_process(proc: subprocess.Popen, name: str, timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return

    print(f"[{name}] terminating...")
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass

    print(f"[{name}] did not exit after terminate, killing...")
    proc.kill()
    proc.wait(timeout=timeout)


def python_has_unitree_sdk(py_cmd: list[str]) -> bool:
    """Check whether a Python command can import unitree_sdk2py + cyclonedds."""
    check_cmd = py_cmd + [
        "-c",
        "import unitree_sdk2py, cyclonedds; print('ok')",
    ]
    result = subprocess.run(
        check_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.returncode == 0


def resolve_python_command(conda_env: str) -> tuple[list[str], str]:
    """Pick runtime Python: current interpreter first, then conda env fallback."""
    native_cmd = [sys.executable]
    if python_has_unitree_sdk(native_cmd):
        return [sys.executable, "-u"], f"native ({sys.executable})"

    conda_bin = shutil.which("conda")
    if conda_bin is None:
        return [sys.executable, "-u"], f"native ({sys.executable})"

    conda_cmd = [conda_bin, "run", "--no-capture-output", "-n", conda_env, "python"]
    if python_has_unitree_sdk(conda_cmd):
        return conda_cmd + ["-u"], f"conda:{conda_env}"

    return [sys.executable, "-u"], f"native ({sys.executable})"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Unitree SDK2 Python HelloWorld DDS integration test"
    )
    parser.add_argument("--startup-wait", type=float, default=2.0, help="Seconds to wait after starting subscriber")
    parser.add_argument("--after-publish-wait", type=float, default=1.0, help="Seconds to wait after publisher exits")
    parser.add_argument("--publisher-timeout", type=float, default=50.0, help="Timeout in seconds for publisher")
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"), help="Conda env name used as fallback runtime")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    sdk_root = repo_root / "third_party" / "unitree_sdk2_python"
    hello_dir = sdk_root / "example" / "helloworld"
    publisher = hello_dir / "publisher.py"
    subscriber = hello_dir / "subscriber.py"

    if not publisher.exists() or not subscriber.exists():
        print("Missing HelloWorld scripts. Expected:")
        print(f"- {publisher}")
        print(f"- {subscriber}")
        return 2

    env = os.environ.copy()
    env.setdefault("CYCLONEDDS_URI", LOOPBACK_CYCLONEDDS_URI)
    env["PYTHONUNBUFFERED"] = "1"

    py_cmd, runtime_name = resolve_python_command(args.conda_env)

    print("=== Unitree DDS HelloWorld Test ===")
    print(f"Python: {sys.executable}")
    print(f"Runtime mode: {runtime_name}")
    print(f"Command prefix: {' '.join(py_cmd)}")
    print(f"Subscriber: {subscriber}")
    print(f"Publisher: {publisher}")

    counters = {"publisher": 0, "subscriber": 0}

    sub_proc = subprocess.Popen(
        py_cmd + [str(subscriber)],
        cwd=str(hello_dir),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    sub_thread = threading.Thread(
        target=stream_output,
        args=(sub_proc, "SUB", counters, "subscriber"),
        daemon=True,
    )
    sub_thread.start()

    print(f"Waiting {args.startup_wait:.1f}s for subscriber startup...")
    time.sleep(args.startup_wait)

    pub_proc = subprocess.Popen(
        py_cmd + [str(publisher)],
        cwd=str(hello_dir),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    try:
        stream_output(pub_proc, "PUB", counters, "publisher")
        pub_return = pub_proc.wait(timeout=args.publisher_timeout)
    except subprocess.TimeoutExpired:
        print("[PUB] timeout reached, terminating publisher")
        terminate_process(pub_proc, "PUB")
        pub_return = 124

    time.sleep(args.after_publish_wait)
    terminate_process(sub_proc, "SUB")
    sub_thread.join(timeout=2.0)

    print("=== Summary ===")
    print(f"Publisher success lines: {counters['publisher']}")
    print(f"Subscriber success lines: {counters['subscriber']}")
    print(f"Publisher exit code: {pub_return}")
    print(f"Subscriber exit code: {sub_proc.returncode}")

    if counters["subscriber"] > 0 and pub_return == 0:
        print("RESULT: PASS")
        return 0

    print("RESULT: FAIL")
    print("Hint: ensure unitree_sdk2py + cyclonedds are installed in the selected runtime.")
    print(f"Try: conda run -n {args.conda_env} python {Path(__file__).resolve()}")
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Interrupted by user")
        raise SystemExit(130)
