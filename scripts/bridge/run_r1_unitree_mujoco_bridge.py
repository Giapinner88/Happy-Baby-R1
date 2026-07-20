#!/usr/bin/env python3
"""Run the local R1 Unitree MuJoCo DDS bridge without a policy process."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
POLICY_ROOT = REPO_ROOT / "sim" / "unitree_mujoco_policy"
UNITREE_MUJOCO_ROOT = REPO_ROOT / "third_party" / "unitree_mujoco"
LOCAL_MUJOCO_ROOT = REPO_ROOT / "assets" / "mujoco"


def build_python_cmd(conda_env: str | None) -> list[str]:
    if conda_env:
        return [
            "conda",
            "run",
            "--no-capture-output",
            "-n",
            conda_env,
            "python",
            "-u",
            "unitree_mujoco2.py",
        ]
    return [sys.executable, "-u", "unitree_mujoco2.py"]


def terminate(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is not None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"))
    parser.add_argument("--robot", default="r1")
    parser.add_argument("--scene", default="scene_hanging.xml")
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--log-dir", default="/tmp/happy_baby_r1_mujoco_bridge")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--init-default-q", action="store_true")
    parser.add_argument("--elastic-band", action="store_true")
    args = parser.parse_args()

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "sim.log"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["ROBOT"] = args.robot
    env["ROBOT_SCENE_NAME"] = args.scene
    env["DOMAIN_ID"] = str(args.domain_id)
    env["INTERFACE"] = args.interface
    env["UNITREE_MUJOCO_ROOT"] = str(UNITREE_MUJOCO_ROOT)
    env["LOCAL_MUJOCO_ROOT"] = str(LOCAL_MUJOCO_ROOT)
    env["USE_JOYSTICK"] = "0"
    env["INIT_DEFAULT_Q"] = "1" if args.init_default_q else "0"
    env["ENABLE_ELASTIC_BAND"] = "1" if args.elastic_band else "0"
    if not args.viewer:
        env["SDL_VIDEODRIVER"] = "dummy"
        env.setdefault("MUJOCO_GL", "egl")

    cmd = build_python_cmd(args.conda_env)
    print(f"Simulator: {' '.join(cmd)}")
    print(f"Robot:     {args.robot}")
    print(f"Scene:     {args.scene}")
    print(f"DDS:       domain={args.domain_id}, interface={args.interface}")
    print(f"Logs:      {log_dir}")

    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            cmd,
            cwd=POLICY_ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        stopped_by_duration = False
        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    break
                time.sleep(0.2)
            stopped_by_duration = proc.poll() is None
        finally:
            terminate(proc)

    print("=== sim log tail ===")
    if log_path.exists():
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        print("\n".join(lines[-30:]))
    if stopped_by_duration:
        return 0
    return proc.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
