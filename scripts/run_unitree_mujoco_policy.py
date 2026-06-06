#!/usr/bin/env python3
"""Launch Unitree MuJoCo simulator and a policy script together.

This keeps simulator and policy on the same DDS domain/interface and writes
both logs to a predictable directory.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
POLICY_ROOT = REPO_ROOT / "sim" / "unitree_mujoco_policy"
MODEL_ROOT = REPO_ROOT / "data" / "models" / "unitree_mujoco_policy"
UNITREE_MUJOCO_ROOT = REPO_ROOT / "third_party" / "unitree_mujoco"


def resolve_runtime_asset(value: str, default_name: str = "") -> Path | None:
    name = value.strip() or default_name
    if not name:
        return None

    path = Path(name).expanduser()
    if path.is_absolute():
        return path

    model_path = MODEL_ROOT / path
    if model_path.exists():
        return model_path

    policy_path = POLICY_ROOT / path
    if policy_path.exists():
        return policy_path

    return model_path


def default_policy_for(script_name: str) -> str:
    return {
        "run98.py": "policy98.onnx",
        "run98_2.py": "policy.onnx",
        "run480.py": "policy480.onnx",
        "run_ai.py": "policy.onnx",
        "run_dance.py": "policy_dance.onnx",
        "run_policy_dance.py": "policy_dance.onnx",
    }.get(script_name, "")


def default_motion_for(script_name: str) -> str:
    if script_name in {"run_dance.py", "run_policy_dance.py"}:
        return "G1_Take_102.bvh_60hz.csv"
    return ""


def bool_env(value: bool) -> str:
    return "1" if value else "0"


def build_python_cmd(conda_env: str | None, script: str) -> list[str]:
    if conda_env:
        return [
            "conda",
            "run",
            "--no-capture-output",
            "-n",
            conda_env,
            "python",
            "-u",
            script,
        ]
    return [sys.executable, "-u", script]


def terminate(proc: subprocess.Popen[str], name: str) -> None:
    if proc.poll() is not None:
        return
    print(f"[{name}] terminating...")
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        print(f"[{name}] killing...")
        proc.kill()
        proc.wait(timeout=3)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run Unitree MuJoCo simulator with a policy script"
    )
    parser.add_argument("--policy-script", default="run98_2.py")
    parser.add_argument("--policy-onnx", default="")
    parser.add_argument("--motion-csv", default="")
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"))
    parser.add_argument("--robot", default="g1")
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--policy-warmup", type=float, default=2.0, help="Seconds to hold/ramp the robot in FixStand before ONNX inference")
    parser.add_argument("--policy-fade", type=float, default=2.0, help="Seconds to fade ONNX actions in after FixStand")
    parser.add_argument("--policy-action-clip", type=float, default=0.6, help="Clip raw ONNX actions to this absolute value; set 0 to disable")
    parser.add_argument("--policy-target-rate-limit", type=float, default=4.0, help="Maximum target joint change in rad/s; set 0 to disable")
    parser.add_argument("--policy-fall-guard-gravity-z", type=float, default=-0.55, help="Reset to DEFAULT_Q if projected gravity z rises above this")
    parser.add_argument("--log-dir", default="/tmp/happy_baby_mujoco_policy")
    parser.add_argument("--viewer", action="store_true", help="Allow the MuJoCo viewer to open if the desktop/display supports it")
    parser.add_argument("--policy-window", action="store_true", help="Show the pygame GAMEPAD CONTROL window for keyboard control")
    args = parser.parse_args()

    if not POLICY_ROOT.exists():
        print(f"Missing policy directory: {POLICY_ROOT}")
        return 2

    sim_script = POLICY_ROOT / "unitree_mujoco2.py"
    if not sim_script.exists():
        print(f"Missing simulator script: {sim_script}")
        return 2

    policy_script = POLICY_ROOT / args.policy_script
    if not policy_script.exists():
        print(f"Missing policy script: {policy_script}")
        return 2

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    sim_log_path = log_dir / "sim.log"
    policy_log_path = log_dir / "policy.log"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["DOMAIN_ID"] = str(args.domain_id)
    env["INTERFACE"] = args.interface
    env["ROBOT"] = args.robot
    env["USE_JOYSTICK"] = "0"
    env["UNITREE_MUJOCO_ROOT"] = str(UNITREE_MUJOCO_ROOT)
    policy_onnx = resolve_runtime_asset(args.policy_onnx, default_policy_for(args.policy_script))
    if policy_onnx is not None:
        env["POLICY_ONNX"] = str(policy_onnx)
    env["POLICY_WARMUP_SECONDS"] = str(max(0.0, args.policy_warmup))
    env["POLICY_FADE_SECONDS"] = str(max(0.0, args.policy_fade))
    env["POLICY_ACTION_CLIP"] = str(max(0.0, args.policy_action_clip))
    env["POLICY_TARGET_RATE_LIMIT"] = str(max(0.0, args.policy_target_rate_limit))
    env["POLICY_FALL_GUARD_GRAVITY_Z"] = str(args.policy_fall_guard_gravity_z)
    motion_csv = resolve_runtime_asset(args.motion_csv, default_motion_for(args.policy_script))
    if motion_csv is not None:
        env["MOTION_CSV"] = str(motion_csv)

    sim_env = env.copy()
    policy_env = env.copy()
    if not args.viewer:
        sim_env["SDL_VIDEODRIVER"] = "dummy"
    if not args.policy_window:
        policy_env["SDL_VIDEODRIVER"] = "dummy"

    sim_cmd = build_python_cmd(args.conda_env, "unitree_mujoco2.py")
    policy_cmd = build_python_cmd(args.conda_env, args.policy_script)

    print(f"Policy:    {' '.join(policy_cmd)}")
    print(f"Simulator: {' '.join(sim_cmd)}")
    print(f"DDS:       domain={args.domain_id}, interface={args.interface}")
    print(f"Logs:      {log_dir}")

    with sim_log_path.open("w", encoding="utf-8") as sim_log, policy_log_path.open(
        "w", encoding="utf-8"
    ) as policy_log:
        policy_proc = subprocess.Popen(
            policy_cmd,
            cwd=POLICY_ROOT,
            env=policy_env,
            stdout=policy_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(args.startup_wait)
        sim_proc = subprocess.Popen(
            sim_cmd,
            cwd=POLICY_ROOT,
            env=sim_env,
            stdout=sim_log,
            stderr=subprocess.STDOUT,
            text=True,
        )

        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if policy_proc.poll() is not None:
                    print("[policy] exited; stopping simulator...")
                    break
                if sim_proc.poll() is not None:
                    print("[sim] exited; stopping policy...")
                    break
                time.sleep(0.2)
        finally:
            terminate(policy_proc, "policy")
            terminate(sim_proc, "sim")

    print("=== policy log tail ===")
    if policy_log_path.exists():
        lines = policy_log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        print("\n".join(lines[-20:]))
    print("=== sim log tail ===")
    if sim_log_path.exists():
        lines = sim_log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        print("\n".join(lines[-20:]))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
