#!/usr/bin/env python3
"""Launch the local R1 Unitree MuJoCo simulator and single policy runner.

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
UNITREE_MUJOCO_ROOT = REPO_ROOT / "third_party" / "unitree_mujoco"


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
        description="Run the R1 Unitree MuJoCo simulator with policy_runner.py"
    )
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"))
    parser.add_argument("--scene", default="", help="Optional R1 scene override, e.g. scene.xml or scene_hanging.xml.")
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--policy-warmup", type=float, default=0.0, help="Seconds to hold/ramp the robot in FixStand before ONNX inference")
    parser.add_argument("--policy-fade", type=float, default=0.0, help="Seconds to fade ONNX actions in after FixStand")
    parser.add_argument("--policy-action-clip", type=float, default=0.0, help="Clip raw ONNX actions to this absolute value; set 0 to disable")
    parser.add_argument("--policy-target-rate-limit", type=float, default=4.0, help="Maximum target joint change in rad/s; set 0 to disable")
    parser.add_argument("--policy-fall-guard-gravity-z", type=float, default=-0.55, help="Reset to DEFAULT_Q if projected gravity z rises above this")
    parser.add_argument("--cmd-vx", type=float, default=0.0, help="Scripted forward velocity command for unattended walking tests")
    parser.add_argument("--cmd-vy", type=float, default=0.0, help="Scripted lateral velocity command for unattended walking tests")
    parser.add_argument("--cmd-yaw", type=float, default=0.0, help="Scripted yaw rate command for unattended walking tests")
    parser.add_argument("--cmd-start", type=float, default=1.0, help="Seconds before scripted command starts")
    parser.add_argument("--cmd-stop", type=float, default=0.0, help="Seconds to stop scripted command; 0 keeps it active")
    parser.add_argument("--cmd-ramp", type=float, default=1.0, help="Seconds to smoothly ramp scripted command")
    parser.add_argument("--manual-vx-scale", type=float, default=0.1, help="Keyboard/gamepad forward command scale")
    parser.add_argument("--manual-vy-scale", type=float, default=0.05, help="Keyboard/gamepad lateral command scale")
    parser.add_argument("--manual-yaw-scale", type=float, default=0.4, help="Keyboard/gamepad yaw command scale")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--viewer", action="store_true", help="Allow the MuJoCo viewer to open if the desktop/display supports it")
    parser.add_argument("--policy-window", action="store_true", help="Show the pygame GAMEPAD CONTROL window for keyboard control")
    parser.add_argument("--hanging", action="store_true", help="Keep r1_hanging_connect equality active if the scene defines it")
    args = parser.parse_args()

    if not POLICY_ROOT.exists():
        print(f"Missing policy directory: {POLICY_ROOT}")
        return 2

    sim_script = POLICY_ROOT / "unitree_mujoco2.py"
    if not sim_script.exists():
        print(f"Missing simulator script: {sim_script}")
        return 2

    policy_script = POLICY_ROOT / "policy_runner.py"
    if not policy_script.exists():
        print(f"Missing policy runner: {policy_script}")
        return 2

    if args.log_dir:
        log_dir = Path(args.log_dir).expanduser()
    else:
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        log_dir = REPO_ROOT / "data" / "runs" / "unitree_mujoco_policy" / stamp
    log_dir.mkdir(parents=True, exist_ok=True)
    sim_log_path = log_dir / "sim.log"
    policy_log_path = log_dir / "policy.log"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env.setdefault("PYTHONNOUSERSITE", "1")
    env["DOMAIN_ID"] = str(args.domain_id)
    env["INTERFACE"] = args.interface
    env["ROBOT"] = "r1"
    if args.scene:
        env["ROBOT_SCENE_NAME"] = args.scene
    env["USE_JOYSTICK"] = "0"
    env["UNITREE_MUJOCO_ROOT"] = str(UNITREE_MUJOCO_ROOT)
    env["INIT_DEFAULT_Q"] = "1"
    env.setdefault("MUJOCO_GL", "egl")
    env["POLICY_WARMUP_SECONDS"] = str(max(0.0, args.policy_warmup))
    env["POLICY_FADE_SECONDS"] = str(max(0.0, args.policy_fade))
    env["POLICY_ACTION_CLIP"] = str(max(0.0, args.policy_action_clip))
    env["POLICY_TARGET_RATE_LIMIT"] = str(max(0.0, args.policy_target_rate_limit))
    env["POLICY_FALL_GUARD_GRAVITY_Z"] = str(args.policy_fall_guard_gravity_z)
    env["POLICY_CMD_VX"] = str(args.cmd_vx)
    env["POLICY_CMD_VY"] = str(args.cmd_vy)
    env["POLICY_CMD_YAW"] = str(args.cmd_yaw)
    env["POLICY_CMD_START_TIME"] = str(max(0.0, args.cmd_start))
    env["POLICY_CMD_STOP_TIME"] = str(max(0.0, args.cmd_stop))
    env["POLICY_CMD_RAMP_TIME"] = str(max(0.0, args.cmd_ramp))
    env["MANUAL_CMD_VX_SCALE"] = str(max(0.0, args.manual_vx_scale))
    env["MANUAL_CMD_VY_SCALE"] = str(max(0.0, args.manual_vy_scale))
    env["MANUAL_CMD_YAW_SCALE"] = str(max(0.0, args.manual_yaw_scale))
    env["DISABLE_HANGING_EQUALITY"] = "0" if args.hanging else "1"

    sim_env = env.copy()
    policy_env = env.copy()
    if not args.viewer:
        sim_env["SDL_VIDEODRIVER"] = "dummy"
    if not args.policy_window:
        policy_env["SDL_VIDEODRIVER"] = "dummy"

    sim_cmd = build_python_cmd(args.conda_env, str(sim_script))
    if args.scene:
        sim_cmd.extend(["--scene", args.scene])
    policy_cmd = build_python_cmd(args.conda_env, str(policy_script))

    print(f"Policy:    {' '.join(policy_cmd)}")
    print(f"Simulator: {' '.join(sim_cmd)}")
    print(f"DDS:       domain={args.domain_id}, interface={args.interface}")
    print(f"Config:    {POLICY_ROOT / 'config.py'}")
    print(f"Logs:      {log_dir}")

    with sim_log_path.open("w", encoding="utf-8") as sim_log, policy_log_path.open(
        "w", encoding="utf-8"
    ) as policy_log:
        policy_proc = subprocess.Popen(
            policy_cmd,
            cwd=log_dir,
            env=policy_env,
            stdout=policy_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        time.sleep(args.startup_wait)
        sim_proc = None
        if policy_proc.poll() is None:
            sim_proc = subprocess.Popen(
                sim_cmd,
                cwd=log_dir,
                env=sim_env,
                stdout=sim_log,
                stderr=subprocess.STDOUT,
                text=True,
            )
        else:
            print("[policy] exited before simulator startup; simulator was not started.")

        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                if policy_proc.poll() is not None:
                    print("[policy] exited; stopping simulator...")
                    break
                if sim_proc is not None and sim_proc.poll() is not None:
                    print("[sim] exited; stopping policy...")
                    break
                time.sleep(0.2)
        finally:
            terminate(policy_proc, "policy")
            if sim_proc is not None:
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
