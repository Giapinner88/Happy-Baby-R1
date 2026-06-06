#!/usr/bin/env python3
"""Run Unitree's official G1 C++ controller against the local MuJoCo bridge."""

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
G1_DEPLOY_ROOT = REPO_ROOT / "third_party" / "unitree_rl_mjlab" / "deploy" / "robots" / "g1"
G1_CTRL = G1_DEPLOY_ROOT / "build" / "g1_ctrl"
ONNXRUNTIME_LIB = (
    REPO_ROOT
    / "third_party"
    / "unitree_rl_mjlab"
    / "deploy"
    / "thirdparty"
    / "onnxruntime-linux-x64-1.22.0"
    / "lib"
)


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


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run official Unitree RL Mjlab G1 controller in MuJoCo"
    )
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"))
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--startup-wait", type=float, default=0.5)
    parser.add_argument("--auto-sim", action="store_true", help="Auto transition Passive -> FixStand -> Velocity")
    parser.add_argument("--auto-passive-seconds", type=float, default=0.5)
    parser.add_argument("--auto-fixstand-seconds", type=float, default=3.0)
    parser.add_argument("--log-dir", default="/tmp/happy_baby_mujoco_official_g1")
    parser.add_argument("--viewer", action="store_true", help="Allow MuJoCo viewer if the desktop/display supports it")
    args = parser.parse_args()

    if not G1_CTRL.exists():
        print(f"Missing official controller: {G1_CTRL}")
        print("Build it with:")
        print("  cmake -S third_party/unitree_rl_mjlab/deploy/robots/g1 -B third_party/unitree_rl_mjlab/deploy/robots/g1/build")
        print("  cmake --build third_party/unitree_rl_mjlab/deploy/robots/g1/build -j$(nproc)")
        return 2

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    ctrl_log_path = log_dir / "g1_ctrl.log"
    sim_log_path = log_dir / "sim.log"

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    env["DOMAIN_ID"] = "0"
    env["INTERFACE"] = args.interface
    env["ROBOT"] = "g1"
    env["USE_JOYSTICK"] = "0"
    env["UNITREE_MUJOCO_ROOT"] = str(UNITREE_MUJOCO_ROOT)

    ld_paths = [str(ONNXRUNTIME_LIB), "/usr/local/lib"]
    if env.get("LD_LIBRARY_PATH"):
        ld_paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(ld_paths)

    ctrl_env = env.copy()
    if args.auto_sim:
        ctrl_env["HB_AUTO_SIM"] = "1"
        ctrl_env["HB_AUTO_SIM_PASSIVE_SECONDS"] = str(max(0.0, args.auto_passive_seconds))
        ctrl_env["HB_AUTO_SIM_FIXSTAND_SECONDS"] = str(max(0.0, args.auto_fixstand_seconds))

    sim_env = env.copy()
    if not args.viewer:
        sim_env["SDL_VIDEODRIVER"] = "dummy"

    ctrl_cmd = [str(G1_CTRL), f"--network={args.interface}"]
    sim_cmd = build_python_cmd(args.conda_env)

    print(f"Controller: {' '.join(ctrl_cmd)}")
    print(f"Simulator:  {' '.join(sim_cmd)}")
    print(f"DDS:        domain=0, interface={args.interface}")
    print(f"Auto FSM:   {'on' if args.auto_sim else 'off'}")
    print(f"Logs:       {log_dir}")

    with ctrl_log_path.open("w", encoding="utf-8") as ctrl_log, sim_log_path.open(
        "w", encoding="utf-8"
    ) as sim_log:
        ctrl_proc = subprocess.Popen(
            ctrl_cmd,
            cwd=G1_DEPLOY_ROOT / "build",
            env=ctrl_env,
            stdout=ctrl_log,
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
                if ctrl_proc.poll() is not None:
                    print("[g1_ctrl] exited; stopping simulator...")
                    break
                if sim_proc.poll() is not None:
                    print("[sim] exited; stopping controller...")
                    break
                time.sleep(0.2)
        finally:
            terminate(ctrl_proc, "g1_ctrl")
            terminate(sim_proc, "sim")

    print("=== g1_ctrl log tail ===")
    if ctrl_log_path.exists():
        print("\n".join(ctrl_log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-30:]))
    print("=== sim log tail ===")
    if sim_log_path.exists():
        print("\n".join(sim_log_path.read_text(encoding="utf-8", errors="replace").splitlines()[-20:]))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
