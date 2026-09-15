#!/usr/bin/env python3
"""Launch the local R1 Unitree MuJoCo simulator and single policy runner.

This keeps simulator and policy on the same DDS domain/interface and writes
both logs to a predictable directory.
"""

from __future__ import annotations

import argparse
import atexit
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
POLICY_ROOT = REPO_ROOT / "sim" / "unitree_mujoco_policy"
UNITREE_MUJOCO_ROOT = REPO_ROOT / "third_party" / "unitree_mujoco"

# Children are launched via `conda run`, so a plain proc.terminate() only signals
# the wrapper and leaves the real python (policy_runner/simulator) orphaned — those
# orphans keep publishing on DDS and corrupt later runs. We start each child in its
# own session/process group and signal the whole group, and register the children
# here so signal/atexit handlers can reap them on any exit path.
_CHILD_PROCS: list[tuple[subprocess.Popen[str], str]] = []


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


def _signal_group(proc: subprocess.Popen[str], sig: int) -> None:
    """Send a signal to the child's whole process group, falling back to the process."""
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except (ProcessLookupError, PermissionError):
        try:
            proc.send_signal(sig)
        except ProcessLookupError:
            pass


def terminate(proc: subprocess.Popen[str], name: str) -> None:
    if proc.poll() is not None:
        return
    print(f"[{name}] terminating...")
    _signal_group(proc, signal.SIGTERM)
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        print(f"[{name}] killing...")
        _signal_group(proc, signal.SIGKILL)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass


def _reap_children() -> None:
    for proc, name in _CHILD_PROCS:
        terminate(proc, name)


def _install_signal_handlers() -> None:
    def _handler(signum: int, _frame: object) -> None:
        print(f"[launcher] received signal {signum}; stopping children...")
        _reap_children()
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, _handler)
    signal.signal(signal.SIGINT, _handler)
    atexit.register(_reap_children)


def main() -> int:
    _install_signal_handlers()
    parser = argparse.ArgumentParser(
        description="Run the R1 Unitree MuJoCo simulator with policy_runner.py"
    )
    parser.add_argument("--conda-env", default=os.environ.get("R1_CONDA_ENV", "r1_env"))
    parser.add_argument("--scene", default="", help="Optional R1 scene override, e.g. scene.xml or scene_hanging.xml.")
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--duration", type=float, default=12.0, help="Seconds to run; set 0 to run until the MuJoCo window/process exits")
    parser.add_argument("--startup-wait", type=float, default=3.0)
    parser.add_argument("--policy-name", default=os.environ.get("POLICY_NAME", ""), help="Policy config name from sim/unitree_mujoco_policy/config.py")
    parser.add_argument("--policy-onnx", default=os.environ.get("POLICY_ONNX", ""), help="Explicit ONNX policy path; overrides the selected policy's default model")
    parser.add_argument("--policy-warmup", type=float, default=0.0, help="Seconds to hold/ramp the robot in FixStand before ONNX inference")
    parser.add_argument("--policy-fade", type=float, default=0.0, help="Seconds to fade ONNX actions in after FixStand")
    parser.add_argument("--policy-action-clip", type=float, default=0.0, help="Clip raw ONNX actions to this absolute value; set 0 to disable")
    parser.add_argument("--policy-target-rate-limit", type=float, default=4.0, help="Maximum target joint change in rad/s; set 0 to disable")
    parser.add_argument("--policy-fall-guard-gravity-z", type=float, default=-0.55, help="Reset to DEFAULT_Q if projected gravity z rises above this")
    parser.add_argument("--gait-period", type=float, default=0.6, help="Gait clock period (s); MUST match the policy's training phase period (v1-v3=0.6, tuned flat_v4=0.8)")
    parser.add_argument("--safety-preset", choices=["off", "conservative"], default="conservative", help="Runtime safety limits for scripted/gamepad commands")
    parser.add_argument("--cmd-limit-vx", type=float, default=None, help="Absolute forward command limit in m/s; default comes from --safety-preset")
    parser.add_argument("--cmd-limit-vy", type=float, default=None, help="Absolute lateral command limit in m/s; default comes from --safety-preset")
    parser.add_argument("--cmd-limit-yaw", type=float, default=None, help="Absolute yaw command limit in rad/s; default comes from --safety-preset")
    parser.add_argument("--cmd-slew-vx", type=float, default=None, help="Forward command slew limit in m/s^2; 0 disables")
    parser.add_argument("--cmd-slew-vy", type=float, default=None, help="Lateral command slew limit in m/s^2; 0 disables")
    parser.add_argument("--cmd-slew-yaw", type=float, default=None, help="Yaw command slew limit in rad/s^2; 0 disables")
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
    parser.add_argument("--direct-eval", action="store_true", help="Run ONNX inference directly inside MuJoCo with MJLab-style position actuators")
    parser.add_argument("--bridge-actuator-mode", choices=["position", "torque"], default="position", help="MuJoCo bridge actuator semantics when not using --direct-eval")
    parser.add_argument("--hanging", action="store_true", help="Keep r1_hanging_connect equality active if the scene defines it")
    parser.add_argument("--record-video", action="store_true", help="Record the MuJoCo simulator view to an mp4 in the run log directory")
    parser.add_argument("--video-path", default="", help="Optional mp4 path; defaults to <log-dir>/mujoco_policy.mp4")
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=float, default=50.0)
    parser.add_argument("--video-codec", default="h264", choices=["h264", "mp4v"], help="mp4 codec; h264 uses ffmpeg/libx264 and is the most compatible")
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
    if args.policy_name:
        env["POLICY_NAME"] = args.policy_name
    if args.policy_onnx:
        env["POLICY_ONNX"] = str(Path(args.policy_onnx).expanduser())
    if args.scene:
        env["ROBOT_SCENE_NAME"] = args.scene
    env["USE_JOYSTICK"] = "0"
    env["UNITREE_MUJOCO_ROOT"] = str(UNITREE_MUJOCO_ROOT)
    env["INIT_DEFAULT_Q"] = "1"
    env["POLICY_WARMUP_SECONDS"] = str(max(0.0, args.policy_warmup))
    env["POLICY_FADE_SECONDS"] = str(max(0.0, args.policy_fade))
    env["POLICY_ACTION_CLIP"] = str(max(0.0, args.policy_action_clip))
    env["POLICY_TARGET_RATE_LIMIT"] = str(max(0.0, args.policy_target_rate_limit))
    env["POLICY_FALL_GUARD_GRAVITY_Z"] = str(args.policy_fall_guard_gravity_z)
    env["POLICY_GAIT_PERIOD"] = str(max(1e-3, args.gait_period))
    if args.safety_preset == "conservative":
        default_cmd_limits = (0.35, 0.15, 0.6)
        default_cmd_slew = (0.6, 0.35, 1.2)
    else:
        default_cmd_limits = (0.0, 0.0, 0.0)
        default_cmd_slew = (0.0, 0.0, 0.0)
    cmd_limit_vx = default_cmd_limits[0] if args.cmd_limit_vx is None else args.cmd_limit_vx
    cmd_limit_vy = default_cmd_limits[1] if args.cmd_limit_vy is None else args.cmd_limit_vy
    cmd_limit_yaw = default_cmd_limits[2] if args.cmd_limit_yaw is None else args.cmd_limit_yaw
    cmd_slew_vx = default_cmd_slew[0] if args.cmd_slew_vx is None else args.cmd_slew_vx
    cmd_slew_vy = default_cmd_slew[1] if args.cmd_slew_vy is None else args.cmd_slew_vy
    cmd_slew_yaw = default_cmd_slew[2] if args.cmd_slew_yaw is None else args.cmd_slew_yaw
    env["POLICY_CMD_LIMIT_VX"] = str(max(0.0, cmd_limit_vx))
    env["POLICY_CMD_LIMIT_VY"] = str(max(0.0, cmd_limit_vy))
    env["POLICY_CMD_LIMIT_YAW"] = str(max(0.0, cmd_limit_yaw))
    env["POLICY_CMD_SLEW_VX"] = str(max(0.0, cmd_slew_vx))
    env["POLICY_CMD_SLEW_VY"] = str(max(0.0, cmd_slew_vy))
    env["POLICY_CMD_SLEW_YAW"] = str(max(0.0, cmd_slew_yaw))
    env["POLICY_CMD_VX"] = str(args.cmd_vx)
    env["POLICY_CMD_VY"] = str(args.cmd_vy)
    env["POLICY_CMD_YAW"] = str(args.cmd_yaw)
    env["POLICY_CMD_START_TIME"] = str(max(0.0, args.cmd_start))
    env["POLICY_CMD_STOP_TIME"] = str(max(0.0, args.cmd_stop))
    env["POLICY_CMD_RAMP_TIME"] = str(max(0.0, args.cmd_ramp))
    env["POLICY_DIRECT_EVAL"] = "1" if args.direct_eval else "0"
    env["UNITREE_BRIDGE_ACTUATOR_MODE"] = args.bridge_actuator_mode
    env["MANUAL_CMD_VX_SCALE"] = str(max(0.0, args.manual_vx_scale))
    env["MANUAL_CMD_VY_SCALE"] = str(max(0.0, args.manual_vy_scale))
    env["MANUAL_CMD_YAW_SCALE"] = str(max(0.0, args.manual_yaw_scale))
    env["DISABLE_HANGING_EQUALITY"] = "0" if args.hanging else "1"
    video_path = Path(args.video_path).expanduser() if args.video_path else log_dir / "mujoco_policy.mp4"
    env["MUJOCO_RECORD_VIDEO"] = "1" if args.record_video else "0"
    env["MUJOCO_RECORD_VIDEO_PATH"] = str(video_path)
    env["MUJOCO_RECORD_VIDEO_WIDTH"] = str(max(16, args.video_width))
    env["MUJOCO_RECORD_VIDEO_HEIGHT"] = str(max(16, args.video_height))
    env["MUJOCO_RECORD_VIDEO_FPS"] = str(max(1.0, args.video_fps))
    env["MUJOCO_RECORD_VIDEO_CODEC"] = args.video_codec
    env["MUJOCO_SIM_DURATION_SECONDS"] = str(max(0.0, args.duration))

    sim_env = env.copy()
    policy_env = env.copy()
    if not args.viewer:
        sim_env["SDL_VIDEODRIVER"] = "dummy"
        sim_env["MUJOCO_HEADLESS"] = "1"
        sim_env.setdefault("MUJOCO_GL", "egl")
    else:
        sim_env.setdefault("MUJOCO_GL", "glfw")
    if not args.policy_window:
        policy_env["SDL_VIDEODRIVER"] = "dummy"

    sim_cmd = build_python_cmd(args.conda_env, str(sim_script))
    if args.scene:
        sim_cmd.extend(["--scene", args.scene])
    policy_cmd = build_python_cmd(args.conda_env, str(policy_script))

    if args.direct_eval:
        print("Policy:    direct MuJoCo eval (MJLab-style position actuators)")
    else:
        print(f"Policy:    {' '.join(policy_cmd)}")
    print(f"Simulator: {' '.join(sim_cmd)}")
    print(f"DDS:       domain={args.domain_id}, interface={args.interface}")
    print(f"Config:    {POLICY_ROOT / 'config.py'}")
    print(f"Logs:      {log_dir}")
    print(
        f"Safety:   preset={args.safety_preset} "
        f"cmd_limit=({cmd_limit_vx:g},{cmd_limit_vy:g},{cmd_limit_yaw:g}) "
        f"cmd_slew=({cmd_slew_vx:g},{cmd_slew_vy:g},{cmd_slew_yaw:g})"
    )
    if not args.direct_eval:
        print(f"Bridge:    actuator_mode={args.bridge_actuator_mode}")
    if args.policy_name:
        print(f"Policy ID: {args.policy_name}")
    if args.policy_onnx:
        print(f"ONNX:      {env['POLICY_ONNX']}")
    if args.record_video:
        print(f"Video:     {video_path}")

    with sim_log_path.open("w", encoding="utf-8") as sim_log, policy_log_path.open(
        "w", encoding="utf-8"
    ) as policy_log:
        policy_proc = None
        sim_proc = None
        if args.direct_eval:
            sim_proc = subprocess.Popen(
                sim_cmd,
                cwd=log_dir,
                env=sim_env,
                stdout=sim_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            _CHILD_PROCS.append((sim_proc, "sim"))
        else:
            policy_proc = subprocess.Popen(
                policy_cmd,
                cwd=log_dir,
                env=policy_env,
                stdout=policy_log,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
            _CHILD_PROCS.append((policy_proc, "policy"))
            time.sleep(args.startup_wait)
            if policy_proc.poll() is None:
                sim_proc = subprocess.Popen(
                    sim_cmd,
                    cwd=log_dir,
                    env=sim_env,
                    stdout=sim_log,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
                _CHILD_PROCS.append((sim_proc, "sim"))
            else:
                print("[policy] exited before simulator startup; simulator was not started.")

        try:
            deadline = None if args.duration <= 0.0 else time.monotonic() + args.duration
            while deadline is None or time.monotonic() < deadline:
                if policy_proc is not None and policy_proc.poll() is not None:
                    print("[policy] exited; stopping simulator...")
                    break
                if sim_proc is not None and sim_proc.poll() is not None:
                    print("[sim] exited; stopping policy...")
                    break
                time.sleep(0.2)
        finally:
            if sim_proc is not None:
                try:
                    sim_proc.wait(timeout=10 if (args.record_video or args.direct_eval) else 1)
                except subprocess.TimeoutExpired:
                    terminate(sim_proc, "sim")
            if policy_proc is not None:
                terminate(policy_proc, "policy")

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
