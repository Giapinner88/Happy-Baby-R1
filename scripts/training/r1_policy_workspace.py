#!/usr/bin/env python3
"""R1 training/export orchestration for both reference frameworks."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from shlex import join


ROOT = Path(__file__).resolve().parents[2]
MJLAB_ROOT = ROOT / "third_party" / "unitree_rl_mjlab"
RL_LAB_ROOT = ROOT / "third_party" / "unitree_rl_lab"
DATA_ROOT = ROOT / "data"
RUN_ROOT = DATA_ROOT / "runs"
POLICY_ROOT = DATA_ROOT / "policies"


@dataclass(frozen=True)
class Framework:
    name: str
    source_root: Path
    work_dir: Path
    train: list[str]
    play: list[str]
    task: str
    experiment: str


FRAMEWORKS = {
    "mjlab": Framework(
        name="mjlab",
        source_root=MJLAB_ROOT,
        work_dir=RUN_ROOT / "mjlab",
        train=[sys.executable, str(ROOT / "scripts" / "training" / "r1_mjlab_train.py")],
        play=[sys.executable, str(ROOT / "scripts" / "training" / "r1_mjlab_play.py")],
        task="Unitree-R1-Flat",
        experiment="r1_velocity",
    ),
    "rl_lab": Framework(
        name="rl_lab",
        source_root=RL_LAB_ROOT,
        work_dir=RUN_ROOT / "rl_lab",
        train=[sys.executable, str(ROOT / "scripts" / "training" / "r1_rl_lab_train.py")],
        play=[sys.executable, str(ROOT / "scripts" / "training" / "r1_rl_lab_play.py")],
        task="Unitree-R1-Velocity",
        experiment="r1_velocity",
    ),
}


def _run(cmd: list[str], framework: Framework, dry_run: bool) -> None:
    env = os.environ.copy()
    env.setdefault("HAPPY_BABY_R1_ROOT", str(ROOT))
    env.setdefault("PYTHONNOUSERSITE", "1")
    cache_root = DATA_ROOT / "cache"
    env.setdefault("XDG_CACHE_HOME", str(cache_root / "xdg"))
    env.setdefault("MPLCONFIGDIR", str(cache_root / "matplotlib"))
    env.setdefault("WARP_CACHE_PATH", str(cache_root / "warp"))
    env["PYTHONPATH"] = os.pathsep.join(
        p
        for p in (
            str(ROOT),
            str(framework.source_root),
            str(framework.source_root / "source" / "unitree_rl_lab"),
            env.get("PYTHONPATH", ""),
        )
        if p
    )
    print(join(cmd))
    if dry_run:
        return
    cache_root.mkdir(parents=True, exist_ok=True)
    framework.work_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(cmd, cwd=framework.work_dir, env=env, check=True)


def _latest_checkpoint(framework: Framework) -> Path | None:
    log_root = framework.work_dir / "logs" / "rsl_rl" / framework.experiment
    checkpoints = sorted(log_root.glob("*/model_*.pt"), key=lambda p: p.stat().st_mtime)
    return checkpoints[-1] if checkpoints else None


def _copy_if_exists(src: Path, dst: Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def collect(framework: Framework, checkpoint: Path | None = None) -> Path:
    checkpoint = checkpoint or _latest_checkpoint(framework)
    if checkpoint is None:
        raise SystemExit(f"No R1 checkpoint found for {framework.name}. Train first or pass --checkpoint.")
    checkpoint = checkpoint.expanduser().resolve()
    run_dir = checkpoint.parent
    out_dir = POLICY_ROOT / framework.name / run_dir.name
    out_dir.mkdir(parents=True, exist_ok=True)

    _copy_if_exists(checkpoint, out_dir / checkpoint.name)

    candidates = [
        run_dir / "policy.onnx",
        run_dir / "policy.onnx.data",
        run_dir / "exported" / "policy.onnx",
        run_dir / "exported" / "policy.onnx.data",
        run_dir / "exported" / "policy.pt",
    ]
    copied = [p.name for p in candidates if _copy_if_exists(p, out_dir / p.name)]
    if not copied:
        raise SystemExit(
            f"Checkpoint exists, but no exported policy was found next to it: {run_dir}. "
            "For mjlab, wait until a save_interval export exists. For rl_lab, run export."
        )
    print(f"Collected {framework.name} policy files into: {out_dir}")
    return out_dir


def cmd_status(_: argparse.Namespace) -> None:
    print(f"workspace: {ROOT}")
    print(f"r1 usd:    {ROOT / 'assets' / 'R1' / 'R1.usd'}")
    print(f"runs:      {RUN_ROOT}")
    print(f"policies:  {POLICY_ROOT}")
    for framework in FRAMEWORKS.values():
        checkpoint = _latest_checkpoint(framework)
        print(f"{framework.name}: task={framework.task} source={framework.source_root}")
        print(f"{framework.name}: work_dir={framework.work_dir}")
        print(f"{framework.name}: latest_checkpoint={checkpoint or 'none'}")


def cmd_train(args: argparse.Namespace) -> None:
    fw = FRAMEWORKS[args.framework]
    extra = getattr(args, "extra", [])
    if args.framework == "mjlab":
        if args.profile in {"flat_v2", "flat_v3", "flat_v4"}:
            if args.terrain != "flat":
                raise SystemExit(f"--profile {args.profile} is only defined for --terrain flat.")
            task = {
                "flat_v2": "Unitree-R1-FlatV2",
                "flat_v3": "Unitree-R1-FlatV3",
                "flat_v4": "Unitree-R1-FlatV4",
            }[args.profile]
        else:
            task = "Unitree-R1-Rough" if args.terrain == "rough" else "Unitree-R1-Flat"
        cmd = fw.train + [task]
        if args.num_envs is not None:
            cmd.append(f"--env.scene.num-envs={args.num_envs}")
        if args.max_iterations is not None:
            cmd.append(f"--agent.max-iterations={args.max_iterations}")
        if args.save_interval is not None:
            cmd.append(f"--agent.save-interval={args.save_interval}")
        if args.run_name:
            cmd.append(f"--agent.run-name={args.run_name}")
        if args.resume:
            cmd.append("--agent.resume=True")
        if args.load_run:
            cmd.append(f"--agent.load-run={args.load_run}")
        if args.load_checkpoint:
            cmd.append(f"--agent.load-checkpoint={args.load_checkpoint}")
        if args.video:
            cmd.append("--video=True")
        if args.video_length is not None:
            cmd.append(f"--video-length={args.video_length}")
        if args.video_interval is not None:
            cmd.append(f"--video-interval={args.video_interval}")
        if not any(arg == "--agent.logger" or arg.startswith("--agent.logger=") for arg in extra):
            cmd.append("--agent.logger=tensorboard")
    else:
        cmd = fw.train + ["--headless", "--task", fw.task]
        if args.num_envs is not None:
            cmd += ["--num_envs", str(args.num_envs)]
        if args.max_iterations is not None:
            cmd += ["--max_iterations", str(args.max_iterations)]
        if args.resume or args.load_run or args.load_checkpoint or args.video:
            print("[WARN] resume/video flags are currently wired for mjlab only.")
    cmd += extra
    _run(cmd, fw, args.dry_run)


def cmd_export(args: argparse.Namespace) -> None:
    fw = FRAMEWORKS[args.framework]
    checkpoint = Path(args.checkpoint).expanduser().resolve() if args.checkpoint else _latest_checkpoint(fw)
    if checkpoint is None:
        raise SystemExit(f"No checkpoint found for {args.framework}.")
    if args.framework == "rl_lab":
        cmd = fw.play + [
            "--headless",
            "--task",
            fw.task,
            "--checkpoint",
            str(checkpoint),
            "--video",
            "--video_length",
            "1",
        ]
        _run(cmd, fw, args.dry_run)
        if args.dry_run:
            return
    elif args.dry_run:
        print(f"collect exported mjlab policy beside {checkpoint}")
        return
    collect(fw, checkpoint)


def cmd_collect(args: argparse.Namespace) -> None:
    fw = FRAMEWORKS[args.framework]
    checkpoint = Path(args.checkpoint).expanduser().resolve() if args.checkpoint else None
    collect(fw, checkpoint)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(required=True)

    status = sub.add_parser("status", help="Show R1 assets, tasks, and latest checkpoints.")
    status.set_defaults(func=cmd_status)

    train = sub.add_parser("train", help="Train R1 in one framework.")
    train.add_argument("framework", choices=FRAMEWORKS)
    train.add_argument("--terrain", choices=["flat", "rough"], default="flat")
    train.add_argument("--profile", choices=["default", "flat_v2", "flat_v3", "flat_v4"], default="default")
    train.add_argument("--num-envs", type=int)
    train.add_argument("--max-iterations", type=int)
    train.add_argument("--save-interval", type=int, help="Save/export checkpoint every N iterations.")
    train.add_argument("--run-name")
    train.add_argument("--resume", action="store_true", help="Resume from a matching existing checkpoint.")
    train.add_argument("--load-run", help="Run regex/name used with --resume, e.g. '.*r1_flat_walk_v2'.")
    train.add_argument("--load-checkpoint", help="Checkpoint regex/name used with --resume, e.g. 'model_.*.pt'.")
    train.add_argument("--video", action="store_true", help="Record short train videos into the run directory.")
    train.add_argument("--video-length", type=int, help="Video length in env steps; 150 ~= 3 seconds at 50 Hz.")
    train.add_argument("--video-interval", type=int, help="Record one video every N env steps.")
    train.add_argument("--dry-run", action="store_true")
    train.set_defaults(func=cmd_train)

    export = sub.add_parser("export", help="Export and collect policy files for one framework.")
    export.add_argument("framework", choices=FRAMEWORKS)
    export.add_argument("--checkpoint")
    export.add_argument("--dry-run", action="store_true")
    export.set_defaults(func=cmd_export)

    collect_cmd = sub.add_parser("collect", help="Collect existing exported policy files.")
    collect_cmd.add_argument("framework", choices=FRAMEWORKS)
    collect_cmd.add_argument("--checkpoint")
    collect_cmd.set_defaults(func=cmd_collect)

    return parser


def main() -> None:
    args, extra = build_parser().parse_known_args()
    args.extra = extra
    args.func(args)


if __name__ == "__main__":
    main()
