#!/usr/bin/env python3
"""Lái R1 trong Isaac bằng một policy LeRobot đã huấn luyện.

Chỗ đứng của tiến trình này trong đường ống giống hệt chỗ của solver upstream
trong T007: nó đọc quan sát, sinh vector 12 khớp, và ghi ra stdout những dòng
JSON mà `run_r1_quest3_live.py` đã biết đọc ở chế độ upstream stream. Nhờ vậy
phía Isaac không cần biết vector đến từ IK của vendor hay từ một mạng nơ-ron.

Khác một điểm so với solver: quan sát phải đi ngược từ simulator ra, nên tiến
trình này còn là subscriber của `policy_obs_stream`.

    Isaac ──ZMQ 5557 (ảnh + state + prompt)──> tiến trình này
      ^                                              │
      └────────── stdin: JSON joint targets ─────────┘

Hai nhịp tách nhau và đó là cố ý. Policy chạy ở nhịp nó được HỌC (8 Hz cho
D002); lệnh thì phát ở nhịp control của Isaac, lặp lại vector mới nhất giữa hai
lần suy luận. Chạy policy ở 30 Hz sẽ đưa cho nó một phân bố thời gian mà nó
chưa từng thấy; còn để stdin im lặng giữa hai lần suy luận thì Isaac chỉ giữ
vector cũ — cùng kết quả nhưng không đo được, nên thà phát tường minh.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

# Thứ tự này phải khớp `meta/r1_conversion.json` của dataset đã train, và khớp
# `controlled_joint_names` của phía Isaac. Lệch một chỗ là tay trái nhận lệnh
# của tay phải, và không có gì trong đường ống báo lỗi đó.
JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "head_pitch_joint",
    "head_yaw_joint",
)

IDENTITY_POSE = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--dataset-repo-id", default="local/r1_d002_8fps")
    parser.add_argument("--obs-port", type=int, default=5557)
    parser.add_argument("--policy-hz", type=float, default=8.0)
    parser.add_argument("--command-hz", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--stats-path", type=Path)
    parser.add_argument(
        "--warmup-timeout-s",
        type=float,
        default=180.0,
        help="Thời gian chờ quan sát ĐẦU TIÊN. Isaac mất hàng chục giây để nạp cảnh.",
    )
    args = parser.parse_args()
    for name in ("policy_hz", "command_hz", "duration_s"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} phải dương")
    return args


def log(message: str) -> None:
    """Nhật ký đi ra stderr. stdout là kênh lệnh và chỉ được chở JSON."""

    print(f"[policy-rollout] {message}", file=sys.stderr, flush=True)


def build_policy(args: argparse.Namespace):
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    from lerobot.policies import make_policy, make_pre_post_processors

    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_dir():
        raise SystemExit(f"không tìm thấy checkpoint: {checkpoint}")

    policy_cfg = PreTrainedConfig.from_pretrained(checkpoint)
    policy_cfg.pretrained_path = str(checkpoint)
    policy_cfg.device = args.device

    ds_meta = LeRobotDatasetMetadata(
        args.dataset_repo_id, root=str(args.dataset_root.expanduser().resolve())
    )
    policy = make_policy(cfg=policy_cfg, ds_meta=ds_meta)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": args.device}},
    )
    log(f"đã nạp policy {policy_cfg.type} từ {checkpoint}")
    log(f"dataset fps={ds_meta.fps} episodes={ds_meta.total_episodes}")
    return policy, preprocessor, postprocessor, torch


def observation_batch(rgb, state, task: str, torch):
    """Đóng gói quan sát theo đúng quy ước của LeRobot.

    Ảnh phải là channel-first float32 trong [0,1]; state là (1, D). Đây là quy
    ước mà `envs/utils.preprocess_observation` áp cho mọi policy, nên tự dựng ở
    đây phải dựng y hệt.
    """

    import numpy as np

    image = torch.from_numpy(np.ascontiguousarray(rgb))
    image = image.permute(2, 0, 1).unsqueeze(0).to(torch.float32) / 255.0
    return {
        "observation.images.head_camera": image,
        "observation.state": torch.from_numpy(np.asarray(state, dtype=np.float32)).unsqueeze(0),
        "task": [task],
    }


def emit(sequence_id: int, joint_positions) -> None:
    line = {
        "schema_version": 1,
        "sequence_id": sequence_id,
        "timestamp_monotonic_s": time.monotonic(),
        "deadman_enabled": True,
        "head_pose": IDENTITY_POSE,
        "left_wrist_pose": IDENTITY_POSE,
        "right_wrist_pose": IDENTITY_POSE,
        "base_velocity": {"vx_mps": 0.0, "vy_mps": 0.0, "yaw_rate_radps": 0.0},
        "reset_requested": False,
        "upstream_joint_names": list(JOINT_NAMES),
        "upstream_joint_position_rad": [float(v) for v in joint_positions],
    }
    sys.stdout.write(json.dumps(line) + "\n")
    sys.stdout.flush()


class DownstreamClosed(Exception):
    """Isaac đã đóng stdin. Không phải lỗi: phía kia hết thời lượng trước."""


def main() -> int:
    args = parse_args()
    from teleop.r1.policy_obs_stream import PolicyObsSubscriber

    policy, preprocessor, postprocessor, torch = build_policy(args)
    subscriber = PolicyObsSubscriber(port=args.obs_port)
    log(f"chờ quan sát trên tcp://127.0.0.1:{args.obs_port}")

    deadline = time.monotonic() + args.warmup_timeout_s
    observation = None
    while observation is None and time.monotonic() < deadline:
        observation = subscriber.latest()
        if observation is None:
            time.sleep(0.02)
    if observation is None:
        log("FAIL: không nhận được quan sát nào; Isaac có bật --policy-obs-port không?")
        subscriber.close()
        return 2
    log("đã nhận quan sát đầu tiên; bắt đầu lái")

    command_period = 1.0 / args.command_hz
    policy_period = 1.0 / args.policy_hz
    start = time.monotonic()
    next_command = start
    next_policy = start
    sequence_id = 0
    inference_count = 0
    inference_times: list[float] = []
    last_action = None
    tasks_seen: dict[str, int] = {}

    try:
        while True:
            now = time.monotonic()
            if now - start >= args.duration_s:
                break
            if now >= next_policy:
                next_policy += policy_period
                fresh = subscriber.latest()
                if fresh is not None:
                    observation = fresh
                rgb, state, task = observation
                tasks_seen[task] = tasks_seen.get(task, 0) + 1
                began = time.monotonic()
                batch = observation_batch(rgb, state, task, torch)
                batch = preprocessor(batch)
                with torch.inference_mode():
                    action = policy.select_action(batch)
                action = postprocessor(action)
                last_action = action.squeeze(0).detach().float().cpu().numpy()
                inference_times.append(time.monotonic() - began)
                inference_count += 1
            if last_action is not None and now >= next_command:
                next_command += command_period
                try:
                    emit(sequence_id, last_action)
                except (BrokenPipeError, OSError):
                    log("Isaac đã đóng stdin; kết thúc")
                    break
                sequence_id += 1
            time.sleep(0.001)
    except KeyboardInterrupt:
        log("dừng theo yêu cầu")
    finally:
        stats = {
            "schema": "happy_baby_r1.policy_rollout_stats",
            "schema_version": 1,
            "checkpoint": str(args.checkpoint),
            "policy_hz_requested": args.policy_hz,
            "command_hz_requested": args.command_hz,
            "inference_count": inference_count,
            "commands_emitted": sequence_id,
            "wall_time_s": round(time.monotonic() - start, 3),
            "achieved_policy_hz": (
                round(inference_count / (time.monotonic() - start), 3)
                if time.monotonic() > start
                else None
            ),
            "inference_s": (
                {
                    "count": len(inference_times),
                    "mean": round(sum(inference_times) / len(inference_times), 6),
                    "max": round(max(inference_times), 6),
                }
                if inference_times
                else None
            ),
            "tasks_seen": tasks_seen,
            **subscriber.stats(),
        }
        if args.stats_path is not None:
            args.stats_path.expanduser().parent.mkdir(parents=True, exist_ok=True)
            args.stats_path.expanduser().write_text(
                json.dumps(stats, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        log(json.dumps(stats, ensure_ascii=False))
        subscriber.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
