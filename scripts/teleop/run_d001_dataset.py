#!/usr/bin/env python3
"""Thu dataset R1 từ một profile D001/D002 — một lệnh, một run id.

Đọc ``experiment_id`` và ``method_record`` từ profile để cấp run id vào đúng
experiment, rồi chạy simulator. Hai nguồn lệnh:

    --replay-run <run-id-hoặc-đường-dẫn>   phát lại một phiên T007 đã ghi
    --host-ip <ip>                          phiên Quest trực tiếp

Đường replay không cần kính và là cách tích luỹ episode nhanh nhất từ dữ liệu
đã có. Đường trực tiếp đi qua đúng launcher mà T007 dùng, nên bridge, stop file
và log kết nối hoạt động y hệt.

Đây là đường simulation-only: root cố định, không DDS, không phần cứng.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from evidence.run_id import allocate_run_id  # noqa: E402
from teleop.r1.launcher import PilotLaunchSpec, SIM_ENV, run_pilot  # noqa: E402


EXPERIMENT_ROOT = ROOT / "experiments" / "r1_dataset" / "quest3_sim_v1" / "D001"
DATASET_CONFIG = EXPERIMENT_ROOT / "config" / "r1_d001_reach_point_dataset.json"
T007_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T007"
T007_PROFILE = T007_ROOT / "config" / "r1_t007_whole_upper_body_live.json"
UPSTREAM_PROFILE = T007_ROOT / "config" / "r1_t007_upstream_stream_live.json"
UPSTREAM_SOLVER_ARGS = ["scripts/teleop/run_r1_upstream_ik_stream.py", "--passthrough"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--replay-run",
        help="Run id T007 hoặc đường dẫn tới raw_commands.jsonl. Không cần Quest.",
    )
    source.add_argument("--host-ip", help="Chạy phiên Quest trực tiếp với IP này.")
    parser.add_argument("--duration-s", type=float, default=13000.0)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--physics-hz", type=float, default=200.0)
    parser.add_argument("--device", default="cuda:0", help="USDRT chỉ hỗ trợ cuda:0.")
    parser.add_argument(
        "--rendering-mode",
        choices=("performance", "balanced", "quality"),
        default="performance",
        help=(
            "Preset RTX của Isaac Lab. Thu dataset mặc định dùng performance để "
            "giảm chi phí sim.render; quality chỉ phù hợp tạo ảnh ngoại tuyến."
        ),
    )
    parser.add_argument("--dataset-config", type=Path, default=DATASET_CONFIG)
    parser.add_argument("--teleop-profile", type=Path, default=T007_PROFILE)
    parser.add_argument(
        "--solver",
        choices=("local", "upstream"),
        default="local",
        help=(
            "'local' chạy iterative pose IK trong Isaac; 'upstream' chèn process "
            "xr_teleoperate R1_A5_ArmIK và để Isaac chỉ áp góc khớp."
        ),
    )
    parser.add_argument(
        "--upstream-stream-profile",
        type=Path,
        default=UPSTREAM_PROFILE,
        help="Profile áp joint stream khi --solver upstream.",
    )
    parser.add_argument(
        "--cert-file", type=Path, default=Path.home() / ".config/xr_teleoperate/t001_10_42/cert.pem"
    )
    parser.add_argument(
        "--key-file", type=Path, default=Path.home() / ".config/xr_teleoperate/t001_10_42/key.pem"
    )
    parser.add_argument(
        "--head-view-port",
        type=int,
        default=0,
        help=(
            "Cổng ZMQ đưa góc nhìn robot vào kính ở phiên trực tiếp. MẶC ĐỊNH TẮT. "
            "Bật nó chuyển TeleVuer sang chế độ 'ego', và ở chế độ đó vendor phát JPEG "
            "480x1280 lên kính ở 30 fps NGAY TỪ KHI vào phiên — kể cả khi chưa có khung "
            "nào, vì bộ nhớ chia sẻ khởi tạo bằng số không. Trong cửa sổ launcher chờ "
            "Enter VR thì Isaac chưa chạy, nên luồng ảnh rỗng đó làm vỡ luồng ghi SSL "
            "của aiohttp (AssertionError trong resume_writing) và kéo sập phiên WebXR: "
            "phiên d001_20260906T044656Z chạy 3 phút 31 giây, rơi toàn bộ 6299 mẫu, "
            "phát ra 0 lệnh. Chỉ bật khi đang gỡ đúng vấn đề này."
        ),
    )
    parser.add_argument(
        "--target-digit-priority",
        help="Chuyển thẳng xuống simulator: chữ số cần bù trước trong phiên này.",
    )
    parser.add_argument("--gui", action="store_true", help="Hiện cửa sổ Isaac thay vì chạy ẩn.")
    parser.add_argument(
        "--allow-low-camera-fps",
        action="store_true",
        help=(
            "Chỉ dùng chẩn đoán: không dừng sớm khi camera không đạt gate FPS. "
            "Episode vẫn bị đánh dấu rejected theo profile."
        ),
    )
    parser.add_argument(
        "--viewport-camera",
        choices=("perspective", "head"),
        default="head",
        help=(
            "Chỉ dùng cùng --gui. 'head' cho cửa sổ Isaac hiện góc nhìn thứ nhất của robot; "
            "'perspective' giữ góc nhìn thứ ba nhìn toàn thân."
        ),
    )
    parser.add_argument("--dry-run", action="store_true", help="In đường dẫn và lệnh, không chạy gì.")
    return parser


def resolve_replay(value: str) -> Path:
    """Chấp nhận cả run id T007 lẫn đường dẫn thẳng tới raw_commands.jsonl."""

    candidate = Path(value).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    stream = T007_ROOT / "runs" / value / "raw_commands.jsonl"
    if stream.is_file():
        return stream.resolve()
    raise SystemExit(
        f"Không tìm thấy luồng lệnh cho {value!r}. Truyền một run id dưới {T007_ROOT / 'runs'} "
        "hoặc đường dẫn tới raw_commands.jsonl."
    )


def resolve_experiment(dataset_config: Path) -> tuple[str, Path]:
    """Lấy protocol và run root từ chính profile, tránh ghi D002 vào D001."""

    resolved = dataset_config.expanduser().resolve()
    try:
        payload = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Không đọc được cấu hình dataset {resolved}: {exc}") from exc
    protocol = str(payload.get("experiment_id") or "").strip().lower()
    method_record = payload.get("method_record")
    if not protocol or not method_record:
        raise SystemExit("Cấu hình dataset phải khai experiment_id và method_record.")
    record_path = Path(str(method_record))
    if not record_path.is_absolute():
        record_path = ROOT / record_path
    experiment_root = record_path.expanduser().resolve().parent
    return protocol, experiment_root / "runs"


def main() -> int:
    args = build_parser().parse_args()
    controller_profile = (
        args.upstream_stream_profile if args.solver == "upstream" else args.teleop_profile
    )
    for path in (args.dataset_config, controller_profile):
        if not path.expanduser().is_file():
            raise SystemExit(f"Không tìm thấy cấu hình: {path}")

    protocol, run_root = resolve_experiment(args.dataset_config)
    run_root.mkdir(parents=True, exist_ok=True)

    controller_args = (
        ["--upstream-joint-stream-config", str(controller_profile)]
        if args.solver == "upstream"
        else ["--whole-upper-body-config", str(controller_profile)]
    )
    sim_args = [
        *controller_args,
        "--dataset-scene-config", str(args.dataset_config),
        "--duration-s", str(args.duration_s),
        "--control-hz", str(args.control_hz),
        "--physics-hz", str(args.physics_hz),
        "--device", args.device,
        "--rendering_mode", args.rendering_mode,
        "--disable-self-collisions",
        "--no-video",
    ]
    if not args.allow_low_camera_fps:
        sim_args.append("--strict-dataset-fps")
    if args.target_digit_priority:
        sim_args += ["--target-digit-priority", args.target_digit_priority]
    if args.gui:
        sim_args += ["--viewport-camera", args.viewport_camera]
    else:
        sim_args.append("--headless")

    print(f"Protocol:     {protocol}", file=sys.stderr)

    if args.replay_run is not None:
        if args.solver == "upstream":
            raise SystemExit(
                "--solver upstream hiện dành cho phiên Quest trực tiếp. Replay cần một "
                "joint stream đã giải sẵn; dùng --solver local cho đường replay."
            )
        # Đường replay không có launcher, nên run id được cấp ở đây. Đường trực
        # tiếp thì `run_pilot` tự cấp — cấp thêm một cái nữa ở đây sẽ tạo ra một
        # id in ra màn hình mà không run nào dùng tới.
        run_id = allocate_run_id(run_root, protocol)
        output_dir = run_root / run_id
        stream = resolve_replay(args.replay_run)
        print(f"Run id:       {run_id}", file=sys.stderr)
        print(f"Evidence dir: {output_dir}", file=sys.stderr)
        print(f"Nguồn lệnh:   replay {stream}", file=sys.stderr)
        command = [
            "conda", "run", "--no-capture-output", "-n", SIM_ENV,
            "python", "scripts/teleop/run_r1_quest3_live.py",
            "--replay-command-file", str(stream),
            "--output-dir", str(output_dir),
            *sim_args,
        ]
        if args.dry_run:
            print("sim: " + " ".join(command), file=sys.stderr)
            return 0
        return subprocess.run(command, cwd=ROOT).returncode

    # Phiên trực tiếp: dùng đúng launcher của T007 để bridge, stop file và log
    # kết nối hoạt động y hệt. Chỉ khác các cờ truyền cho simulator.
    print(f"Nguồn lệnh:   Quest trực tiếp tại {args.host_ip}", file=sys.stderr)
    return run_pilot(
        PilotLaunchSpec(
            protocol=protocol,
            run_root=run_root,
            repo_root=ROOT,
            host_ip=args.host_ip,
            duration_s=args.duration_s,
            cert_file=args.cert_file,
            key_file=args.key_file,
            physics_hz=args.physics_hz,
            control_hz=args.control_hz,
            trigger_value_threshold=5.0,
            stop_file_dir=Path("/tmp"),
            disable_self_collisions=True,
            extra_sim_args=[
                *controller_args,
                "--dataset-scene-config", str(args.dataset_config),
                "--device", args.device,
                "--rendering_mode", args.rendering_mode,
                "--no-video",
                *([] if args.allow_low_camera_fps else ["--strict-dataset-fps"]),
                *(["--viewport-camera", args.viewport_camera] if args.gui else ["--headless"]),
                *(["--target-digit-priority", args.target_digit_priority]
                  if args.target_digit_priority else []),
            ],
            idle_stop_s=0.0,
            quest_ready_timeout_s=180.0,
            head_view_port=args.head_view_port or None,
            solver_args=(UPSTREAM_SOLVER_ARGS if args.solver == "upstream" else None),
            solver_stats_filename=(
                "upstream_solver_stats.json" if args.solver == "upstream" else None
            ),
        ),
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    raise SystemExit(main())
