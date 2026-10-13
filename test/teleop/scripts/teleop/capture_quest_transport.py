#!/usr/bin/env python3
"""Capture Quest transport telemetry without emitting any R1 command.

This T001-A helper starts only the vendored TeleVuer HTTPS endpoint and records
the vendor wrapper's pose/controller samples. It does not import project R1
mapping, simulator, DDS, ROS, Unitree SDK, or hardware code.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
TELEVUER_SOURCE = ROOT / "third_party" / "xr_teleoperate" / "teleop" / "televuer" / "src"
sys.path.insert(0, str(ROOT))

from evidence.run_id import allocate_run_id  # noqa: E402
from evidence.writer import (  # noqa: E402
    write_evidence_completeness,
    write_experiment_config,
    write_metadata,
    write_resolved_config,
    write_runner_command,
    write_status,
)


T001_A_RUN_ROOT = ROOT / "experiments" / "r1_teleop" / "quest3_sim_v1" / "T001" / "runs"
T001_A_PROTOCOL = "t001_a"


def _default_output_dir() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return ROOT / "results" / "smoke" / "r1_teleop_transport" / stamp


def _allocated_output_dir() -> Path:
    """A fresh T001-A evidence directory, so no operator has to invent an id."""

    T001_A_RUN_ROOT.mkdir(parents=True, exist_ok=True)
    return T001_A_RUN_ROOT / allocate_run_id(T001_A_RUN_ROOT, T001_A_PROTOCOL)


def _sample(telemetry: object, input_mode: str) -> dict[str, object]:
    result: dict[str, object] = {
        "timestamp_monotonic_s": time.monotonic(),
        "motion_data_ready": bool(telemetry.motion_data_ready),
        "head_pose_matrix": telemetry.head_pose.tolist(),
        "left_wrist_pose_matrix": telemetry.left_wrist_pose.tolist(),
        "right_wrist_pose_matrix": telemetry.right_wrist_pose.tolist(),
    }
    if input_mode == "controller":
        result["controller"] = {
            "left": {
                "trigger": telemetry.left_ctrl_trigger,
                "trigger_value": telemetry.left_ctrl_triggerValue,
                "squeeze": telemetry.left_ctrl_squeeze,
                "squeeze_value": telemetry.left_ctrl_squeezeValue,
                "thumbstick": telemetry.left_ctrl_thumbstick,
                "thumbstick_value": telemetry.left_ctrl_thumbstickValue.tolist(),
                "a_button": telemetry.left_ctrl_aButton,
                "b_button": telemetry.left_ctrl_bButton,
            },
            "right": {
                "trigger": telemetry.right_ctrl_trigger,
                "trigger_value": telemetry.right_ctrl_triggerValue,
                "squeeze": telemetry.right_ctrl_squeeze,
                "squeeze_value": telemetry.right_ctrl_squeezeValue,
                "thumbstick": telemetry.right_ctrl_thumbstick,
                "thumbstick_value": telemetry.right_ctrl_thumbstickValue.tolist(),
                "a_button": telemetry.right_ctrl_aButton,
                "b_button": telemetry.right_ctrl_bButton,
            },
        }
    return result


def _poses_are_finite(sample: dict[str, object]) -> bool:
    for field in ("head_pose_matrix", "left_wrist_pose_matrix", "right_wrist_pose_matrix"):
        matrix = sample[field]
        if not all(math.isfinite(float(value)) for row in matrix for value in row):
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="New evidence directory; never overwritten. Omit it and use --evidence to allocate one.",
    )
    parser.add_argument(
        "--evidence",
        action="store_true",
        help="Allocate a fresh t001_a_<UTC> run directory under the experiment instead of a disposable smoke path.",
    )
    parser.add_argument("--duration-s", type=float, default=120.0, help="Capture duration after the server starts.")
    parser.add_argument("--frequency-hz", type=float, default=30.0, help="Telemetry sampling rate.")
    parser.add_argument("--input-mode", choices=("controller", "hand"), default="controller")
    parser.add_argument("--host-ip", required=True, help="Host IP shown in the Quest connection URL.")
    parser.add_argument("--cert-file", type=Path, help="HTTPS certificate for the Quest endpoint.")
    parser.add_argument("--key-file", type=Path, help="Private key paired with --cert-file.")
    args = parser.parse_args()
    if args.duration_s <= 0.0 or args.frequency_hz <= 0.0:
        raise SystemExit("--duration-s and --frequency-hz must be positive.")
    if (args.cert_file is None) != (args.key_file is None):
        raise SystemExit("Specify both --cert-file and --key-file, or neither.")
    if args.cert_file is not None and (not args.cert_file.is_file() or not args.key_file.is_file()):
        raise SystemExit("--cert-file and --key-file must both exist as regular files.")

    if args.output_dir is not None and args.evidence:
        raise SystemExit("Use either --output-dir or --evidence, not both.")
    if args.output_dir is not None:
        output_dir = args.output_dir
    elif args.evidence:
        output_dir = _allocated_output_dir()
    else:
        output_dir = _default_output_dir()
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SystemExit(f"Refusing to overwrite transport capture: {output_dir}")
    output_dir.mkdir(parents=True)
    config = {
        "mode": "capture_only",
        "input_mode": args.input_mode,
        "duration_s": args.duration_s,
        "frequency_hz": args.frequency_hz,
        "host_ip": args.host_ip,
        "vuer_url": f"https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012",
        "cert_file": str(args.cert_file.resolve()) if args.cert_file else "TeleVuer default resolution",
        "r1_command_emitted": False,
        "simulator_sink_called": False,
        "dds_or_hardware_called": False,
    }
    write_resolved_config(output_dir, config)
    write_experiment_config(output_dir, config)
    write_runner_command(output_dir)
    write_metadata(output_dir, ROOT, {"televuer_source": str(TELEVUER_SOURCE)})

    sys.path.insert(0, str(TELEVUER_SOURCE))
    try:
        from televuer import TeleVuerWrapper
    except ImportError as exc:
        write_status(output_dir, "failed", reason=f"Cannot import TeleVuer: {exc}")
        raise SystemExit(f"Cannot import TeleVuer in this environment: {exc}") from exc

    wrapper = TeleVuerWrapper(
        use_hand_tracking=args.input_mode == "hand",
        binocular=True,
        img_shape=(480, 1280),
        display_fps=args.frequency_hz,
        display_mode="pass-through",
        zmq=False,
        webrtc=False,
        cert_file=str(args.cert_file.resolve()) if args.cert_file else None,
        key_file=str(args.key_file.resolve()) if args.key_file else None,
    )
    print(f"Quest capture endpoint: {config['vuer_url']}", flush=True)
    print("Open this URL in Quest Browser, accept the local certificate if prompted, then move the headset/controllers.", flush=True)

    samples: list[dict[str, object]] = []
    period_s = 1.0 / args.frequency_hz
    deadline = time.monotonic() + args.duration_s
    try:
        while time.monotonic() < deadline:
            samples.append(_sample(wrapper.get_tele_data(), args.input_mode))
            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        wrapper.close()

    (output_dir / "transport_samples.jsonl").write_text(
        "".join(json.dumps(sample, sort_keys=True) + "\n" for sample in samples), encoding="utf-8"
    )
    ready_count = sum(1 for sample in samples if sample["motion_data_ready"])
    ready_finite_count = sum(
        1 for sample in samples if sample["motion_data_ready"] and _poses_are_finite(sample)
    )
    write_evidence_completeness(
        output_dir,
        {
            "transport_samples": (output_dir / "transport_samples.jsonl").is_file(),
            "sample_count": len(samples),
            "video": False,
            "video_reason": "T001-A is capture-only: it records transport telemetry and runs no simulator, so there is nothing to render.",
        },
    )
    status_path = write_status(
        output_dir,
        "completed",
        scientific_outcome="transport_data_received" if ready_count else "no_transport_data_received",
        extra={
            "sample_count": len(samples),
            "motion_ready_sample_count": ready_count,
            "motion_ready_finite_pose_sample_count": ready_finite_count,
            "r1_command_emitted": False,
            "simulator_sink_called": False,
        },
    )
    print(status_path.read_text(encoding="utf-8").strip(), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
