#!/usr/bin/env python3
"""Convert fresh vendor R1-A5 solutions to the existing HB sidecar wire order.

This workstation process has no robot transport. It never solves IK or invents
targets when the Quest source stalls. The sidecar remains the sole session
anchor and the high-level process remains the sole motor publisher. The
``--joint-limits-only`` mode is an explicit hardware-test option: it keeps the
URDF position clamp and fail-closed input checks while bypassing the additional
velocity/acceleration follower.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import queue
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from teleop.r1.rate_limit import OnlineJointLimiter  # noqa: E402

MODEL_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "head_pitch_joint", "head_yaw_joint",
)
WIRE_NAMES = MODEL_NAMES[:10] + ("head_yaw_joint", "head_pitch_joint")
SOLVER_ID = "upstream_xr_teleoperate_R1_A5_ArmIK"


def joint_limits(path: Path) -> tuple[np.ndarray, np.ndarray]:
    root = ET.parse(path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    lower, upper = [], []
    for name in MODEL_NAMES:
        limit = joints[name].find("limit")
        if limit is None:
            raise ValueError(f"missing limit for {name}")
        lower.append(float(limit.attrib["lower"]))
        upper.append(float(limit.attrib["upper"]))
    return np.asarray(lower), np.asarray(upper)


def parse_solution(payload: dict) -> np.ndarray:
    if payload.get("upstream_solver") != SOLVER_ID:
        raise ValueError("wrong vendor solver")
    if tuple(payload.get("upstream_joint_names", ())) != MODEL_NAMES:
        raise ValueError("wrong vendor joint order")
    values = np.asarray(payload.get("upstream_joint_position_rad"), dtype=float)
    if values.shape != (12,) or not np.all(np.isfinite(values)):
        raise ValueError("vendor solution must contain 12 finite joints")
    return values


def reader(lines: "queue.Queue[str | None]") -> None:
    for line in sys.stdin:
        if line.strip():
            lines.put(line)
    lines.put(None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=120.0)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--source-timeout-s", type=float, default=0.3)
    parser.add_argument("--release-debounce-s", type=float, default=5.0)
    parser.add_argument("--stream-contract", choices=("legacy", "h4"), default="legacy")
    parser.add_argument("--max-joint-velocity-rad-s", type=float, default=0.5)
    parser.add_argument("--max-joint-acceleration-rad-s2", type=float, default=1.0)
    parser.add_argument(
        "--joint-limits-only", action="store_true",
        help="Emit URDF-clamped targets directly; skip the velocity/acceleration limiter.",
    )
    parser.add_argument("--diagnostics-log", type=Path)
    parser.add_argument("--preflight", action="store_true", help="Validate HB URDF contract and exit.")
    args = parser.parse_args()
    if (not math.isfinite(args.duration_s) or args.duration_s < 0 or
            not math.isfinite(args.release_debounce_s) or args.release_debounce_s < 0 or
            not all(math.isfinite(value) and value > 0 for value in (
                args.control_hz, args.source_timeout_s,
                args.max_joint_velocity_rad_s, args.max_joint_acceleration_rad_s2,
            ))):
        raise SystemExit("duration/release debounce must be non-negative; other target timing must be positive")
    hb_urdf = ROOT / "src" / "assets" / "R1.urdf"
    contract = json.loads((ROOT / "config" / "vendor_r1_a5_845b25b.json").read_text(encoding="utf-8"))
    if hashlib.sha256(hb_urdf.read_bytes()).hexdigest() != contract["hb_urdf_sha256"]:
        raise SystemExit("HB R1 URDF changed; recheck vendor-to-HB FK before running")
    lower, upper = joint_limits(hb_urdf)
    if args.preflight:
        print("[VENDOR TARGET PREFLIGHT] HB R1 URDF pinned", file=sys.stderr, flush=True)
        return 0
    limiter = OnlineJointLimiter(
        max_velocity_rad_s=args.max_joint_velocity_rad_s,
        max_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
        dt_s=1.0 / args.control_hz,
        lower_limits=lower,
        upper_limits=upper,
    )
    if args.diagnostics_log:
        args.diagnostics_log.parent.mkdir(parents=True, exist_ok=True)
    log = args.diagnostics_log.open("w", encoding="utf-8") if args.diagnostics_log else None

    def event(name: str, **details: object) -> None:
        record = {"event": name, "monotonic_s": time.monotonic(), **details}
        if log:
            log.write(json.dumps(record, separators=(",", ":")) + "\n")
            log.flush()
        if name != "target_emitted":
            if name in {"source_rejected", "stop"}:
                prefix = "ERROR " if name == "source_rejected" else ""
                detail = details.get("reason", "unknown")
                print(f"[TARGET] {prefix}{name}: {detail}", file=sys.stderr, flush=True)
            else:
                print(f"[TARGET] {name}", file=sys.stderr, flush=True)

    lines: "queue.Queue[str | None]" = queue.Queue()
    threading.Thread(target=reader, args=(lines,), daemon=True).start()
    started = time.monotonic()
    period = 1.0 / args.control_hz
    last_source_at: float | None = None
    last_sequence = -1
    output_sequence = 0
    release_since: float | None = None
    stopped = False
    try:
        while args.duration_s == 0 or time.monotonic() - started < args.duration_s:
            tick = time.monotonic()
            newest: dict | None = None
            closed = False
            while True:
                try:
                    line = lines.get_nowait()
                except queue.Empty:
                    break
                if line is None:
                    closed = True
                    break
                try:
                    payload = json.loads(line)
                    sequence = payload["sequence_id"]
                    source_at = float(payload["timestamp_monotonic_s"])
                    if type(sequence) is not int or sequence <= last_sequence:
                        raise ValueError("non-increasing source sequence")
                    if not math.isfinite(source_at) or source_at > time.monotonic() + 0.05:
                        raise ValueError("invalid source timestamp")
                    last_sequence = sequence
                    newest = payload
                    last_source_at = time.monotonic()
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    event("source_rejected", reason=str(exc))
                    stopped = True
                    return 2
            if closed:
                event("stop", reason="upstream_stream_closed", last_sequence=last_sequence)
                stopped = True
                return 0
            now = time.monotonic()
            if last_source_at is not None and now - last_source_at > args.source_timeout_s:
                event("stop", reason="source_watchdog", last_sequence=last_sequence)
                stopped = True
                return 2
            if newest is not None:
                enabled = newest.get("deadman_enabled")
                if type(enabled) is not bool:
                    event("source_rejected", reason="missing boolean deadman")
                    stopped = True
                    return 2
                if newest.get("reset_requested"):
                    event("stop", reason="reset_requested")
                    stopped = True
                    return 0
                if not enabled:
                    if release_since is None:
                        release_since = now
                    if args.stream_contract == "h4":
                        # Keep H4 alive during the release grace window. The
                        # disabled frame resets source-zero and lets a fresh
                        # squeeze re-arm safely without restarting the bridge.
                        output_sequence += 1
                        wire = {
                            "schema_version": 1,
                            "sequence_id": output_sequence,
                            "upstream_sequence_id": last_sequence,
                            "enabled": False,
                            "sent_monotonic_s": time.monotonic(),
                        }
                        try:
                            print(json.dumps(wire, separators=(",", ":")), flush=True)
                        except BrokenPipeError:
                            sys.stdout = open(os.devnull, "w", encoding="utf-8")
                            event("stop", reason="downstream_closed", last_sequence=last_sequence)
                            stopped = True
                            return 2
                else:
                    release_since = None
                    solved_at = newest.get(
                        "upstream_solved_timestamp_monotonic_s",
                        newest["timestamp_monotonic_s"],
                    )
                    try:
                        solved_at = float(solved_at)
                    except (TypeError, ValueError):
                        solved_at = float("nan")
                    if not math.isfinite(solved_at) or now - solved_at > args.source_timeout_s:
                        event("stop", reason="stale_solved_sample", last_sequence=last_sequence)
                        stopped = True
                        return 2
                    if newest.get("upstream_solved_this_sample") is not True:
                        event("source_rejected", reason="enabled sample without fresh vendor solve")
                        stopped = True
                        return 2
                    try:
                        solved = parse_solution(newest)
                    except (TypeError, ValueError) as exc:
                        event("source_rejected", reason=str(exc))
                        stopped = True
                        return 2
                    bounded = np.clip(solved, lower, upper)
                    max_clamp = float(np.max(np.abs(bounded - solved)))
                    if max_clamp > 0.001:
                        event("source_rejected", reason="joint_limit_mismatch", max_clamp_rad=max_clamp)
                        stopped = True
                        return 2
                    positions = bounded if args.joint_limits_only else limiter.step(bounded)
                    output_sequence += 1
                    wire = {
                        "schema_version": 1,
                        "sequence_id": output_sequence,
                        "upstream_sequence_id": last_sequence,
                        # H4 sidecar rejects the old implicit-deadman wire.
                        # Keep this field on the legacy-compatible target too:
                        # the legacy parser ignores unknown keys, while the H4
                        # parser requires an explicit boolean deadman lease.
                        "enabled": True,
                        "sent_monotonic_s": time.monotonic(),
                        "joint_names": WIRE_NAMES,
                        "positions_rad": [*(float(v) for v in positions[:10]),
                                          float(positions[11]), float(positions[10])],
                        "solution_kind": SOLVER_ID,
                    }
                    try:
                        print(json.dumps(wire, separators=(",", ":")), flush=True)
                    except BrokenPipeError:
                        sys.stdout = open("/dev/null", "w", encoding="utf-8")
                        event("stop", reason="downstream_closed", last_sequence=last_sequence)
                        stopped = True
                        return 2
                    event("target_emitted", source_sequence=last_sequence,
                          target_sequence=output_sequence, source_age_s=now - float(newest["timestamp_monotonic_s"]),
                          max_clamp_rad=max_clamp)
            if (release_since is not None and args.release_debounce_s > 0 and
                    now - release_since >= args.release_debounce_s):
                event("stop", reason="deadman_released", last_sequence=last_sequence)
                stopped = True
                return 0
            remaining = period - (time.monotonic() - tick)
            if remaining > 0:
                time.sleep(remaining)
        event("stop", reason="duration_elapsed", last_sequence=last_sequence)
        stopped = True
        return 0
    finally:
        if not stopped:
            event("stop", reason="exception", last_sequence=last_sequence)
        if log:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())
