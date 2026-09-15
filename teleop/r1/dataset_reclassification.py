"""Create an immutable, traceable reclassification of recorded XR episodes."""

from __future__ import annotations

import hashlib
import json
import re
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TOO_SHORT_PATTERN = re.compile(r"^too_short: (\d+) item < (\d+)$")


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return payload


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _episode_number(path: Path) -> int:
    try:
        return int(path.name.removeprefix("episode_"))
    except ValueError as exc:
        raise ValueError(f"invalid episode directory name: {path.name}") from exc


def _validate_images(episode_dir: Path, frames: list[Any]) -> None:
    for position, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"{episode_dir}: frame {position} is not an object")
        colors = frame.get("colors")
        if not isinstance(colors, dict) or not colors:
            raise ValueError(f"{episode_dir}: frame {position} has no color image")
        for relative in colors.values():
            image_path = episode_dir / str(relative)
            if not image_path.is_file():
                raise ValueError(f"{episode_dir}: missing image {relative}")


def _only_too_short(reasons: list[Any]) -> bool:
    return bool(reasons) and all(
        isinstance(reason, str) and TOO_SHORT_PATTERN.fullmatch(reason)
        for reason in reasons
    )


def _summary(episode_number: int, payload: dict[str, Any]) -> dict[str, Any]:
    info = payload["info"]
    task = info.get("task") or {}
    frames = payload["data"]
    return {
        "episode": episode_number,
        "item_count": len(frames),
        "projected_item_count": int(info.get("projected_item_count") or 0),
        "writer_dropped_capture_count": int(info.get("writer_dropped_capture_count") or 0),
        "measured_fps": (info.get("image") or {}).get("fps"),
        "accepted": bool(info.get("accepted")),
        "rejection_reasons": list(info.get("rejection_reasons") or []),
        "target_digit": task.get("target_digit"),
        "target_slot": task.get("target_slot"),
    }


def _frame_index(episode_number: int, payload: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for position, frame in enumerate(payload["data"]):
        sim_state = frame.get("sim_state") or {}
        row: dict[str, Any] = {
            "episode": episode_number,
            "idx": int(frame.get("idx", position)),
            "control_step": sim_state.get("control_step"),
            "elapsed_s": sim_state.get("elapsed_s"),
        }
        for camera_name, relative in (frame.get("colors") or {}).items():
            row[str(camera_name)] = f"episode_{episode_number:04d}/{relative}"
        rows.append(row)
    return rows


def reclassify_run(source_run: Path, output_root: Path, min_items: int) -> dict[str, Any]:
    """Copy a run and waive only its old ``too_short`` rejection criterion."""

    source_run = source_run.expanduser().resolve()
    output_root = output_root.expanduser().resolve()
    if min_items < 1:
        raise ValueError("min_items must be at least 1")
    if not source_run.is_dir():
        raise ValueError(f"source run does not exist: {source_run}")
    if output_root.exists():
        raise FileExistsError(f"refusing to overwrite derived dataset: {output_root}")

    source_index_path = source_run / "episode_index.json"
    source_index = _read_json(source_index_path) if source_index_path.is_file() else {}
    original_policy = dict(source_index.get("rejection_policy") or {})
    source_dirs: list[tuple[Path, bool]] = []
    for branch, originally_accepted in (
        ("episodes", True),
        ("episodes_accepted", True),
        ("episodes_rejected", False),
    ):
        branch_root = source_run / branch
        if branch_root.is_dir():
            source_dirs.extend(
                (path, originally_accepted)
                for path in sorted(branch_root.glob("episode_*"))
                if path.is_dir()
            )
    if not source_dirs:
        raise ValueError(f"source run has no episode directories: {source_run}")

    episode_names: set[str] = set()
    accepted_summaries: list[dict[str, Any]] = []
    rejected_summaries: list[dict[str, Any]] = []
    frame_rows: list[dict[str, Any]] = []
    promoted: list[str] = []
    source_records: list[dict[str, Any]] = []
    incomplete_episode_directories: list[str] = []

    output_root.mkdir(parents=True)
    accepted_root = output_root / "episodes"
    rejected_root = output_root / "episodes_rejected"
    accepted_root.mkdir()
    rejected_root.mkdir()
    try:
        for source_episode, originally_accepted in source_dirs:
            if source_episode.name in episode_names:
                raise ValueError(f"duplicate episode name in source run: {source_episode.name}")
            episode_names.add(source_episode.name)
            data_path = source_episode / "data.json"
            if not data_path.is_file():
                incomplete_episode_directories.append(str(source_episode))
                continue
            payload = _read_json(data_path)
            frames = payload.get("data")
            info = payload.get("info")
            if not isinstance(frames, list) or not frames or not isinstance(info, dict):
                raise ValueError(f"invalid episode payload: {data_path}")
            _validate_images(source_episode, frames)
            reasons = list(info.get("rejection_reasons") or [])
            was_accepted = bool(info.get("accepted"))
            if was_accepted != originally_accepted:
                raise ValueError(
                    f"branch/status mismatch for {data_path}: accepted={was_accepted}"
                )

            promote = not was_accepted and len(frames) >= min_items and _only_too_short(reasons)
            destination_root = accepted_root if was_accepted or promote else rejected_root
            destination = destination_root / source_episode.name
            shutil.copytree(source_episode, destination, copy_function=shutil.copy2)
            source_hash = _sha256(data_path)
            if promote:
                derived_payload = _read_json(destination / "data.json")
                derived_info = derived_payload["info"]
                derived_info["accepted"] = True
                derived_info["rejection_reasons"] = []
                derived_info["reclassification"] = {
                    "schema": "happy_baby_r1.episode_reclassification",
                    "schema_version": 1,
                    "source_run": str(source_run),
                    "source_episode": source_episode.name,
                    "source_data_sha256": source_hash,
                    "original_accepted": False,
                    "original_rejection_reasons": reasons,
                    "new_min_items": min_items,
                    "waived_criterion": "too_short_only",
                }
                (destination / "data.json").write_text(
                    json.dumps(derived_payload, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                payload = derived_payload
                promoted.append(source_episode.name)

            number = _episode_number(source_episode)
            summary = _summary(number, payload)
            if summary["accepted"]:
                accepted_summaries.append(summary)
                frame_rows.extend(_frame_index(number, payload))
            else:
                rejected_summaries.append(summary)
            source_records.append(
                {
                    "episode": source_episode.name,
                    "source_branch": source_episode.parent.name,
                    "source_data_sha256": source_hash,
                    "frames": len(frames),
                    "promoted": promote,
                }
            )

        summaries = sorted(accepted_summaries + rejected_summaries, key=lambda row: row["episode"])
        rejection_counts: dict[str, int] = {}
        for summary in rejected_summaries:
            for reason in summary["rejection_reasons"]:
                category = str(reason).split(":", 1)[0]
                rejection_counts[category] = rejection_counts.get(category, 0) + 1
        derived_index = {
            "episode_count_total": len(summaries),
            "episode_count_accepted": len(accepted_summaries),
            "episode_count_rejected": len(rejected_summaries),
            "rejection_reason_counts": rejection_counts,
            "rejection_policy": {
                **original_policy,
                "min_items": min_items,
                "derivation": "waive prior too_short rejection only",
            },
            "episodes": summaries,
            "frames": frame_rows,
            "derived_from": str(source_run),
        }
        (output_root / "episode_index.json").write_text(
            json.dumps(derived_index, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

        measured_fps = [
            float(summary["measured_fps"])
            for summary in accepted_summaries
            if summary["measured_fps"] is not None
        ]
        manifest = {
            "schema": "happy_baby_r1.dataset_reclassification",
            "schema_version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "source_run": str(source_run),
            "source_run_id": source_run.name,
            "source_episode_index_sha256": (
                _sha256(source_index_path) if source_index_path.is_file() else None
            ),
            "source_episode_index_present": source_index_path.is_file(),
            "source_run_immutable": True,
            "new_min_items": min_items,
            "waived_criterion": "too_short_only",
            "other_rejection_reasons_preserved": True,
            "episodes_total": len(summaries),
            "episodes_accepted": len(accepted_summaries),
            "episodes_promoted": len(promoted),
            "episodes_rejected": len(rejected_summaries),
            "frames_accepted": len(frame_rows),
            "accepted_measured_fps_range": (
                [min(measured_fps), max(measured_fps)] if measured_fps else None
            ),
            "certified_true_30_fps": bool(
                measured_fps and all(27.0 <= fps <= 33.0 for fps in measured_fps)
            ),
            "promoted_episode_names": promoted,
            "incomplete_episode_directories": incomplete_episode_directories,
            "source_episodes": source_records,
        }
        (output_root / "reclassification_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return manifest
    except BaseException:
        shutil.rmtree(output_root, ignore_errors=True)
        raise


__all__ = ["reclassify_run"]
