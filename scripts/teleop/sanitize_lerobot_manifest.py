#!/usr/bin/env python3
"""Remove workstation-specific absolute paths from an R1 LeRobot manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def repository_relative(value: object, repo_root: Path) -> str:
    path = Path(str(value)).expanduser().resolve()
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError as exc:
        raise ValueError(f"từ chối upload đường dẫn ngoài repository: {path}") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    args = parser.parse_args()

    dataset_root = args.dataset_root.expanduser().resolve()
    repo_root = args.repo_root.expanduser().resolve()
    manifest_path = dataset_root / "meta" / "r1_conversion.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest["source_root"] = repository_relative(manifest["source_root"], repo_root)
        manifest["source_episodes"] = [
            repository_relative(value, repo_root)
            for value in manifest.get("source_episodes", [])
        ]
    except (OSError, KeyError, TypeError, json.JSONDecodeError, ValueError) as exc:
        print(f"FAIL: không sanitize được {manifest_path}: {exc}")
        return 2

    manifest["path_policy"] = "repository_relative_no_host_path"
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"PASS: manifest chỉ còn đường dẫn tương đối: {manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
