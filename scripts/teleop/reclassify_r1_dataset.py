#!/usr/bin/env python3
"""Create a traceable accepted view from a completed run or recovery copy."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from teleop.r1.dataset_reclassification import reclassify_run  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-run", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--min-items", type=int, default=30)
    args = parser.parse_args()
    manifest = reclassify_run(args.source_run, args.output_root, args.min_items)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
