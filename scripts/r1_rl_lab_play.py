#!/usr/bin/env python3
"""Play/export the workspace R1 task through the upstream Unitree RL Lab script."""

from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RL_LAB_ROOT = ROOT / "third_party" / "unitree_rl_lab"


def main() -> None:
    os.environ.setdefault("HAPPY_BABY_R1_ROOT", str(ROOT))
    sys.path.insert(0, str(ROOT / "training"))
    sys.path.insert(0, str(RL_LAB_ROOT / "source" / "unitree_rl_lab"))
    sys.path.insert(0, str(RL_LAB_ROOT / "scripts" / "rsl_rl"))

    import happy_baby_r1_training.rl_lab  # noqa: F401

    script = RL_LAB_ROOT / "scripts" / "rsl_rl" / "play.py"
    sys.argv = [str(script)] + sys.argv[1:]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()

