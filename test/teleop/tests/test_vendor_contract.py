"""The IK dependency must name a reviewed immutable upstream snapshot."""
from __future__ import annotations

import json
import re
from pathlib import Path


TELEOP_DIR = Path(__file__).resolve().parents[1]


def test_vendor_contract_pins_the_r1_a5_solver_and_urdf() -> None:
    contract = json.loads(
        (TELEOP_DIR / "config" / "upstream_vendor_contract.json").read_text(
            encoding="utf-8"
        )
    )

    upstream = contract["upstream"]
    assert upstream["repository"] == "https://github.com/unitreerobotics/xr_teleoperate.git"
    assert upstream["release"] == "v1.6"
    assert upstream["release_date"] == "2026-07-29"
    assert upstream["git_tag"] is None
    assert upstream["commit"] == "64ed45b4177e6297936940866df623b72621643a"
    assert re.fullmatch(r"[0-9a-f]{40}", upstream["commit"])

    artifacts = contract["artifacts"]
    assert artifacts["ik_source"] == {
        "path": "teleop/robot_control/robot_arm_ik.py",
        "class": "R1_A5_ArmIK",
        "sha256": "f395b3ef868ed3c854654b08b33f74ee73f57dbb7c0e8f20440a7f788115a085",
    }
    assert artifacts["urdf"] == {
        "path": "assets/r1/r1_a5.urdf",
        "sha256": "886405d88699b3436eef8034f60a8379e8357ac6cfb74fb306ca8d15e9d75d89",
    }
