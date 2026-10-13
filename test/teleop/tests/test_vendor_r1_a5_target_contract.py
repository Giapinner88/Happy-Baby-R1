"""Offline wire and provenance checks for the opt-in R1-A5 vendor route."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_r1_vendor_targets", ROOT / "scripts" / "teleop" / "run_r1_vendor_targets.py"
)
assert SPEC is not None and SPEC.loader is not None
targets = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(targets)


def test_pinned_vendor_artifacts_are_present() -> None:
    contract = json.loads((ROOT / "config" / "vendor_r1_a5_845b25b.json").read_text())
    assert contract["commit"] == "845b25a32f7febedf220e830952a7134897adb9d"
    assert hashlib.sha256((ROOT / "src" / "assets" / "R1.urdf").read_bytes()).hexdigest() == contract["hb_urdf_sha256"]
    vendor = ROOT / "third_party" / "xr_teleoperate_r1_a5_845b25b"
    for artifact in contract["artifacts"].values():
        assert hashlib.sha256((vendor / artifact["path"]).read_bytes()).hexdigest() == artifact["sha256"]
    meshes = sorted((vendor / "assets" / "r1" / "meshes").iterdir())
    manifest = "".join(f"{p.name} {hashlib.sha256(p.read_bytes()).hexdigest()}\n" for p in meshes)
    assert len(meshes) == contract["mesh_count"]
    assert hashlib.sha256(manifest.encode()).hexdigest() == contract["mesh_tree_sha256"]


def test_vendor_joint_order_is_explicit_and_head_wire_order_is_swapped() -> None:
    assert targets.MODEL_NAMES[-2:] == ("head_pitch_joint", "head_yaw_joint")
    assert targets.WIRE_NAMES[-2:] == ("head_yaw_joint", "head_pitch_joint")
    payload = {
        "upstream_solver": targets.SOLVER_ID,
        "upstream_joint_names": targets.MODEL_NAMES,
        "upstream_joint_position_rad": [0.0] * 10 + [0.2, -0.3],
    }
    assert targets.parse_solution(payload).tolist()[-2:] == [0.2, -0.3]
    payload["upstream_joint_names"] = targets.WIRE_NAMES
    with pytest.raises(ValueError, match="joint order"):
        targets.parse_solution(payload)


def test_vendor_solution_rejects_nonfinite_and_hb_limits_cover_neutral() -> None:
    lo, hi = targets.joint_limits(ROOT / "src" / "assets" / "R1.urdf")
    assert lo.shape == hi.shape == (12,)
    assert np.all(lo <= 0.0) and np.all(hi >= 0.0)
    payload = {
        "upstream_solver": targets.SOLVER_ID,
        "upstream_joint_names": targets.MODEL_NAMES,
        "upstream_joint_position_rad": [0.0] * 11 + [float("nan")],
    }
    with pytest.raises(ValueError, match="finite"):
        targets.parse_solution(payload)
