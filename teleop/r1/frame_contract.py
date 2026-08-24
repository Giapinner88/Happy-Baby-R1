"""Audited frame contract at the vendored TeleVuer boundary.

The vendor wrapper owns the OpenXR basis conversion, wrist local-axis
alignment, head-yaw-relative transform, and head-to-waist workspace offset.
Project code consumes its processed matrices, so vendor drift must fail before
the bridge emits a command rather than silently changing every IK target.
"""
from __future__ import annotations

import ast
import hashlib
import inspect
from pathlib import Path
import re
from typing import Any

import numpy as np


OPENXR_TO_ROBOT = np.array(
    [[0.0, 0.0, -1.0, 0.0], [-1.0, 0.0, 0.0, 0.0],
     [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
LEFT_WRIST_TO_UNITREE = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0],
     [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
RIGHT_WRIST_TO_UNITREE = np.array(
    [[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0],
     [0.0, -1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
)
HEAD_TO_WAIST_OFFSET_M = np.array([0.15, 0.0, 0.45])
ARM_REFERENCE_MODE = "head_yaw"
HEAD_POSE_FRAME = "robot_world"
WRIST_POSE_FRAME = "neutral_waist_yaw_link"
WRIST_INITIAL_CONVENTION = "unitree_arm_urdf"


class FrameContractError(RuntimeError):
    """Raised when the selected vendor wrapper no longer matches the audit."""


def _attribute_path(node: ast.AST) -> str:
    parts: list[str] = []
    current = node
    while isinstance(current, ast.Attribute):
        parts.append(current.attr)
        current = current.value
    if isinstance(current, ast.Name):
        parts.append(current.id)
    return ".".join(reversed(parts))


def validate_vendor_r1_a5_ik_contract(path: Path) -> dict[str, object]:
    """Fail closed when the vendored R1-A5 IK reference changes materially.

    This is a parity guard, not an instruction to duplicate the upstream
    optimizer.  Project solvers may deliberately differ, but their evidence
    must remain tied to the exact reference contract they were compared with.
    """

    source_path = Path(path).resolve()
    try:
        source = source_path.read_text(encoding="utf-8")
        tree = ast.parse(source)
    except (OSError, SyntaxError) as exc:
        raise FrameContractError(f"cannot audit vendor R1-A5 IK source: {exc}") from exc
    klass = next(
        (
            node
            for node in tree.body
            if isinstance(node, ast.ClassDef) and node.name == "R1_A5_ArmIK"
        ),
        None,
    )
    if klass is None:
        raise FrameContractError("vendor source has no R1_A5_ArmIK class")
    class_source = ast.get_source_segment(source, klass)
    if class_source is None:
        raise FrameContractError("cannot isolate vendor R1_A5_ArmIK source")
    compact = re.sub(r"\s+", "", class_source)

    required_fragments = {
        "locked waist/head joints": (
            '"waist_yaw_joint"',
            '"head_pitch_joint"',
            '"head_yaw_joint"',
        ),
        "0.20 m bilateral virtual endpoints": (
            "pin.Frame('L_ee'",
            "pin.Frame('R_ee'",
            "np.array([0.20,0,0]).T",
        ),
        "reference objective weights": (
            "self.opti.minimize(50*self.translational_cost+0.5*self.rotation_cost+0.02*self.regularization_cost+0.1*self.smooth_cost)",
        ),
        "URDF position bounds": (
            "self.reduced_robot.model.lowerPositionLimit",
            "self.reduced_robot.model.upperPositionLimit",
        ),
        "IPOPT iteration limit": ("'ipopt.max_iter':30",),
        "previous-solution warm start": (
            "'ipopt.warm_start_init_point':'yes'",
            "self.opti.set_initial(self.var_q,self.init_data)",
            "self.opti.set_value(self.var_q_last,self.init_data)",
        ),
        "FIR output filter": (
            "WeightedMovingFilter(np.array([0.4,0.3,0.2,0.1]),10)",
        ),
    }
    for label, fragments in required_fragments.items():
        if label == "0.20 m bilateral virtual endpoints":
            ok = all(fragment in compact for fragment in fragments[:2]) and (
                compact.count(fragments[2]) >= 2
            )
        else:
            ok = all(fragment in compact for fragment in fragments)
        if not ok:
            raise FrameContractError(f"vendor R1_A5_ArmIK changed: {label}")
    if "elbow_pole" in compact.lower() or "bilateral_symmetry" in compact.lower():
        raise FrameContractError(
            "vendor R1_A5_ArmIK now declares an elbow-pole or symmetry mechanism"
        )

    solve = next(
        (
            node
            for node in klass.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "solve_ik"
        ),
        None,
    )
    if solve is None:
        raise FrameContractError("vendor R1_A5_ArmIK has no solve_ik method")
    calls = [node for node in ast.walk(solve) if isinstance(node, ast.Call)]
    if any(_attribute_path(call.func).endswith(".scale_arms") for call in calls):
        raise FrameContractError("vendor R1_A5_ArmIK now applies scale_arms in solve_ik")
    direct_targets: set[tuple[str, str]] = set()
    for call in calls:
        if not _attribute_path(call.func).endswith(".set_value") or len(call.args) < 2:
            continue
        direct_targets.add((_attribute_path(call.args[0]), _attribute_path(call.args[1])))
    expected_targets = {
        ("self.param_tf_l", "left_wrist"),
        ("self.param_tf_r", "right_wrist"),
    }
    if not expected_targets.issubset(direct_targets):
        raise FrameContractError(
            "vendor R1_A5_ArmIK no longer sends wrist matrices directly to IK parameters"
        )

    return {
        "source_path": str(source_path),
        "source_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "class_name": "R1_A5_ArmIK",
        "wrist_target_policy": "direct_absolute_processed_pose",
        "virtual_endpoint_m": 0.2,
        "objective_weights": {
            "translation": 50.0,
            "rotation": 0.5,
            "joint_regularization": 0.02,
            "previous_solution": 0.1,
        },
        "ipopt_max_iterations": 30,
        "fir_weights": [0.4, 0.3, 0.2, 0.1],
        "explicit_elbow_pole_constraint": False,
        "explicit_bilateral_symmetry_constraint": False,
    }


def _matrix(module: Any, name: str) -> np.ndarray:
    try:
        value = np.asarray(getattr(module, name), dtype=float)
    except (AttributeError, TypeError, ValueError) as exc:
        raise FrameContractError(f"vendor wrapper is missing valid {name}") from exc
    if value.shape != (4, 4) or not np.all(np.isfinite(value)):
        raise FrameContractError(f"vendor {name} must be a finite 4x4 matrix")
    return value


def validate_vendor_frame_contract(module: Any) -> dict[str, object]:
    """Validate the exact transforms documented in ``docs/teleop/02``."""

    expected = {
        "T_ROBOT_OPENXR": OPENXR_TO_ROBOT,
        "T_TO_UNITREE_HUMANOID_LEFT_ARM": LEFT_WRIST_TO_UNITREE,
        "T_TO_UNITREE_HUMANOID_RIGHT_ARM": RIGHT_WRIST_TO_UNITREE,
    }
    for name, reference in expected.items():
        if not np.array_equal(_matrix(module, name), reference):
            raise FrameContractError(f"vendor {name} differs from the audited frame contract")

    try:
        default_mode = inspect.signature(module.TeleVuerWrapper.__init__).parameters[
            "arm_reference_mode"
        ].default
    except (AttributeError, KeyError, TypeError, ValueError) as exc:
        raise FrameContractError("vendor wrapper has no arm_reference_mode contract") from exc
    if default_mode != ARM_REFERENCE_MODE:
        raise FrameContractError(
            f"vendor default arm_reference_mode={default_mode!r}, expected {ARM_REFERENCE_MODE!r}"
        )

    # Audit the non-URDF workspace offset independently of the constant names.
    head = np.eye(4)
    head[:3, 3] = [0.1, -0.2, 1.4]
    wrist = np.eye(4)
    wrist[:3, 3] = [0.4, 0.3, 1.2]
    try:
        transformed = np.asarray(
            module.transform_IPunitree_Brobot_world_arm_to_head_then_waist(
                wrist, head, ARM_REFERENCE_MODE
            ),
            dtype=float,
        )
    except Exception as exc:
        raise FrameContractError("vendor head-yaw/waist transform could not be audited") from exc
    expected_position = wrist[:3, 3] - head[:3, 3] + HEAD_TO_WAIST_OFFSET_M
    if transformed.shape != (4, 4) or not np.allclose(
        transformed[:3, 3], expected_position, atol=1e-12, rtol=0.0
    ):
        raise FrameContractError("vendor head-to-waist offset differs from [0.15, 0, 0.45] m")

    return {
        "arm_reference_mode": ARM_REFERENCE_MODE,
        "head_pose_frame": HEAD_POSE_FRAME,
        "wrist_pose_frame": WRIST_POSE_FRAME,
        "wrist_initial_convention": WRIST_INITIAL_CONVENTION,
        "head_to_waist_offset_m": HEAD_TO_WAIST_OFFSET_M.tolist(),
    }


__all__ = [
    "ARM_REFERENCE_MODE",
    "FrameContractError",
    "HEAD_POSE_FRAME",
    "HEAD_TO_WAIST_OFFSET_M",
    "LEFT_WRIST_TO_UNITREE",
    "OPENXR_TO_ROBOT",
    "RIGHT_WRIST_TO_UNITREE",
    "WRIST_INITIAL_CONVENTION",
    "WRIST_POSE_FRAME",
    "validate_vendor_r1_a5_ik_contract",
    "validate_vendor_frame_contract",
]
