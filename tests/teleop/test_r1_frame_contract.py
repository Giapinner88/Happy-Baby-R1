from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from teleop.r1.frame_contract import (
    FrameContractError,
    LEFT_WRIST_TO_UNITREE,
    OPENXR_TO_ROBOT,
    RIGHT_WRIST_TO_UNITREE,
    validate_vendor_frame_contract,
    validate_vendor_r1_a5_ik_contract,
)


ROOT = Path(__file__).resolve().parents[2]
R1_A5_IK_SOURCE = (
    ROOT / "third_party/xr_teleoperate_v1_6/teleop/robot_control/robot_arm_ik.py"
)


class Wrapper:
    def __init__(self, arm_reference_mode: str = "head_yaw") -> None:
        self.arm_reference_mode = arm_reference_mode


def transform(wrist: np.ndarray, head: np.ndarray, mode: str) -> np.ndarray:
    assert mode == "head_yaw"
    result = wrist.copy()
    result[:3, 3] = wrist[:3, 3] - head[:3, 3] + [0.15, 0.0, 0.45]
    return result


def module(**overrides: object) -> SimpleNamespace:
    values: dict[str, object] = {
        "T_ROBOT_OPENXR": OPENXR_TO_ROBOT.copy(),
        "T_TO_UNITREE_HUMANOID_LEFT_ARM": LEFT_WRIST_TO_UNITREE.copy(),
        "T_TO_UNITREE_HUMANOID_RIGHT_ARM": RIGHT_WRIST_TO_UNITREE.copy(),
        "TeleVuerWrapper": Wrapper,
        "transform_IPunitree_Brobot_world_arm_to_head_then_waist": transform,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_audited_vendor_contract_is_accepted() -> None:
    record = validate_vendor_frame_contract(module())
    assert record["arm_reference_mode"] == "head_yaw"
    assert record["wrist_pose_frame"] == "neutral_waist_yaw_link"
    assert record["head_to_waist_offset_m"] == [0.15, 0.0, 0.45]


def test_basis_drift_is_refused() -> None:
    changed = OPENXR_TO_ROBOT.copy()
    changed[0, 0] = 1.0
    with pytest.raises(FrameContractError, match="T_ROBOT_OPENXR"):
        validate_vendor_frame_contract(module(T_ROBOT_OPENXR=changed))


def test_waist_offset_drift_is_refused() -> None:
    def wrong_offset(wrist: np.ndarray, head: np.ndarray, mode: str) -> np.ndarray:
        result = wrist.copy()
        result[:3, 3] = wrist[:3, 3] - head[:3, 3]
        return result

    with pytest.raises(FrameContractError, match="head-to-waist offset"):
        validate_vendor_frame_contract(
            module(transform_IPunitree_Brobot_world_arm_to_head_then_waist=wrong_offset)
        )


def test_real_vendor_r1_a5_ik_contract_is_accepted() -> None:
    record = validate_vendor_r1_a5_ik_contract(R1_A5_IK_SOURCE)
    assert record["wrist_target_policy"] == "direct_absolute_processed_pose"
    assert record["virtual_endpoint_m"] == 0.2
    assert record["objective_weights"]["previous_solution"] == 0.1
    assert record["fir_weights"] == [0.4, 0.3, 0.2, 0.1]


def test_vendor_r1_a5_objective_drift_is_refused(tmp_path: Path) -> None:
    source = R1_A5_IK_SOURCE.read_text(encoding="utf-8")
    prefix, r1_source = source.split("class R1_A5_ArmIK:", maxsplit=1)
    changed = prefix + "class R1_A5_ArmIK:" + r1_source.replace(
        "50 * self.translational_cost", "49 * self.translational_cost", 1
    )
    changed_path = tmp_path / "robot_arm_ik.py"
    changed_path.write_text(changed, encoding="utf-8")
    with pytest.raises(FrameContractError, match="objective weights"):
        validate_vendor_r1_a5_ik_contract(changed_path)
