"""5-DOF elbow swivel retargeting and hand/elbow decoupling regression tests."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_r1_vendor_ik_stream", ROOT / "scripts" / "teleop" / "run_r1_vendor_ik_stream.py"
)
assert SPEC is not None and SPEC.loader is not None
ik = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ik)


def _build_arm_fk(side: str):
    tree = ET.parse(ROOT / "third_party" / "xr_teleoperate_r1_a5_845b25b" / "assets" / "r1" / "r1_a5.urdf")
    joints = {j.attrib["name"]: j for j in tree.getroot().findall("joint")}
    chain = [
        f"{side}_shoulder_pitch_joint",
        f"{side}_shoulder_roll_joint",
        f"{side}_shoulder_yaw_joint",
        f"{side}_elbow_joint",
        f"{side}_wrist_roll_joint",
    ]
    bounds = [
        (float(joints[n].find("limit").attrib["lower"]), float(joints[n].find("limit").attrib["upper"]))
        for n in chain
    ]

    def fk(q: np.ndarray) -> dict[str, np.ndarray]:
        T = np.eye(4)
        frames: dict[str, np.ndarray] = {}
        for name, angle in zip(chain, q):
            j = joints[name]
            xyz = [float(v) for v in j.find("origin").attrib["xyz"].split()]
            rpy = [float(v) for v in j.find("origin").attrib["rpy"].split()]
            axis = [float(v) for v in j.find("axis").attrib["xyz"].split()]
            Tj = np.eye(4)
            Tj[:3, :3] = (
                Rotation.from_euler("xyz", rpy).as_matrix()
                @ Rotation.from_rotvec(np.asarray(axis, dtype=float) * float(angle)).as_matrix()
            )
            Tj[:3, 3] = xyz
            T = T @ Tj
            frames[name] = T.copy()
        Te = np.eye(4)
        Te[:3, 3] = [0.20, 0.0, 0.0]
        frames["ee"] = T @ Te
        return frames

    return fk, bounds


def _solve_arm_ik(fk, bounds, T_target: np.ndarray, p_elbow_target: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
    def cost(q_var: np.ndarray) -> float:
        frames = fk(q_var)
        pos_err = frames["ee"][:3, 3] - T_target[:3, 3]
        el_name = "right_elbow_joint" if "right_elbow_joint" in frames else "left_elbow_joint"
        el_err = frames[el_name][:3, 3] - p_elbow_target
        rot_err = Rotation.from_matrix(frames["ee"][:3, :3] @ T_target[:3, :3].T).as_rotvec()
        w = ik.SWIVEL_IK_WEIGHTS
        return float(
            w["translational"] * np.sum(pos_err**2)
            + w["elbow"] * np.sum(el_err**2)
            + w["rotation"] * np.sum(rot_err**2)
            + w["regularization"] * np.sum(q_var**2)
            + w["smooth"] * np.sum((q_var - q_prev) ** 2)
        )

    res = minimize(cost, q_prev, method="SLSQP", bounds=bounds, options={"ftol": 1e-9, "maxiter": 100})
    return np.asarray(res.x, dtype=float)


@pytest.mark.parametrize("side", ["left", "right"])
def test_neutral_pose_is_preserved_exactly(side: str) -> None:
    fk, _ = _build_arm_fk(side)
    f0 = fk(np.zeros(5))
    T_neutral = f0["ee"]
    p_el0 = f0[f"{side}_elbow_joint"][:3, 3]

    T_out, p_el_out = ik.retarget_wrist_for_elbow_swivel(T_neutral, side)
    np.testing.assert_allclose(T_out[:3, 3], T_neutral[:3, 3], atol=1e-6)
    np.testing.assert_allclose(p_el_out, p_el0, atol=1e-4)
    rot_err_deg = np.rad2deg(
        np.linalg.norm(Rotation.from_matrix(T_out[:3, :3] @ T_neutral[:3, :3].T).as_rotvec())
    )
    assert rot_err_deg < 0.02


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("ray_pitch_deg", [0.0, 35.0, -25.0])
@pytest.mark.parametrize("mode", ["pure_roll", "rigid_swivel", "forearm_yaw"])
def test_fixed_hand_with_elbow_swing_moves_elbow_and_holds_hand_steady(
    side: str, ray_pitch_deg: float, mode: str
) -> None:
    """When the operator holds the controller at a fixed point and swings their elbow,
    the robot must swing its elbow while keeping its hand stationary (without wrist whip).
    """
    fk, bounds = _build_arm_fk(side)
    sign = 1.0 if side == "left" else -1.0
    p_ee_fixed = np.array([0.28, sign * 0.20, 0.02], dtype=float)
    p_shoulder = np.array([0.03445, sign * 0.1379, 0.1849], dtype=float)
    u_axis = (p_ee_fixed - p_shoulder) / np.linalg.norm(p_ee_fixed - p_shoulder)

    q_prev = np.zeros(5)
    elbow_positions = []
    ee_positions = []
    shoulder_rolls = []
    wrist_rolls = []

    for flap_deg in (0.0, 15.0, 30.0, 45.0):
        outward_angle = sign * flap_deg
        if mode == "pure_roll":
            R_forearm = Rotation.from_euler("x", outward_angle, degrees=True).as_matrix()
        elif mode == "rigid_swivel":
            R_forearm = Rotation.from_rotvec(np.deg2rad(outward_angle) * u_axis).as_matrix()
        else:
            R_forearm = Rotation.from_euler("z", -outward_angle, degrees=True).as_matrix()
        R_ctrl = R_forearm @ Rotation.from_euler("y", ray_pitch_deg, degrees=True).as_matrix()

        T_raw = np.eye(4)
        T_raw[:3, :3] = R_ctrl
        T_raw[:3, 3] = p_ee_fixed

        T_target, p_el_target = ik.retarget_wrist_for_elbow_swivel(T_raw, side)
        q_prev = _solve_arm_ik(fk, bounds, T_target, p_el_target, q_prev)
        frames = fk(q_prev)
        elbow_positions.append(frames[f"{side}_elbow_joint"][:3, 3].copy())
        ee_positions.append(frames["ee"][:3, 3].copy())
        shoulder_rolls.append(float(np.rad2deg(q_prev[1])))
        wrist_rolls.append(float(np.rad2deg(q_prev[4])))

    # 1. Hand (EE) stays rock-solid at p_ee_fixed (< 5 mm drift across 45 deg elbow flap):
    for p_ee in ee_positions:
        assert np.linalg.norm(p_ee - p_ee_fixed) < 0.005

    # 2. Elbow swings outward monotonically by > 10 cm in 3D (> 35 deg shoulder_roll):
    total_elbow_travel = np.linalg.norm(elbow_positions[-1] - elbow_positions[0])
    assert total_elbow_travel > 0.10
    assert abs(shoulder_rolls[-1] - shoulder_rolls[0]) > 35.0
    if side == "right":
        assert shoulder_rolls[0] > shoulder_rolls[1] > shoulder_rolls[2] > shoulder_rolls[3]
    else:
        assert shoulder_rolls[0] < shoulder_rolls[1] < shoulder_rolls[2] < shoulder_rolls[3]

    # 3. Wrist roll does not hijack the elbow swivel:
    assert max(abs(w) for w in wrist_rolls) < 12.0


def test_fixed_elbow_with_hand_swing_keeps_elbow_stationary() -> None:
    """When the operator keeps their elbow fixed and swings their forearm/hand
    left/right or up/down, the robot's elbow stays fixed while the hand tracks.
    """
    fk, bounds = _build_arm_fk("right")
    q_prev = np.zeros(5)
    for yaw_deg, el_deg in [(-25.0, 0.0), (0.0, 0.0), (25.0, 0.0), (0.0, -20.0), (0.0, 25.0)]:
        q_true = np.deg2rad([0.0, 0.0, yaw_deg, el_deg, 0.0])
        f_true = fk(q_true)
        T_target, p_el_target = ik.retarget_wrist_for_elbow_swivel(f_true["ee"], "right")
        # The vendor smooth weight (0.1) lags a single step; a held target is
        # resolved every frame, so check the error after a few frames.
        for _ in range(3):
            q_prev = _solve_arm_ik(fk, bounds, T_target, p_el_target, q_prev)
        f_sol = fk(q_prev)
        elbow_err = np.linalg.norm(
            f_sol["right_elbow_joint"][:3, 3] - f_true["right_elbow_joint"][:3, 3]
        )
        ee_err = np.linalg.norm(f_sol["ee"][:3, 3] - f_true["ee"][:3, 3])
        assert elbow_err < 0.008
        assert ee_err < 0.003


def test_vendor_mode_leaves_solver_untouched() -> None:
    class Solver:
        def solve_ik(self, left, right):
            return np.zeros(10), np.zeros(10)

    solver = Solver()
    original = solver.solve_ik
    ik.configure_elbow_aware_solver(solver, "vendor")
    assert solver.elbow_mode == "vendor"
    assert solver.solve_ik == original
    assert not hasattr(solver, "elbow_cost")


@pytest.mark.parametrize("first", ["vendor", "swivel"])
def test_elbow_mode_cannot_be_reconfigured(first: str) -> None:
    solver = type("Solver", (), {})()
    solver.elbow_mode = first
    with pytest.raises(RuntimeError):
        ik.configure_elbow_aware_solver(solver, "vendor")


def test_swivel_mode_rejects_solver_without_opti_terms() -> None:
    solver = type("Solver", (), {"solve_ik": lambda self, left, right: None})()
    with pytest.raises(SystemExit):
        ik.configure_elbow_aware_solver(solver, "swivel")


@pytest.mark.parametrize("mode", ["vendor", "swivel"])
def test_main_passes_elbow_mode_to_load_solver(mode: str, monkeypatch, tmp_path) -> None:
    seen = []

    class Solver:
        def solve_ik(self, left, right):
            return np.zeros(10), np.zeros(10)

    def load_solver(elbow_mode):
        seen.append(elbow_mode)
        return Solver()

    empty = tmp_path / "empty.jsonl"
    empty.write_text("", encoding="utf-8")
    monkeypatch.setattr(ik, "load_solver", load_solver)
    monkeypatch.setattr("sys.argv", ["ik", "--input", str(empty), "--elbow-mode", mode])
    try:
        ik.main()
    except SystemExit:
        pass
    assert seen == [mode]
