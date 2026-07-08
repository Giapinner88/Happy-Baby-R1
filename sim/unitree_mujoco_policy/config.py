import os
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
UNITREE_MUJOCO_ROOT = Path(
    os.environ.get("UNITREE_MUJOCO_ROOT", REPO_ROOT / "third_party" / "unitree_mujoco")
).expanduser()
LOCAL_MUJOCO_ROOT = Path(
    os.environ.get("LOCAL_MUJOCO_ROOT", REPO_ROOT / "asset" / "mujoco")
).expanduser()
MODEL_ROOT = Path(
    os.environ.get("UNITREE_MUJOCO_POLICY_MODEL_ROOT", REPO_ROOT / "data" / "models" / "unitree_mujoco_policy")
).expanduser()

# Policy selection lives here by design. Add new R1 policies to POLICIES and
# change POLICY_NAME; the launcher always runs policy_runner.py.
POLICY_NAME = os.environ.get("POLICY_NAME", "r1_velocity")

MOTOR_JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
    "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
    "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_roll", "waist_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll",
]

R1_VELOCITY_CONTROLLED_JOINTS = [
    *MOTOR_JOINT_NAMES,
]

POLICIES = {
    "r1_velocity": {
        "robot": "r1",
        "scene": "scene_hanging.xml",
        "onnx": "r1_velocity.onnx",
        "controlled_joints": R1_VELOCITY_CONTROLLED_JOINTS,
        "step_dt": 0.02,
    },
}

if POLICY_NAME not in POLICIES:
    known = ", ".join(sorted(POLICIES))
    raise ValueError(f"Unknown POLICY_NAME={POLICY_NAME!r}. Known policies: {known}")

POLICY = POLICIES[POLICY_NAME]

ROBOT = POLICY["robot"]
ROBOT_SCENE_NAME = os.environ.get("ROBOT_SCENE_NAME", os.environ.get("SCENE", POLICY["scene"]))


def _bool_env(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def _resolve_robot_scene() -> Path:
    override = os.environ.get("ROBOT_SCENE")
    if override:
        return Path(override).expanduser().resolve()

    scene_path = Path(ROBOT_SCENE_NAME).expanduser()
    if scene_path.is_absolute():
        return scene_path.resolve()

    local_scene = LOCAL_MUJOCO_ROOT / "unitree_robots" / ROBOT / scene_path
    if local_scene.exists():
        return local_scene.resolve()

    upstream_scene = UNITREE_MUJOCO_ROOT / "unitree_robots" / ROBOT / scene_path
    return upstream_scene.resolve()


ROBOT_SCENE = str(_resolve_robot_scene())
DOMAIN_ID = int(os.environ.get("DOMAIN_ID", "1")) # Domain id
INTERFACE = os.environ.get("INTERFACE", "lo") # Interface

USE_JOYSTICK = _bool_env("USE_JOYSTICK", False) # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = os.environ.get("JOYSTICK_TYPE", "switch") # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = int(os.environ.get("JOYSTICK_DEVICE", "0")) # Joystick number

PRINT_SCENE_INFORMATION = _bool_env("PRINT_SCENE_INFORMATION", True) # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = _bool_env("ENABLE_ELASTIC_BAND", False) # Virtual spring band, used for lifting humanoids
HUMANOID_HG_ROBOTS = {"r1"}
USE_HG_IDL = _bool_env("USE_HG_IDL", True)

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer


def _resolve_policy_path() -> Path:
    override = os.environ.get("POLICY_ONNX")
    if override:
        path = Path(override).expanduser()
        if path.is_absolute():
            return path.resolve()
        local_override = MODEL_ROOT / path
        if local_override.exists():
            return local_override.resolve()
        repo_override = REPO_ROOT / path
        if repo_override.exists():
            return repo_override.resolve()
        return (Path(__file__).resolve().parent / path).resolve()

    onnx_path = Path(POLICY["onnx"]).expanduser()
    if onnx_path.is_absolute():
        return onnx_path.resolve()
    return (MODEL_ROOT / onnx_path).resolve()


POLICY_ONNX = str(_resolve_policy_path())
CONTROL_DT = float(POLICY["step_dt"])
CONTROLLED_JOINTS = list(POLICY["controlled_joints"])
CONTROLLED_INDICES = np.array(
    [MOTOR_JOINT_NAMES.index(name) for name in CONTROLLED_JOINTS],
    dtype=np.int64,
)
ACTION_DIM = len(CONTROLLED_JOINTS)
OBS_DIM = 3 + 3 + 3 + 2 + ACTION_DIM + ACTION_DIM + ACTION_DIM

DEFAULT_Q_BY_JOINT = {
    "left_hip_pitch": -0.1,
    "left_knee": 0.3,
    "left_ankle_pitch": -0.2,
    "right_hip_pitch": -0.1,
    "right_knee": 0.3,
    "right_ankle_pitch": -0.2,
    "left_shoulder_pitch": 0.35,
    "left_shoulder_roll": 0.18,
    "left_elbow": 0.87,
    "right_shoulder_pitch": 0.35,
    "right_shoulder_roll": -0.18,
    "right_elbow": 0.87,
}
DEFAULT_Q = np.array(
    [DEFAULT_Q_BY_JOINT.get(name, 0.0) for name in MOTOR_JOINT_NAMES],
    dtype=np.float32,
)

KP_BY_JOINT = {}
KD_BY_JOINT = {}
ACTION_SCALE_BY_JOINT = {}
for name in MOTOR_JOINT_NAMES:
    if any(key in name for key in ("hip", "knee")):
        kp, kd, scale = 100.0, 2.0, 0.15
    elif "ankle" in name:
        kp, kd, scale = 40.0, 2.0, 0.3125
    elif "waist" in name:
        kp, kd, scale = 100.0, 2.0, 0.15
    elif "shoulder_pitch" in name or "shoulder_roll" in name:
        kp, kd, scale = 40.0, 2.0, 0.375
    elif "shoulder_yaw" in name or "elbow" in name or "wrist_roll" in name:
        kp, kd, scale = 20.0, 1.0, 0.4125
    else:
        kp, kd, scale = 20.0, 1.0, 0.0
    KP_BY_JOINT[name] = kp
    KD_BY_JOINT[name] = kd
    ACTION_SCALE_BY_JOINT[name] = scale

KP_ARRAY = np.array([KP_BY_JOINT[name] for name in MOTOR_JOINT_NAMES], dtype=np.float32)
KD_ARRAY = np.array([KD_BY_JOINT[name] for name in MOTOR_JOINT_NAMES], dtype=np.float32)
ACTION_SCALE = np.array(
    [ACTION_SCALE_BY_JOINT[name] for name in MOTOR_JOINT_NAMES],
    dtype=np.float32,
)
