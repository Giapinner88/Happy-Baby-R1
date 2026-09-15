"""MJLab R1 robot config backed by the workspace MuJoCo runtime asset."""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg


WORKSPACE_ROOT = Path(
    os.environ.get("HAPPY_BABY_R1_ROOT", Path(__file__).resolve().parents[2])
)
R1_XML = WORKSPACE_ROOT / "assets" / "mujoco" / "unitree_robots" / "r1" / "R1.xml"


def get_assets(meshdir: str) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    update_assets(assets, R1_XML.parent / meshdir, meshdir)
    return assets


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(R1_XML))
    sensor_names = {sensor.name for sensor in spec.sensors}
    if "imu_ang_vel" not in sensor_names:
        spec.add_sensor(
            name="imu_ang_vel",
            type=mujoco.mjtSensor.mjSENS_GYRO,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname="imu",
        )
    if "imu_lin_vel" not in sensor_names:
        spec.add_sensor(
            name="imu_lin_vel",
            type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname="imu",
        )
    if "imu_lin_acc" not in sensor_names:
        spec.add_sensor(
            name="imu_lin_acc",
            type=mujoco.mjtSensor.mjSENS_ACCELEROMETER,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname="imu",
        )
    if "root_angmom" not in sensor_names:
        spec.add_sensor(
            name="root_angmom",
            type=mujoco.mjtSensor.mjSENS_SUBTREEANGMOM,
            objtype=mujoco.mjtObj.mjOBJ_BODY,
            objname="pelvis",
        )
    spec.assets = get_assets(spec.meshdir)
    return spec


# Actuator config — ported from the tuned Unitree RL MJLab R1 ("ok" G1-equivalent
# recipe, ~/train_mujoco). Legs split into HIP / KNEE / ANKLE and waist into
# YAW / ROLL so each group carries its own effort limit; damping raised 2.0 -> 3.0
# so the leg is less under-damped (oscillation decays faster after a disturbance).
# R1_ACTION_SCALE is derived from these (0.25 * effort / stiffness), so the runtime
# config.py KP/KD/ACTION_SCALE MUST be kept in sync with this block.
R1_ACTUATOR_HIP = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hip.*",),
    stiffness=100.0,
    damping=3.0,
    effort_limit=88.0,
    armature=0.01,
)
R1_ACTUATOR_KNEE = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_knee.*",),
    stiffness=100.0,
    damping=3.0,
    effort_limit=139.0,
    armature=0.01,
)
R1_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_ankle.*",),
    stiffness=40.0,
    damping=3.0,
    effort_limit=50.0,
    armature=0.01,
)
R1_ACTUATOR_WAIST_YAW = BuiltinPositionActuatorCfg(
    target_names_expr=("waist_yaw.*",),
    stiffness=100.0,
    damping=3.0,
    effort_limit=88.0,
    armature=0.01,
)
R1_ACTUATOR_WAIST_ROLL = BuiltinPositionActuatorCfg(
    target_names_expr=("waist_roll.*",),
    stiffness=100.0,
    damping=3.0,
    effort_limit=50.0,
    armature=0.01,
)
R1_ACTUATOR_ARM = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_shoulder.*", ".*_elbow.*", ".*_wrist.*"),
    stiffness=40.0,
    damping=2.0,
    effort_limit=25.0,
    armature=0.01,
)

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0, 0, 0.76),
    joint_pos={
        ".*_hip_pitch_joint": -0.1,
        ".*_knee_joint": 0.3,
        ".*_ankle_pitch_joint": -0.2,
        ".*_shoulder_pitch_joint": 0.35,
        ".*_elbow_joint": 0.87,
        "left_shoulder_roll_joint": 0.18,
        "right_shoulder_roll_joint": -0.18,
    },
    joint_vel={".*": 0.0},
)

FULL_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision",),
    condim={r"^(left|right)_foot[1-7]_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot[1-7]_collision$": 1},
    friction={r"^(left|right)_foot[1-7]_collision$": (0.6,)},
)

R1_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        R1_ACTUATOR_HIP,
        R1_ACTUATOR_KNEE,
        R1_ACTUATOR_ANKLE,
        R1_ACTUATOR_WAIST_YAW,
        R1_ACTUATOR_WAIST_ROLL,
        R1_ACTUATOR_ARM,
    ),
    soft_joint_pos_limit_factor=0.9,
)


def get_r1_robot_cfg() -> EntityCfg:
    return EntityCfg(
        init_state=HOME_KEYFRAME,
        collisions=(FULL_COLLISION,),
        spec_fn=get_spec,
        articulation=R1_ARTICULATION,
    )


R1_ACTION_SCALE: dict[str, float] = {}
for actuator in R1_ARTICULATION.actuators:
    assert isinstance(actuator, BuiltinPositionActuatorCfg)
    assert actuator.effort_limit is not None
    for name in actuator.target_names_expr:
        R1_ACTION_SCALE[name] = 0.25 * actuator.effort_limit / actuator.stiffness
