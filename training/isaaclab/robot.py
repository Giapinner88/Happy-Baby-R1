"""IsaacLab articulation config for the workspace R1 USD asset."""

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import configclass


WORKSPACE_ROOT = Path(os.environ.get("HAPPY_BABY_R1_ROOT", Path(__file__).resolve().parents[3]))
R1_USD_PATH = WORKSPACE_ROOT / "assets" / "R1" / "R1.usd"


@configclass
class R1UsdFileCfg(sim_utils.UsdFileCfg):
    activate_contact_sensors: bool = True
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=True,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
    )


@configclass
class R1ArticulationCfg(ArticulationCfg):
    """Isaac Lab articulation plus the motor ordering used by the R1 SDK."""

    joint_sdk_names: list[str] = None


UNITREE_R1_CFG = R1ArticulationCfg(
    spawn=R1UsdFileCfg(
        usd_path=str(R1_USD_PATH),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.76),
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
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_pitch_joint",
                ".*_hip_roll_joint",
                ".*_hip_yaw_joint",
                ".*_knee_joint",
            ],
            effort_limit_sim=60.0,
            velocity_limit_sim=32.0,
            stiffness=100.0,
            damping=2.0,
            armature=0.01,
        ),
        "ankles": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            effort_limit_sim=50.0,
            velocity_limit_sim=30.0,
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "waist": ImplicitActuatorCfg(
            joint_names_expr=["waist_.*_joint"],
            effort_limit_sim=60.0,
            velocity_limit_sim=32.0,
            stiffness=100.0,
            damping=2.0,
            armature=0.01,
        ),
        "upper_arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
            effort_limit_sim=60.0,
            velocity_limit_sim=37.0,
            stiffness=40.0,
            damping=2.0,
            armature=0.01,
        ),
        "forearms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
            ],
            effort_limit_sim=33.0,
            velocity_limit_sim=37.0,
            stiffness=20.0,
            damping=1.0,
            armature=0.01,
        ),
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_.*_joint"],
            effort_limit_sim=20.0,
            velocity_limit_sim=37.0,
            stiffness=20.0,
            damping=1.0,
            armature=0.01,
        ),
    },
    soft_joint_pos_limit_factor=0.9,
    # Keep this in the R1JointIndex order from the Unitree SDK2 R1 example.
    # Unitree RL Lab uses it when writing params/deploy.yaml.
    joint_sdk_names=[
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_roll_joint",
        "waist_yaw_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "head_pitch_joint",
        "head_yaw_joint",
    ],
)
