import os
from pathlib import Path

ROBOT = os.environ.get("ROBOT", "g1") # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
REPO_ROOT = Path(__file__).resolve().parents[2]
UNITREE_MUJOCO_ROOT = Path(
    os.environ.get("UNITREE_MUJOCO_ROOT", REPO_ROOT / "third_party" / "unitree_mujoco")
).expanduser()
ROBOT_SCENE = str(UNITREE_MUJOCO_ROOT / "unitree_robots" / ROBOT / "scene.xml")
DOMAIN_ID = int(os.environ.get("DOMAIN_ID", "1")) # Domain id
INTERFACE = os.environ.get("INTERFACE", "lo") # Interface

USE_JOYSTICK = os.environ.get("USE_JOYSTICK", "0").lower() in {"1", "true", "yes"} # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "switch" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer
