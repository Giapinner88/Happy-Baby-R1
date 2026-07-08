#!/usr/bin/env python3
"""Sync the local R1 MuJoCo asset from Unitree RL MJLab's training model.

This keeps runtime MuJoCo and MJLab training on the same MJCF/collision model
without modifying third_party. The local runtime keeps its scene files, while
R1.xml and meshes are refreshed from the training source.
"""

from __future__ import annotations

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TRAIN_R1_ROOT = ROOT / "third_party" / "unitree_rl_mjlab" / "src" / "assets" / "robots" / "unitree_r1"
TRAIN_XML = TRAIN_R1_ROOT / "xmls" / "r1.xml"
TRAIN_MESH_DIR = TRAIN_R1_ROOT / "xmls" / "assets"
LOCAL_R1_ROOT = ROOT / "asset" / "mujoco" / "unitree_robots" / "r1"
LOCAL_XML = LOCAL_R1_ROOT / "R1.xml"
LOCAL_MESH_DIR = LOCAL_R1_ROOT / "meshes"

R1_MOTOR_JOINTS = [
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
]


def main() -> None:
    if not TRAIN_XML.exists():
        raise FileNotFoundError(f"Missing training R1 XML: {TRAIN_XML}")
    if not TRAIN_MESH_DIR.exists():
        raise FileNotFoundError(f"Missing training R1 mesh directory: {TRAIN_MESH_DIR}")

    LOCAL_R1_ROOT.mkdir(parents=True, exist_ok=True)
    LOCAL_MESH_DIR.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(TRAIN_XML)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", "meshes")

    pelvis = root.find(".//body[@name='pelvis']")
    if pelvis is None:
        raise ValueError("Training R1 XML does not contain body name='pelvis'")
    if pelvis.find("site[@name='pelvis_hang_site']") is None:
        ET.SubElement(
            pelvis,
            "site",
            {
                "name": "pelvis_hang_site",
                "pos": "0 0 0.18",
                "size": "0.025",
                "rgba": "1 0.2 0.2 1",
            },
        )

    valid_bodies = {body.attrib["name"] for body in root.iter("body") if "name" in body.attrib}
    contact = root.find("contact")
    if contact is not None:
        for exclude in list(contact.findall("exclude")):
            body1 = exclude.attrib.get("body1")
            body2 = exclude.attrib.get("body2")
            if body1 not in valid_bodies or body2 not in valid_bodies:
                contact.remove(exclude)

    for section_name in ("actuator", "sensor"):
        section = root.find(section_name)
        if section is not None:
            root.remove(section)

    actuator = ET.SubElement(root, "actuator")
    for joint in R1_MOTOR_JOINTS:
        name = joint.removesuffix("_joint")
        if "ankle" in joint:
            ctrlrange = "-50 50"
        elif "shoulder_yaw" in joint or "elbow" in joint or "wrist_roll" in joint:
            ctrlrange = "-33 33"
        else:
            ctrlrange = "-60 60"
        ET.SubElement(actuator, "motor", {"name": name, "joint": joint, "ctrlrange": ctrlrange})

    sensor = ET.SubElement(root, "sensor")
    for joint in R1_MOTOR_JOINTS:
        name = joint.removesuffix("_joint")
        ET.SubElement(sensor, "jointpos", {"name": f"{name}_pos", "joint": joint})
    for joint in R1_MOTOR_JOINTS:
        name = joint.removesuffix("_joint")
        ET.SubElement(sensor, "jointvel", {"name": f"{name}_vel", "joint": joint})
    for joint in R1_MOTOR_JOINTS:
        name = joint.removesuffix("_joint")
        ET.SubElement(sensor, "jointactuatorfrc", {"name": f"{name}_torque", "joint": joint})
    ET.SubElement(sensor, "framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu"})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_acc", "site": "imu"})
    ET.SubElement(sensor, "framepos", {"name": "frame_pos", "objtype": "site", "objname": "imu"})
    ET.SubElement(sensor, "framelinvel", {"name": "frame_vel", "objtype": "site", "objname": "imu"})

    ET.indent(tree, space="  ")
    tree.write(LOCAL_XML, encoding="utf-8", xml_declaration=False)

    for src in sorted(TRAIN_MESH_DIR.iterdir()):
        if src.is_file():
            shutil.copy2(src, LOCAL_MESH_DIR / src.name)

    print(f"Synced R1 XML: {LOCAL_XML}")
    print(f"Synced R1 meshes: {LOCAL_MESH_DIR}")


if __name__ == "__main__":
    main()
