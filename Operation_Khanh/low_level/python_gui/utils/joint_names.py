"""Joint definitions for R1 robot (26 joints)"""

# Joint names matching Unitree SDK
JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_roll", "waist_yaw",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
    "head_pitch", "head_yaw"
]

# Map from joint index (0-25) to Unitree SDK motor index
JOINT_IDX = [
    0, 1, 2, 3, 4, 5,       # Left leg joints 0-5 → motors 0-5
    6, 7, 8, 9, 10, 11,    # Right leg joints 6-11 → motors 6-11
    12, 13,                 # Waist joints 12-13 → motors 12-13
    14, 15, 16, 17, 18,    # Left arm joints 14-18 → motors 14-18
    22, 23, 24, 25, 26,    # Right arm joints 19-23 → motors 22-26
    29, 30                  # Head joints 24-25 → motors 29-30
]

# Joint groups for UI layout (joint indices 0-25)
JOINT_GROUPS = {
    "LEFT LEG": list(range(0, 6)),
    "RIGHT LEG": list(range(6, 12)),
    "WAIST": [12, 13],
    "LEFT ARM": list(range(14, 19)),
    "RIGHT ARM": list(range(19, 24)),
    "HEAD": [24, 25]
}

# Human-readable joint names for display
JOINT_DISPLAY_NAMES = {
    0: "L.Hip Pitch",
    1: "L.Hip Roll",
    2: "L.Hip Yaw",
    3: "L.Knee",
    4: "L.Ankle Pitch",
    5: "L.Ankle Roll",
    6: "R.Hip Pitch",
    7: "R.Hip Roll",
    8: "R.Hip Yaw",
    9: "R.Knee",
    10: "R.Ankle Pitch",
    11: "R.Ankle Roll",
    12: "Waist Roll",
    13: "Waist Yaw",
    14: "L.Shoulder Pitch",
    15: "L.Shoulder Roll",
    16: "L.Shoulder Yaw",
    17: "L.Elbow",
    18: "L.Wrist Roll",
    19: "R.Shoulder Pitch",
    20: "R.Shoulder Roll",
    21: "R.Shoulder Yaw",
    22: "R.Elbow",
    23: "R.Wrist Roll",
    24: "Head Pitch",
    25: "Head Yaw"
}
