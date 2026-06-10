import csv

# Joint names order in G1:
JOINT_NAMES = [
    "left_hip_pitch",     "left_hip_roll",     "left_hip_yaw",
    "left_knee",          "left_ankle_pitch",  "left_ankle_roll",
    "right_hip_pitch",    "right_hip_roll",    "right_hip_yaw",
    "right_knee",         "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw",          "waist_roll",        "waist_pitch",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw",
    "left_elbow",         "left_wrist_roll",   "left_wrist_pitch",  "left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw",
    "right_elbow",        "right_wrist_roll",  "right_wrist_pitch", "right_wrist_yaw",
]

# Load CSV using standard library
rows = []
with open("motions/motion_data.csv", "r") as f:
    reader = csv.reader(f)
    for r in reader:
        rows.append([float(x) for x in r])

print(f"Loaded {len(rows)} rows.")

for idx, name in enumerate(JOINT_NAMES):
    col = [r[7 + idx] for r in rows]
    col_min = min(col)
    col_max = max(col)
    col_mean = sum(col) / len(col)
    print(f"Col {7 + idx} ({name}): min={col_min:.4f}, max={col_max:.4f}, mean={col_mean:.4f}")
