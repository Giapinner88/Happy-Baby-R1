"""
Script kiểm tra file NPZ và so sánh với CSV.
Chạy từ .venv: python check_npz.py
"""
import numpy as np

NPZ_PATH = "motions/motion_data.npz"
CSV_PATH = "motions/motion_data.csv"

print("=" * 60)
print("1. CẤU TRÚC FILE NPZ")
print("=" * 60)
d = np.load(NPZ_PATH)
for k in d.files:
    print(f"  {k}: shape={d[k].shape}, dtype={d[k].dtype}")

joint_pos_npz = d["joint_pos"]   # shape: (T, N_joints)
joint_vel_npz = d["joint_vel"]
num_frames_npz = joint_pos_npz.shape[0]
num_joints_npz = joint_pos_npz.shape[1]

print(f"\n  Tổng frames NPZ: {num_frames_npz}")
print(f"  Số joints trong NPZ: {num_joints_npz}")

print("\n" + "=" * 60)
print("2. PHÂN TÍCH joint_pos TRONG NPZ (khung đầu tiên)")
print("=" * 60)
JOINT_NAMES_RL = [
    "left_hip_pitch",      "left_hip_roll",       "left_hip_yaw",
    "left_knee",           "left_ankle_pitch",    "left_ankle_roll",
    "right_hip_pitch",     "right_hip_roll",      "right_hip_yaw",
    "right_knee",          "right_ankle_pitch",   "right_ankle_roll",
    "waist_yaw",           "waist_roll",          "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll",  "left_shoulder_yaw",
    "left_elbow",          "left_wrist_roll",     "left_wrist_pitch",  "left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow",         "right_wrist_roll",    "right_wrist_pitch", "right_wrist_yaw",
]

frame0 = joint_pos_npz[0]
print(f"  Khung 0 ({len(frame0)} values):")
if len(frame0) == 29:
    for i, (name, val) in enumerate(zip(JOINT_NAMES_RL, frame0)):
        print(f"    [{i:2d}] {name:25s} = {val:+.4f}")
else:
    # Nhiều hơn 29 joints, có thể chứa nhiều joint hơn (theo thứ tự MuJoCo)
    print(f"  NPZ có {len(frame0)} joints (KHÔNG PHẢI 29!)")
    for i, val in enumerate(frame0):
        print(f"    [{i:2d}] = {val:+.4f}")

print("\n" + "=" * 60)
print("3. PHÂN TÍCH joint_pos TRONG CSV (cột 7:36)")
print("=" * 60)
csv_data = np.loadtxt(CSV_PATH, delimiter=",")
dof_pos_csv = csv_data[:, 7:36]
num_frames_csv = dof_pos_csv.shape[0]
print(f"  Tổng frames CSV: {num_frames_csv}")
print(f"  Số DOF CSV (cols 7-35): {dof_pos_csv.shape[1]}")

frame0_csv = dof_pos_csv[0]
print(f"\n  Khung 0 CSV DOF:")
if len(frame0_csv) == 29:
    for i, (name, val) in enumerate(zip(JOINT_NAMES_RL, frame0_csv)):
        print(f"    [{i:2d}] {name:25s} = {val:+.4f}")

print("\n" + "=" * 60)
print("4. SO SÁNH: NPZ vs CSV (khung đầu, 29 joints)")
print("=" * 60)
if len(frame0) == 29 and len(frame0_csv) == 29:
    print(f"  {'Idx':>3} {'Joint':25s}  {'NPZ':>10}  {'CSV':>10}  {'Diff':>10}")
    print(f"  {'-'*65}")
    for i in range(29):
        diff = frame0[i] - frame0_csv[i]
        flag = " ← KHÁC!" if abs(diff) > 0.05 else ""
        print(f"  [{i:2d}] {JOINT_NAMES_RL[i]:25s}  {frame0[i]:+10.4f}  {frame0_csv[i]:+10.4f}  {diff:+10.4f}{flag}")
else:
    print(f"  NPZ joints={len(frame0)}, CSV DOF={len(frame0_csv)} -> Khác số lượng!")
    print(f"  NPZ first frame: {frame0}")
    print(f"  CSV first frame: {frame0_csv}")

print("\n" + "=" * 60)
print("5. THỐNG KÊ MIN/MAX NPZ joint_pos")
print("=" * 60)
if len(frame0) == 29:
    for i in range(29):
        col = joint_pos_npz[:, i]
        print(f"  [{i:2d}] {JOINT_NAMES_RL[i]:25s}: min={col.min():+.4f}, max={col.max():+.4f}")
