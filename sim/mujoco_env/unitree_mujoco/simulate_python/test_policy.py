"""
test_policy.py — Kiểm tra offline policy output với NPZ và logic torso quat đúng.
Chạy: python test_policy.py
"""
import numpy as np
import onnxruntime as ort

# === Config ===
POLICY_PATH = "policy_motion_data.onnx"
NPZ_PATH = "motions/motion_data.npz"

DEFAULT_Q = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0,
                       0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0], dtype=np.float32)
ACTION_SCALE = np.array([0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44,
                          0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07], dtype=np.float32)

JOINT_NAMES = [
    "left_hip_pitch","left_hip_roll","left_hip_yaw","left_knee","left_ankle_pitch","left_ankle_roll",
    "right_hip_pitch","right_hip_roll","right_hip_yaw","right_knee","right_ankle_pitch","right_ankle_roll",
    "waist_yaw","waist_roll","waist_pitch",
    "left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw","left_elbow","left_wrist_roll","left_wrist_pitch","left_wrist_yaw",
    "right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw","right_elbow","right_wrist_roll","right_wrist_pitch","right_wrist_yaw",
]

# === Quaternion helpers ===
def quat_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)

def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=np.float32)

def quat_to_rot_mat(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ], dtype=np.float32)

def yaw_quat(q):
    w, x, y, z = q
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    c, s = np.cos(yaw/2), np.sin(yaw/2)
    return np.array([c, 0.0, 0.0, s], dtype=np.float32)

def angle_axis_quat(angle, axis):
    c, s = np.cos(angle/2), np.sin(angle/2)
    return np.array([c, s*axis[0], s*axis[1], s*axis[2]], dtype=np.float32)

def compute_torso_quat(pelvis_q, waist_yaw, waist_roll, waist_pitch):
    q = quat_mul(pelvis_q, angle_axis_quat(waist_yaw,   [0,0,1]))
    q = quat_mul(q,        angle_axis_quat(waist_roll,  [1,0,0]))
    q = quat_mul(q,        angle_axis_quat(waist_pitch, [0,1,0]))
    return q

# === Load NPZ ===
print(f"Loading NPZ: {NPZ_PATH}")
d = np.load(NPZ_PATH)
joint_pos_npz  = d["joint_pos"].astype(np.float32)
joint_vel_npz  = d["joint_vel"].astype(np.float32)
root_quat_npz  = d["body_quat_w"][:, 0, :].astype(np.float32)  # pelvis, wxyz
num_frames = joint_pos_npz.shape[0]
print(f"  Frames: {num_frames}, joints: {joint_pos_npz.shape[1]}")

# === Load ONNX ===
session = ort.InferenceSession(POLICY_PATH, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name
print(f"ONNX input: {input_name}, shape: {session.get_inputs()[0].shape}")

# === init_quat (perfect tracking: robot = ref at frame 0) ===
ref_torso_q0 = compute_torso_quat(
    root_quat_npz[0],
    joint_pos_npz[0, 12], joint_pos_npz[0, 13], joint_pos_npz[0, 14]
)
ref_yaw_q = yaw_quat(ref_torso_q0)
# Giả sử robot bắt đầu đúng với motion (robot_yaw = ref_yaw)
# => init_quat = robot_yaw * ref_yaw^{-1} = ref_yaw * ref_yaw^{-1} = identity
init_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # identity

print(f"\nChạy policy với {num_frames} frames (perfect tracking)...\n")

last_action = np.zeros(29, dtype=np.float32)
actions_history = []

for frame_idx in range(num_frames):
    ref_q  = joint_pos_npz[frame_idx]
    ref_dq = joint_vel_npz[frame_idx]
    ref_root_q = root_quat_npz[frame_idx]

    # Assume perfect tracking: robot = ref
    # motion_command
    motion_command = np.concatenate([ref_q, ref_dq])

    # torso quats (perfect: real = ref)
    ref_torso_q  = compute_torso_quat(ref_root_q, ref_q[12], ref_q[13], ref_q[14])
    real_torso_q = ref_torso_q  # perfect tracking

    target_torso_q = quat_mul(init_quat, ref_torso_q)
    q_rel = quat_mul(quat_inv(target_torso_q), real_torso_q)
    R = quat_to_rot_mat(q_rel).T  # transpose như C++
    motion_anchor_ori_b = np.array([R[0,0],R[0,1], R[1,0],R[1,1], R[2,0],R[2,1]], dtype=np.float32)

    gyro = np.zeros(3, dtype=np.float32)
    q_rel_rl = ref_q - DEFAULT_Q
    dq_rl = ref_dq.copy()

    obs = np.concatenate([motion_command, motion_anchor_ori_b, gyro, q_rel_rl, dq_rl, last_action]).astype(np.float32)

    action = session.run(None, {input_name: np.expand_dims(obs, 0)})[0][0]
    actions_history.append(action.copy())
    last_action = action.copy()

actions_history = np.array(actions_history)

print(f"{'Idx':>3} {'Joint':<26} {'Action_min':>10} {'Action_max':>10} {'Action_mean':>12} {'Target_min':>10} {'Target_max':>10}")
print("-" * 85)
for i in range(29):
    col = actions_history[:, i]
    tgt_min = DEFAULT_Q[i] + col.min() * ACTION_SCALE[i]
    tgt_max = DEFAULT_Q[i] + col.max() * ACTION_SCALE[i]
    print(f"[{i:2d}] {JOINT_NAMES[i]:<26} {col.min():+10.3f} {col.max():+10.3f} {col.mean():+12.3f} {tgt_min:+10.3f} {tgt_max:+10.3f}")
