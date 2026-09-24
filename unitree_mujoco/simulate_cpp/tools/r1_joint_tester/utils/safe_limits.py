"""Safe joint limits for R1 robot (in degrees)"""

# Safe limits: General joints at 90% range, Head at max range
# Shoulder roll clamped to avoid hip collision
SAFE_LIMITS_DEG = [
    [-152.0, 130.0],  # 0: left_hip_pitch
    [-52.0, 92.0],    # 1: left_hip_roll
    [-141.0, 141.0],  # 2: left_hip_yaw
    [-2.0, 131.0],    # 3: left_knee
    [-45.0, 28.0],    # 4: left_ankle_pitch
    [-13.0, 13.0],    # 5: left_ankle_roll
    [-152.0, 130.0],  # 6: right_hip_pitch
    [-92.0, 52.0],    # 7: right_hip_roll
    [-141.0, 141.0],  # 8: right_hip_yaw
    [-2.0, 131.0],    # 9: right_knee
    [-45.0, 28.0],    # 10: right_ankle_pitch
    [-13.0, 13.0],   # 11: right_ankle_roll
    [-27.0, 27.0],   # 12: waist_roll
    [-135.0, 135.0], # 13: waist_yaw
    [-165.0, 105.0], # 14: left_shoulder_pitch
    [5.0, 134.0],    # 15: left_shoulder_roll (Clamped min to 5.0 to avoid hip collision)
    [-99.0, 99.0],   # 16: left_shoulder_yaw
    [-46.0, 116.0],  # 17: left_elbow
    [-99.0, 99.0],   # 18: left_wrist_roll
    [-165.0, 105.0], # 19: right_shoulder_pitch
    [-134.0, -5.0],  # 20: right_shoulder_roll (Clamped max to -5.0 to avoid hip collision)
    [-99.0, 99.0],   # 21: right_shoulder_yaw
    [-46.0, 116.0],  # 22: right_elbow
    [-99.0, 99.0],   # 23: right_wrist_roll
    [-20.0, 20.0],   # 24: head_pitch (Max raw range)
    [-34.0, 34.0]    # 25: head_yaw (Max raw range)
]

# Convert to radians for internal use
import math
SAFE_LIMITS_RAD = [[math.radians(lim[0]), math.radians(lim[1])] for lim in SAFE_LIMITS_DEG]

# Temperature thresholds (in Celsius)
TEMP_WARNING = 50.0
TEMP_CRITICAL = 65.0

# Velocity limits (rad/s) - max safe velocity
VEL_LIMITS_LEG_WAIST = 1.0   # ~57 deg/s
VEL_LIMITS_ARM_HEAD = 1.5    # ~86 deg/s

def get_vel_limit(joint_id):
    """Get velocity limit for a joint"""
    if joint_id < 14 or joint_id in [24, 25]:  # Legs, waist, head
        return VEL_LIMITS_LEG_WAIST
    return VEL_LIMITS_ARM_HEAD

def get_temp_status(temp):
    """Get temperature status color"""
    if temp >= TEMP_CRITICAL:
        return "critical"
    elif temp >= TEMP_WARNING:
        return "warning"
    return "normal"
