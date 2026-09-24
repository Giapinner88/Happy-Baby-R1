import math

# Standard reflected inertia formula at joint output:
# J_joint = J_m * (N_1 * N_2)**2 + J_1 * N_2**2 + J_2
# where:
# rotors = (J_m, J_1, J_2)
# gears = (1, N_1, N_2) (gears[0] is always 1)
def compute_armature(rotors, gears):
    J_m, J_1, J_2 = rotors
    N_1, N_2 = gears[1], gears[2]
    N_total = N_1 * N_2
    return J_m * (N_total**2) + J_1 * (N_2**2) + J_2

# 7520_22:
rotors_7520_22 = (0.489e-4, 0.109e-4, 0.738e-4)
gears_7520_22 = (1, 4.5, 5)
armature_7520_22 = compute_armature(rotors_7520_22, gears_7520_22)

# 7520_14:
rotors_7520_14 = (0.489e-4, 0.098e-4, 0.533e-4)
gears_7520_14 = (1, 4.5, 1.0 + (48.0 / 22.0))
armature_7520_14 = compute_armature(rotors_7520_14, gears_7520_14)

# 5020:
rotors_5020 = (0.139e-4, 0.017e-4, 0.169e-4)
gears_5020 = (1, 1.0 + (46.0 / 18.0), 1.0 + (56.0 / 16.0))
armature_5020 = compute_armature(rotors_5020, gears_5020)

# 4010:
# ROTOR_INERTIAS_4010 = (0.068e-4, 0, 0)
# GEARS_4010 = (1, 5, 5)
rotors_4010 = (0.068e-4, 0.0, 0.0)
gears_4010 = (1, 5, 5)
armature_4010 = compute_armature(rotors_4010, gears_4010)

print(f"Armatures:")
print(f"  7520_22: {armature_7520_22:.6f}")
print(f"  7520_14: {armature_7520_14:.6f}")
print(f"  5020:    {armature_5020:.6f}")
print(f"  4010:    {armature_4010:.6f}")

# Natural freq
nat_freq = 10 * 2.0 * math.pi

stiffness_7520_22 = armature_7520_22 * (nat_freq**2)
stiffness_7520_14 = armature_7520_14 * (nat_freq**2)
stiffness_5020 = armature_5020 * (nat_freq**2)
stiffness_4010 = armature_4010 * (nat_freq**2)

# Action scale = 0.25 * effort_limit / stiffness
scale_7520_22 = 0.25 * 139.0 / stiffness_7520_22
scale_7520_14 = 0.25 * 88.0 / stiffness_7520_14
scale_5020 = 0.25 * 25.0 / stiffness_5020
scale_4010 = 0.25 * 5.0 / stiffness_4010

# Waist and ankle use 5020 * 2:
# scale_waist = 0.25 * (25.0*2) / (stiffness_5020*2) = scale_5020
scale_waist = scale_5020
scale_ankle = scale_5020

print(f"\nCalculated Action Scales (0.25 * effort / stiffness):")
print(f"  7520_22 (hip roll, knees): {scale_7520_22:.6f}")
print(f"  7520_14 (hip pitch, hip yaw, waist yaw): {scale_7520_14:.6f}")
print(f"  5020 (shoulders, elbows, wrist roll, waist roll/pitch, ankles): {scale_5020:.6f}")
print(f"  4010 (wrist pitch, wrist yaw): {scale_4010:.6f}")
