from scipy.spatial.transform import Rotation as R
import numpy as np

# Let's say robot is pitched forward by 30 degrees (rotation around Y axis)
# quat in scalar-last (x,y,z,w)
r = R.from_euler('y', 30, degrees=True)
q_xyzw = r.as_quat()
x, y, z, w = q_xyzw
print(f"Quat: w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}")

# Projected gravity is R^T * [0, 0, -1]^T
g_world = np.array([0, 0, -1])
g_proj_true = r.inv().apply(g_world)
print("True projected gravity:", g_proj_true)

gx = 2 * (w * y - x * z)
gy = -2 * (y * z + w * x)
gz = 2 * (x**2 + y**2) - 1
print("My projected gravity:  ", [gx, gy, gz])

