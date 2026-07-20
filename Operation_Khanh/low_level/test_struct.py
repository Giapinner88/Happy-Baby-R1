import struct
import numpy as np

try:
    fmt = '<BB26f'
    target_q = np.zeros(26, dtype=np.float32)
    # Without tolist()
    data = struct.pack(fmt, 1, 0, *target_q)
    print("Success without tolist!")
except Exception as e:
    print("Error without tolist:", e)

try:
    data = struct.pack(fmt, 1, 0, *target_q.tolist())
    print("Success with tolist!")
except Exception as e:
    print("Error with tolist:", e)
