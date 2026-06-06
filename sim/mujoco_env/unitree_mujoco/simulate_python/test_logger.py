import numpy as np
import time
from fail_detector.logger import IMULogger

logger = IMULogger(prefix="test_plot", window_before=1, window_after=1)
for i in range(100):
    t = time.perf_counter()
    g = np.array([0, 0, -1])
    gyro = np.array([1, 2, 3])
    accel = np.array([0, 0, 9.8])
    dq = np.array([0.1, 0.2, 35.0])
    logger.log_step(t, g, gyro, accel, dq)
    if i == 50:
        logger.trigger_fail_event(t)
    time.sleep(0.02)

time.sleep(2)
print("Done")
