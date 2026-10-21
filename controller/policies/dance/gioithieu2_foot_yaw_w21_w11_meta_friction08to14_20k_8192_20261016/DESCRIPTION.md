# Gioithieu2 — Meta, friction 0.8–1.4, 8,192 environments

Final 20,000-iteration Meta policy trained from the clip-specific Gioithieu2
foot-yaw-corrected motion at 50 Hz (6,570 frames).

Training uses 8,192 scene and terrain environments, seed 42, and randomized
foot friction in [0.8, 1.4]. Tracking covers the pelvis, both leg chains,
torso, and both arm chains; end-effector termination covers both ankles and
wrists.

This package is installed inactive pending visual review and hardware testing.
