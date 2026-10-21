# Doremon — Meta retrain, friction 0.8–1.4, 8,192 environments

Final 20,000-iteration Meta policy retrained from the `video1.npz` motion in
the existing `doremon_meta_new_pipeline_20260924_20k` HB package. The motion
contains 4,560 frames at 50 Hz.

Training uses 8,192 scene and terrain environments, seed 42, and randomized
foot friction in [0.8, 1.4]. Tracking covers the pelvis, both leg chains,
torso, and both arm chains; end-effector termination covers both ankles and
wrists.

This package is installed inactive pending visual review and hardware testing.
