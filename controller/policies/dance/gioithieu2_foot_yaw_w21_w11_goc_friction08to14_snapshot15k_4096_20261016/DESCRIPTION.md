# Gioithieu2 — GOC snapshot at iteration 15,000

This inactive HB dance package is a test snapshot from the still-running
Gioithieu2 GOC training run. It is not the final 20,000-iteration policy.

The motion is the isolated Gioithieu2 clip-specific foot-yaw correction:
canonical W21/W21/W11 postprocessing at 50 Hz, 6,570 frames. Training uses
4,096 scene and terrain environments, seed 42, and randomized foot friction
in [0.8, 1.4]. Tracking covers the pelvis, both leg chains, torso, and both arm
chains; end-effector termination covers both ankles and wrists.

The package remains inactive so it can be reviewed before replacing an active
HB dance slot.
