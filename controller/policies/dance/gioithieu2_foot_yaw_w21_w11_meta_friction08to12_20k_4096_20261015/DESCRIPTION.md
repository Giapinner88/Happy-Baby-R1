# Gioithieu2 — clip-specific foot-yaw correction

Meta tracking policy trained from the canonical 50 Hz R1 motion created from
`gioithieu2.mp4`.  The motion uses the GMR W21/W21/W11 post-processing chain
and an isolated, constant bilateral ankle-yaw bias correction calibrated from
the 0.5--4.5 s standing reference.  It is not a change to the shared GVHMR or
GMR pipeline.

Training uses `Unitree-R1-Tracking-No-State-Estimation`, 2,048 environments,
20,000 iterations, seed 42, and foot-friction domain randomization [0.8, 1.2].
The package is inactive: it is installed for review and hardware acceptance,
but does not change any existing HB dance slot.

The MP3 is extracted from the exact source video, delayed 1.5 s to align with
the canonical entry bridge, and padded to the 131.4 s R1 motion timeline.
