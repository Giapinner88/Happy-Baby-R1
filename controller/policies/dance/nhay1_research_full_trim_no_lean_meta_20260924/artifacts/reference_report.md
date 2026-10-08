# Reference report — `nhay1_r1_robot_feasible_trimmed.npz`

```
==============================================================================
REFERENCE INSPECTION  —  nhay1_r1_robot_feasible_trimmed.npz
==============================================================================

[1] CO BAN
  fps            : 50.0
  so frame       : 1244
  thoi luong     : 24.88 s
  so body        : 25
  so dof         : 24

[1b] ENTRY HOME BRIDGE
  frames/seconds : 75/1.50
  standing root z: 0.734039 m

[1c] EXIT HOME BRIDGE / HOLD
  bridge/hold    : 75/15 frames
  bridge seconds : 1.50
  standing root z: 0.734039 m

[2] QUY DAO GOC (pelvis)
  x range        : [ -0.046,   0.052] m
  y range        : [ -0.102,   0.111] m
  chieu cao z    : min 0.690  mean 0.741  max 0.772 m
  duong di ngang : 1.30 m

[3] NGHIENG THAN  (+ = nga sau, - = cui truoc; yaw-independent)
  pelvis      lean mean  -2.28  median  -1.78  max   3.79  min  -7.92  | up-tilt mean  3.91  [OK  ]
  torso_link  lean mean  -2.27  median  -1.99  max   4.46  min  -7.93  | up-tilt mean  4.91  [OK  ]

[4] TIEP DAT BAN CHAN
  left ankle  z  : min 0.055  mean 0.064 m
  right ankle z  : min 0.056  mean 0.068 m
  chan thap nhat : 0.055 m  (motion tiep dat tot: ~0.025-0.046 m)
  %frame cham dat: 71.7%  (nguong ankle z<=0.06)
  reference stance: left 92.0% / right 93.0% / either 99.7% (FK collision-height + velocity + hysteresis)

[5] TUNG KHOP: range (deg) + bien do con lai toi gioi han
  joint                          min     max   range  margin_lo margin_hi  flag
  left_hip_pitch_joint         -40.3     3.7    44.0      127.7     142.3  
  left_hip_roll_joint           -1.7    18.9    20.6       58.3      81.1  
  left_hip_yaw_joint            -2.0    24.0    26.1      155.0     133.0  
  left_knee_joint               -0.7    73.2    73.9        9.3      65.8  
  left_ankle_pitch_joint       -15.3     3.6    18.9       34.7      29.4  
  left_ankle_roll_joint        -10.0     4.6    14.6        5.0      10.4  
  right_hip_pitch_joint        -41.9    -0.8    41.1      126.1     146.8  
  right_hip_roll_joint         -21.8     0.0    21.8       78.2      60.0  
  right_hip_yaw_joint          -24.9     5.3    30.3      132.1     151.7  
  right_knee_joint               0.5    74.4    73.9       10.5      64.6  
  right_ankle_pitch_joint      -18.4     4.1    22.5       31.6      28.9  
  right_ankle_roll_joint        -0.4    10.0    10.4       14.6       5.0  
  waist_roll_joint              -9.8     7.9    17.7       20.2      22.1  
  waist_yaw_joint              -15.6    12.0    27.6      134.4     138.0  
  left_shoulder_pitch_joint    -70.2    25.7    95.9      109.8      94.3  
  left_shoulder_roll_joint      10.3   109.7    99.4       23.3      32.3  
  left_shoulder_yaw_joint      -51.1    17.7    68.8       58.9      92.3  
  left_elbow_joint             -37.9    65.9   103.8       18.0      59.3  
  left_wrist_roll_joint        -41.2    20.2    61.4       68.8      89.8  
  right_shoulder_pitch_joint   -52.5    27.5    80.0      127.5      92.5  
  right_shoulder_roll_joint    -83.8   -10.3    73.5       58.2      23.3  
  right_shoulder_yaw_joint       0.0    48.6    48.6      110.0      61.4  
  right_elbow_joint            -50.9    57.9   108.8        5.0      67.3  
  right_wrist_roll_joint       -17.9    40.7    58.6       92.1      69.3  

[6] VAN TOC
  base lin vel   : max 0.39 m/s  mean 0.06
  joint vel      : max 6.58 rad/s (377 deg/s)

[7] LIEN TUC (buoc nhay khop frame-to-frame)
  buoc nhay max  : 7.7 deg tai left_knee_joint
  #frame nhay lon: 0  (nguong 15.0 deg)

==============================================================================
KET LUAN: PASS — khong phat hien bat thuong ro rang.
==============================================================================
```
