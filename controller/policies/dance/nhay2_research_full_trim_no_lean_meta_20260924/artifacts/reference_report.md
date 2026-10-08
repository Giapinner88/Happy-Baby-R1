# Reference report — `nhay2_r1_robot_feasible_trimmed.npz`

```
==============================================================================
REFERENCE INSPECTION  —  nhay2_r1_robot_feasible_trimmed.npz
==============================================================================

[1] CO BAN
  fps            : 50.0
  so frame       : 785
  thoi luong     : 15.70 s
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
  x range        : [ -0.080,   0.043] m
  y range        : [ -0.076,   0.312] m
  chieu cao z    : min 0.734  mean 0.758  max 0.801 m
  duong di ngang : 1.47 m

[3] NGHIENG THAN  (+ = nga sau, - = cui truoc; yaw-independent)
  pelvis      lean mean  -2.08  median  -2.46  max   2.09  min  -5.57  | up-tilt mean  3.59  [OK  ]
  torso_link  lean mean  -2.38  median  -2.70  max   1.69  min  -6.00  | up-tilt mean  3.50  [OK  ]

[4] TIEP DAT BAN CHAN
  left ankle  z  : min 0.055  mean 0.077 m
  right ankle z  : min 0.056  mean 0.080 m
  chan thap nhat : 0.055 m  (motion tiep dat tot: ~0.025-0.046 m)
  %frame cham dat: 52.7%  (nguong ankle z<=0.06)
  reference stance: left 74.4% / right 74.4% / either 100.0% (FK collision-height + velocity + hysteresis)

[5] TUNG KHOP: range (deg) + bien do con lai toi gioi han
  joint                          min     max   range  margin_lo margin_hi  flag
  left_hip_pitch_joint         -35.0     1.7    36.6      133.0     144.3  
  left_hip_roll_joint           -6.8    14.5    21.4       53.2      85.5  
  left_hip_yaw_joint             0.0    22.6    22.6      157.0     134.4  
  left_knee_joint               -0.2    61.0    61.2        9.8      78.0  
  left_ankle_pitch_joint       -26.8     1.1    27.9       23.2      31.9  
  left_ankle_roll_joint         -5.4     1.5     6.9        9.6      13.5  
  right_hip_pitch_joint        -41.8    -3.0    38.7      126.2     149.0  
  right_hip_roll_joint         -19.0     4.1    23.1       81.0      55.9  
  right_hip_yaw_joint          -23.9     3.3    27.2      133.1     153.7  
  right_knee_joint               0.1    68.9    68.8       10.1      70.1  
  right_ankle_pitch_joint      -16.0     7.1    23.1       34.0      25.9  
  right_ankle_roll_joint        -0.9    10.0    10.9       14.1       5.0  
  waist_roll_joint             -11.5    11.2    22.8       18.5      18.8  
  waist_yaw_joint              -18.8    11.8    30.6      131.2     138.2  
  left_shoulder_pitch_joint     -8.1    20.1    28.2      171.9      99.9  
  left_shoulder_roll_joint      10.1    33.8    23.7       23.1     108.2  
  left_shoulder_yaw_joint      -34.8     0.0    34.8       75.2     110.0  
  left_elbow_joint             -33.3    82.6   115.9       22.6      42.6  
  left_wrist_roll_joint        -20.6     7.6    28.2       89.4     102.4  
  right_shoulder_pitch_joint    -8.4    20.1    28.4      171.6      99.9  
  right_shoulder_roll_joint    -40.3   -10.3    29.9      101.8      23.3  
  right_shoulder_yaw_joint      -4.3    36.7    40.9      105.7      73.3  
  right_elbow_joint            -34.9    72.3   107.2       21.0      52.9  
  right_wrist_roll_joint        -6.7    22.5    29.2      103.3      87.5  

[6] VAN TOC
  base lin vel   : max 0.56 m/s  mean 0.11
  joint vel      : max 7.78 rad/s (446 deg/s)

[7] LIEN TUC (buoc nhay khop frame-to-frame)
  buoc nhay max  : 8.9 deg tai left_elbow_joint
  #frame nhay lon: 0  (nguong 15.0 deg)

==============================================================================
KET LUAN: PASS — khong phat hien bat thuong ro rang.
==============================================================================
```
