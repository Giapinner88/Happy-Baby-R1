# Reference report — `nhay3_r1_20260910.npz`

```
==============================================================================
REFERENCE INSPECTION  —  nhay3_r1_20260910.npz
==============================================================================

[1] CO BAN
  fps            : 50.0
  so frame       : 1161
  thoi luong     : 23.22 s
  so body        : 25
  so dof         : 24

[2] QUY DAO GOC (pelvis)
  x range        : [ -0.088,   0.027] m
  y range        : [ -0.039,   0.333] m
  chieu cao z    : min 0.734  mean 0.742  max 0.751 m
  duong di ngang : 1.82 m

[3] NGHIENG THAN  (+ = nga sau, - = cui truoc; yaw-independent)
  pelvis      lean mean  -1.12  median  -1.60  max   4.85  min  -4.11  | up-tilt mean  3.23  [OK  ]
  torso_link  lean mean  -1.36  median  -1.71  max   4.49  min  -4.11  | up-tilt mean  2.92  [OK  ]

[4] TIEP DAT BAN CHAN (ankle_roll_link z)
  left ankle  z  : min 0.056  mean 0.072 m
  right ankle z  : min 0.056  mean 0.068 m
  chan thap nhat : 0.056 m  (motion tiep dat tot: ~0.025-0.046 m)
  %frame cham dat: 97.1%  (nguong ankle z<=0.06)

[5] TUNG KHOP: range (deg) + bien do con lai toi gioi han
  joint                          min     max   range  margin_lo margin_hi  flag
  left_hip_pitch_joint         -51.6    15.3    66.9      116.4     130.7  
  left_hip_roll_joint           -8.5    15.8    24.2       51.5      84.2  
  left_hip_yaw_joint           -15.3    35.2    50.5      141.7     121.8  
  left_knee_joint                0.2    97.7    97.5       10.2      41.3  
  left_ankle_pitch_joint       -17.0    -0.2    16.8       33.0      33.2  
  left_ankle_roll_joint        -10.3     1.4    11.7        4.7      13.6  
  right_hip_pitch_joint        -48.6    12.6    61.2      119.4     133.4  
  right_hip_roll_joint         -18.8     5.6    24.4       81.2      54.4  
  right_hip_yaw_joint          -38.8    11.2    50.0      118.2     145.8  
  right_knee_joint              -0.2    87.7    87.9        9.8      51.3  
  right_ankle_pitch_joint      -13.9     0.7    14.6       36.1      32.3  
  right_ankle_roll_joint         0.0    10.4    10.4       15.0       4.6  
  waist_roll_joint              -6.9     9.9    16.9       23.1      20.1  
  waist_yaw_joint              -27.2    22.5    49.7      122.8     127.5  
  left_shoulder_pitch_joint    -14.1    25.6    39.7      165.9      94.4  
  left_shoulder_roll_joint      10.3    59.5    49.1       23.3      82.5  
  left_shoulder_yaw_joint      -45.9    46.1    92.0       64.1      63.9  
  left_elbow_joint             -38.7    76.4   115.1       17.2      48.8  
  left_wrist_roll_joint        -24.7    19.1    43.8       85.3      90.9  
  right_shoulder_pitch_joint   -20.3    23.9    44.1      159.7      96.1  
  right_shoulder_roll_joint    -63.3   -10.3    53.0       78.7      23.3  
  right_shoulder_yaw_joint     -67.2    43.7   110.9       42.8      66.3  
  right_elbow_joint            -51.4    75.7   127.1        4.5      49.5  
  right_wrist_roll_joint       -22.4    12.3    34.7       87.6      97.7  

[6] VAN TOC
  base lin vel   : max 0.28 m/s  mean 0.08
  joint vel      : max 8.27 rad/s (474 deg/s)

[7] LIEN TUC (buoc nhay khop frame-to-frame)
  buoc nhay max  : 9.6 deg tai left_knee_joint
  #frame nhay lon: 0  (nguong 15.0 deg)

==============================================================================
KET LUAN: PASS — khong phat hien bat thuong ro rang.
==============================================================================
```
