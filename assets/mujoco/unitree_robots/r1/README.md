# R1 MJCF runtime model

`R1.xml` cùng mesh và các `scene_*.xml` là model duy nhất dùng chung cho:

- huấn luyện/đánh giá MJLab qua overlay `training/`;
- kiểm tra scene bằng `scripts/simulation/run_r1_mujoco_model.py`;
- DDS bridge/policy runtime trong `sim/unitree_mujoco_policy/`.

`scene_hanging.xml` là scene debug an toàn; `scene_stairs.xml`,
`scene_slope.xml` và `scene_obstacles.xml` là biến thể địa hình.
