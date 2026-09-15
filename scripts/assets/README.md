# Asset maintenance entry points

`sync_r1_mujoco_asset.py` lấy model R1 từ vendor MJLab, bổ sung thành phần
runtime cần thiết và cập nhật `assets/mujoco/unitree_robots/r1/`. Review diff
MJCF và chạy viewer trước khi dùng policy/bridge.
