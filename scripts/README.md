# Workspace entry points

Nhánh này chỉ duy trì `scripts/teleop/`. Chạy từ repository root qua `make`
hoặc theo [README teleop](teleop/README.md); không gọi vendor script trực tiếp
khi đã có wrapper của workspace.

Các launcher training, MuJoCo, DDS bridge và asset synchronization cũ đã được
loại vì implementation/config tương ứng không còn thuộc nhánh `teleop`.
