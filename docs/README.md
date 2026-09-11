# Tài liệu Happy Baby R1 Teleop

Nhánh này chỉ duy trì tài liệu cho Quest 3 teleop tay/đầu trên R1-A5. Nội dung
locomotion, training, MuJoCo, ROS onboarding và các protocol T001–T006/T008 đã
được loại khỏi active workspace; Git history vẫn giữ lịch sử của chúng.

| Nhu cầu | Nguồn authoritative |
|---|---|
| Chạy teleop trong Isaac | [Runbook mô phỏng](teleop/r1_quest3_teleop_sim.md) |
| Chuẩn bị/chạy pilot treo | [Runbook hardware](teleop/r1_quest3_teleop_hardware.md) |
| Kiểm tra điều kiện còn thiếu | [Hardware gate](../hardware/teleop/docs/hardware_gate.md) |
| Hiểu frame, IK và tracking | [Bộ tài liệu phương pháp](teleop/README.md) |
| Quy tắc an toàn | [Safety index](safety/README.md) |
| Entrypoint và diagnostics | [Scripts teleop](../scripts/teleop/README.md) |
| Cấu hình đối chứng active | [Baseline config](../experiments/r1_teleop/quest3_sim_v1/baseline/config/README.md) |

Thông số runtime thuộc file config hoặc run đã resolve; tài liệu chỉ giải thích
ngữ nghĩa và dẫn tới owner đó. Không dùng tài liệu lịch sử thay cho trạng thái
hardware gate hiện tại.
