# Baseline — R1 Quest 3 arms/head teleop

## Câu hỏi

Pipeline Quest 3 → initial-head anchor → upstream `R1_A5_ArmIK` → Isaac/R1 có
tạo target tay/đầu hữu hạn, đúng thứ tự khớp và fail-closed hay không?

## Phạm vi

- [`baseline/`](baseline/README.md) là protocol active duy nhất và là đối chứng
  vendor bất biến cho source code do dự án phát triển sau này.
- Base/locomotion, chân điều khiển, training và child interaction ngoài phạm vi.
- Simulation không tạo hardware authority. Hardware phải qua
  [runbook](../../../docs/teleop/r1_quest3_teleop_hardware.md) và
  [gate](../../../hardware/teleop/docs/hardware_gate.md).

Run hoàn chỉnh được giữ nguyên dù pass, fail hay inconclusive. Một run canonical
chỉ hoàn tất khi manifest xác nhận đủ data, metrics, figures và video.
