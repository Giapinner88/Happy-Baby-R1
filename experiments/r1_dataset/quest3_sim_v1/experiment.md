# R1 Quest 3 dataset collection — simulation v1

**Mode:** `simulation_only`. Không có DDS, không có Unitree SDK, không có
hardware claim trong bất kỳ protocol nào ở đây.

## Câu hỏi của experiment

Có thu được dataset học bắt chước dùng được cho R1, từ teleop Quest 3 trong
Isaac Sim, ở định dạng chuyển thẳng sang LeRobot không?

## Quan hệ với `r1_teleop/quest3_sim_v1`

Experiment này **tiêu thụ** đường teleop đã được nghiên cứu ở `r1_teleop`
(T001–T008) chứ không nghiên cứu lại nó. Bộ giải IK, hợp đồng frame, deadman và
chính sách fail-closed đều thuộc về `r1_teleop`; ở đây chúng là điều kiện có
sẵn. Nếu một run dataset phát hiện lỗi ở tầng teleop thì lỗi đó thuộc về
`r1_teleop`, và run dataset bị loại.

Ranh giới:

| Thuộc `r1_teleop` | Thuộc `r1_dataset` |
|---|---|
| IK, rate limit, workspace projection | Vật thể trong scene, nhãn task |
| Deadman, hold, watchdog | Ranh giới episode, tiêu chí loại episode |
| Bằng chứng bám sát mục tiêu | Ảnh quan sát, cặp state/action |

## Protocols

| Protocol | Nội dung | Trạng thái |
|---|---|---|
| `d001` | Dataset "chỉ tay vào vật" — hai tay + đầu, không end-effector | Đã khai báo, chưa có run |

`d002` (thao tác có gripper) chưa được mở và **sẽ không được mở** cho tới khi R1
có end-effector trong asset. Xem [PLAN.md](PLAN.md) Giai đoạn 5.

## Kế hoạch

[PLAN.md](PLAN.md) — sáu giai đoạn, mỗi giai đoạn một cổng đo được.

## Layout

```
D001/
├── D001.md      hồ sơ protocol: định nghĩa, tiêu chí hợp lệ, thủ tục chạy
├── config/      cấu hình khai báo, có thể sửa
└── runs/        bằng chứng bất biến, một thư mục cho mỗi lần chạy (gitignored)
```
