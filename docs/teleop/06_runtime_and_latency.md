# 06 — Runtime, Rate and Latency

## 1. Reviewed metadata

Metadata lịch sử T007 được review:

```text
Reviewed 2026-08-18 against `t007_whole_upper_body_20260818T114338Z`.
```

Verified state:

| Item | Value |
|---|---|
| Host IP | `10.42.0.1` on `wlp77s0`, connection `HB-Hotspot` |
| Certificate SAN | `IP Address:10.42.0.1` |
| Certificate validity của run lịch sử | to 2026-09-01; không còn giá trị cho phiên mới |
| `tv` env | vuer 0.0.60, websockets 16.0 |
| Server bind | port 8012 |
| Vendor tree | `third_party/xr_teleoperate` |

## 2. Observed performance

- bridge emits: **30 Hz**
- simulator achieved: **9.98 Hz**
- queue policy: **latest-wins**
- mỗi control step drain toàn queue
- fresh-command age:
  - mean: **17.6 ms**
  - max: **66 ms**

Observed limiting factor:

```text
IK compute, not transport
```

## 3. Last run metadata

```text
dropped_sample_count = 1927
emitted_command_count = 3457
connection duration = 115 s
rejected_sample_count = 0
```

`dropped_sample_count` không đồng nghĩa packet loss; phần lớn là thời gian trước khi operator vào immersive session.

Đánh giá connection bằng:
- `connect_count`
- `disconnect_count`
- `rejected_sample_count`

không dùng riêng `dropped_sample_count`.

## 4. Current bottleneck

Pipeline active dùng upstream solver và dispatch 10 Hz trên hardware path.
Mỗi run mới phải ghi solve time, command age và achieved rate; không dùng số đo
coupled lịch sử để mô tả performance hiện tại.
