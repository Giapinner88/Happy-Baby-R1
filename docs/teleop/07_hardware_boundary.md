# 07 — Hardware Boundary

## 1. Accepted status

```text
accepted method for suspended arms/head pilot only
```

Quyết định sole-owner trước đây nằm trong Git history; contract active được
giữ tại file này và được kiểm tra bởi test tĩnh của `hardware/teleop/`.

## 2. Hardware architecture

```text
Quest/Vuer
→ IK on workstation
→ JSONL/SSH
→ loopback UDP sidecar
→ high-level ZERO TORQUE arbitration
→ sole rt/lowcmd publisher
```

`hb_high_level` là **DDS motor writer duy nhất**.

Loopback endpoint:

```text
127.0.0.1:5560
```

## 3. UTL1 packet

```text
60-byte little-endian
magic = 0x314c5455
sequence
enable
arm_valid
head_valid
arm_q[10]
head_yaw
head_pitch
```

Motor slots:

| Group | IDL slots |
|---|---|
| left arm | 15–19 |
| right arm | 22–26 |
| head pitch | 29 |
| head yaw | 30 |

## 4. Accepted pilot envelope

$$
\boxed{
q_{des,i}
=
q_{0,i}
+
\operatorname{clamp}
(s_i-s_{0,i},-e_i,+e_i)
}
$$

Hardware pilot assumptions:
- R1-A5;
- `mode_machine=1`;
- robot suspended/fixed;
- dedicated R3 E-stop operator;
- initial pose mechanically valid;
- launcher envelope relative to anchor: ±1.0 rad normally and ±3.2 rad for the
  six shoulder joints; asset joint limits still apply.

Không claim:
- collision avoidance;
- balance;
- trajectory tracking accuracy;
- floor operation.

## 5. Rule

Không bypass sole-lowcmd-owner architecture khi thử hardware.
