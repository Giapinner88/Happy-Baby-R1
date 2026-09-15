# D003 — Keep high-level as the sole R1 lowcmd owner

Status: selected for suspended arms/head hardware pilot  
Date: 2026-08-18

## Context

The existing high-level service publishes a complete `rt/lowcmd` message at
500 Hz. The first hardware teleop adapter independently published another
complete `rt/lowcmd`. DDS does not arbitrate those writers; their messages can
interleave, and the high-level gamepad/E-stop path cannot reliably override
the second writer.

## Decision

`hardware/high_level/` remains the only process allowed to publish
`rt/lowcmd`. A local sidecar sends a versioned 12-float arms/head target over
UDP bound only to loopback. High-level acts on it only in ZERO TORQUE,
preserves R3 E-stop priority, applies session-relative, rate and freshness
guards, and returns arm/head to zero torque on release or fault.

The authoritative IPC order is left arm[5], right arm[5], head pitch, head
yaw. R1-A5 motor indices are arms 15–19/22–26, head pitch 29, head yaw 30. The
wire packet stores its final two floats as head yaw, then head pitch; the
sidecar performs that explicit boundary conversion.

## Alternatives

- Two simultaneous `rt/lowcmd` publishers: rejected because there is no
  ownership or packet arbitration.
- Pause or stop high-level during teleop: rejected because it removes the R3
  software E-stop path and the required zero-torque owner.
- Publish `rt/arm_sdk` beside high-level: rejected by the bounded suspended
  pilot, where encoder movement was negligible in Dev Mode.
- A network-visible motor socket: rejected because the sidecar already runs on
  the robot; the UDP receiver binds only `127.0.0.1`.

## Consequences

- `hardware/teleop/src/teleop/hardware/run_teleop.py` is read-only preflight.
- The sidecar may subscribe to `rt/lowstate` for validation/evidence but cannot
  create a DDS publisher.
- A fresh lowstate sample anchors current robot encoders after the first valid
  target arrives; every selected joint remains within ±0.15 rad of that anchor.
- Hardware evidence remains valid only for a robot suspended on its fixture;
  this decision does not authorize on-ground operation.
- Previous direct-lowcmd runs remain historical plumbing evidence and are not
  comparable ownership evidence for this architecture.

## Validation

Require parser tests, source assertions excluding teleop DDS publishers, x86
and aarch64 high-level builds, read-only service/socket checks, then a bounded
suspended run with an R3 E-stop operator. Evidence must show target and encoder
traces, normal deadman release, high-level remaining active, and no second DDS
motor publisher.

## Reversal condition

Revisit if Unitree supplies a verified controller-arbitration API, if the
high-level loop cannot meet its 500 Hz schedule with IPC polling, or if
hardware evidence shows incorrect joint mapping, unsafe latency, watchdog
failure or E-stop priority failure.
