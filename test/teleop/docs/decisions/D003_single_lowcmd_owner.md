# D003 — Keep high-level as the sole R1 lowcmd owner

Status: selected for suspended arms/head hardware pilot  
Date: 2026-08-18

## Context

The existing high-level service publishes a complete `rt/lowcmd` Damping
message at 500 Hz while IDLE. The first hardware teleop adapter independently
published another complete `rt/lowcmd`. DDS does not arbitrate those writers;
their messages can interleave, and the high-level gamepad/E-stop path cannot
reliably override the second writer.

## Decision

`hardware/high_level/` remains the only process allowed to publish
`rt/lowcmd`. A local sidecar sends a versioned 12-float arms/head target over
UDP bound only to loopback. High-level acts on it only in ZERO TORQUE,
preserves R3 E-stop priority, applies session-relative, rate and freshness
guards, and returns arm/head to zero torque on release or fault.

The authoritative IPC order is left arm[5], right arm[5], head yaw, head
pitch. R1-A5 motor indices are arms 15–19/22–26, head yaw 29, head pitch 30.

## Alternatives

- Two simultaneous `rt/lowcmd` publishers: rejected because there is no
  ownership or packet arbitration.
- Pause or stop high-level during teleop: rejected because it removes the R3
  software E-stop path and the required zero-torque owner.
- Publish `rt/arm_sdk` beside high-level: rejected by the bounded suspended
  pilot, where encoder movement was negligible in Dev Mode.
- A network-visible motor socket: rejected because the sidecar already runs on
  the robot; the UDP receiver binds only `127.0.0.1`.

## Rationale

One complete motor-command owner makes priority deterministic. The sidecar can
fail or lose SSH without leaving an independent DDS writer: high-level detects
stale IPC, ramps down the selected gains, and resumes its existing Zero Torque
loop. Locomotion, dance and normal high-level state transitions keep their
existing command path.

## Consequences

- High-level gains a small ZERO-TORQUE-only teleop path inside its existing loop.
- The sidecar may subscribe to `rt/lowstate` for validation/evidence but cannot
  create a DDS publisher.
- A first target anchors both Quest source zero and current robot encoders;
  every selected joint remains within ±0.15 rad of that anchor and 0.30 rad/s.
- Hardware evidence remains valid only for a robot suspended on its fixture;
  this decision does not authorize on-ground operation.
- The previous direct-lowcmd smoke runs remain historical plumbing evidence
  but are not comparable ownership evidence for this architecture.

## Validation

Require parser tests, a source assertion that exactly one `rt/lowcmd`
publisher exists, a clean x86 and aarch64 build, read-only service/socket
checks, then a bounded suspended run with R3 E-stop operator. Evidence must
show target and observed encoder traces, normal deadman release, high-level
remaining active, and no second DDS motor publisher.

## Reversal condition

Revisit if Unitree supplies a verified controller-arbitration API, if the
high-level loop can no longer meet its 500 Hz schedule with IPC polling, or if
hardware evidence shows incorrect joint mapping, unsafe latency, watchdog
failure or E-stop priority failure.
