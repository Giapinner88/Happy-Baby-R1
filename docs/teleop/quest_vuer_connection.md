# Quest -> vuer -> teleop connection

Reviewed 2026-08-18 against `t007_whole_upper_body_20260818T114338Z`.

## Path

```text
Quest 3 (Quest Browser, immersive WebXR session)
      | HTTPS + WSS, port 8012, HB-Hotspot
      v
quest_bridge.py            conda env `tv`, Python 3.10, vuer 0.0.60
      | newline-delimited R1TeleopCommand JSON on stdout, 30 Hz
      v
run_r1_quest3_live.py      conda env `unitree_sim_env`, Python 3.11, IsaacLab
```

The two environments cannot share an interpreter (`vuer` and IsaacLab conflict),
so they are separate processes joined by a pipe. Both run on one host, so
`time.monotonic()` is a shared timebase and command age is measured directly
rather than estimated.

## Verified state

| Item | Value |
|---|---|
| Host IP | `10.42.0.1` on `wlp77s0`, connection `HB-Hotspot` |
| Certificate SAN | `IP Address:10.42.0.1` — matches the host IP |
| Certificate validity | to 2026-09-01 |
| `tv` env | vuer 0.0.60, websockets 16.0, `TeleVuerWrapper` imports |
| Server bind | port 8012 listens when the wrapper is constructed |
| Vendor tree | `third_party/xr_teleoperate` |

`scripts/check_vuer.sh` in `hardware/teleop/` re-runs all of these.

## Rate mismatch is by design

The bridge emits at 30 Hz while the simulator loop achieved 9.98 Hz. This is not
a backlog: each control step drains the whole queue and keeps only the newest
command, discarding the rest, so the consumer always acts on the freshest pose.
Observed fresh-command age was 17.6 ms mean and 66 ms maximum, which is
consistent with latest-wins rather than a growing queue. The limiting factor is
IK compute, not transport.

## Reading the drop counter

The last run recorded `dropped_sample_count=1927` against
`emitted_command_count=3457`. That is not packet loss. A sample is dropped when
`motion_data_ready` is clear or the pose has not changed for
`max_pose_stale_s`; 1927 samples at 30 Hz is the 64 s the operator spent before
entering the immersive session. The session then held one connection for 115 s
with `rejected_sample_count=0`, meaning every pose matrix passed the
orthonormality and determinant checks.

Judge the connection by `connect_count`, `disconnect_count` and
`rejected_sample_count`, not by the drop counter.

## Known gaps

- Opening the page is not enough: without entering the immersive VR session the
  bridge waits at `motion_data_ready=0` and emits nothing.
- Two vendor trees are present. The bridge uses `xr_teleoperate`, while the
  upper-body IK compatibility tests load the R1-A5 reference asset from
  `xr_teleoperate_v1_6`. They are pinned separately and must not be assumed
  interchangeable.
- The robot at `10.42.0.33` was not on the hotspot during this review, and no
  DHCP reservation was found for it, so that address is not guaranteed stable.
