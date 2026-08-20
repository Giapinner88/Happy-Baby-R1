# Quest 3 G1/Dex3 legacy reference

This document preserves the boundary of the older vendor G1/Dex3 Quest flow.
It is not an R1 teleoperation procedure, is not an active workspace entry
point, and makes no claim of R1 compatibility.

The current R1 simulation-only procedure is
[r1_quest3_teleop_sim.md](r1_quest3_teleop_sim.md). It uses the project-owned
`R1TeleopCommand` schema, does not send DDS commands, and does not call
`hardware/high_level/`.

## What the legacy flow referenced

- `third_party/unitree_sim_isaaclab` for Unitree simulator examples.
- `third_party/xr_teleoperate` for Vuer/Quest acquisition and G1/Dex3
  retargeting examples.
- G1/H1 vendor tasks/action providers, not an R1 task or R1 command sink.

`third_party/` remains read-only. Consult the checked-out vendor README and
source for an upstream version when historical comparison is necessary; do not
copy its DDS path into R1 v1.

## External machine settings

Do not store host IPs, certificates, private keys, or machine-specific paths
in this repository. A legacy investigation must supply them outside the repo,
for example:

```bash
export HB_R1_ROOT="/path/to/Happy-Baby-R1"
export XR_TELEOP_HOST="quest-host.example-or-lan-address"
export XR_TELEOP_CERT="/secure/external/path/cert.pem"
export XR_TELEOP_KEY="/secure/external/path/key.pem"
```

Certificates and keys stay outside the repository. This legacy material is
excluded from R1 experiments, policy promotion, and any hardware decision.
